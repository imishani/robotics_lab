import inspect
import os
import sys
import threading
import time
from concurrent.futures import TimeoutError

from .autogen.messages import Frame_pb2 as FramePb  # NOQA
from .autogen.messages import Errors_pb2 as ErrorsPb  # NOQA

from .Exceptions.KClientException import KClientException
from .Exceptions.KServerException import KServerException

from .UDPTransport import UDPTransport

from .autogen.client_stubs import SessionClientRpc as Session
from .RouterClient import RouterClient, RouterClientSendOptions

def DropConnectionTimeoutCallback():
    file_basename = '.'.join(os.path.basename(__file__).split('.')[:-1])    # strip extension (.py)
    print("[{}] Connection timeout detected ({})".format(file_basename, time.time()), file=sys.stderr)

class SessionManager(Session.SessionClient):

    def __init__(self, router: RouterClient, connectionTimeoutCallback = DropConnectionTimeoutCallback):
        router.registerHitCallback(self.Hit)
        self.m_connectionTimeoutCallback = connectionTimeoutCallback    # Callback notifying the application that current 'User Session' lost communication (no Rx) longer than 'inactivityTimeout_ms'
        self.m_keepAliveInterval_ms = 150
        self.event = threading.Event()
        self.m_thread = SessionValidationThread(self.event, self)
        self.m_thread.daemon = True
        self.m_hasBeenSent = False
        self.m_hasBeenReceived = False
        super().__init__(router)

    def __del__(self):
        self.event.set()
        if self.m_thread.is_alive():
            self.m_thread.join(timeout=2)


    def _checkTransport(self):
        """Raise KClientException if trying to create a session using UDP on port other than realtime port"""

        RT_PORT = 10001

        if isinstance(self.router.transport, UDPTransport):

            ipAddress = self.router.transport.address[0]
            port = self.router.transport.address[1]
            
            if port != RT_PORT:
                raise KClientException(
                    ErrorsPb.UNSUPPORTED_NETWORK_TYPE,
                    """
Since 2.0 release, UDPTransport is no longer supported except on port %d (reserved for cyclic control). TCPTransport must be used instead:
    from kortex_api.TCPTransport import TCPTransport
    transport = TCPTransport()
    transport.connect("%s", %d)
""" % (RT_PORT, ipAddress, port))


    def CreateSession(self, createSessionInfo, options:RouterClientSendOptions=RouterClientSendOptions()):

        try:
            super().CreateSession(createSessionInfo, options=options)
            self.m_inactivityTimeout_ms = createSessionInfo.session_inactivity_timeout
            self.m_thread.start()

        except TimeoutError:
            # TODO Remove this check after 2.0 release
            self._checkTransport()
            # reraise Timeout if no other exception was raised
            raise


    def CloseSession(self, options:RouterClientSendOptions=RouterClientSendOptions()):
        super().router.m_hitCallback = None     # Unregister hit callback from router
        self.event.set()
        time.sleep(0.010)
        # try:
        #     if self.m_thread.is_alive():
        #         self.m_thread.join(timeout=30)
        # except Exception as ex:
        #     print("[{}.{}] join() failed with: {}".format(self.__class__.__name__, inspect.stack()[0][3], ex), file=sys.stderr, flush=True)

        try:
            super().CloseSession(options=options)
            # TODO: Implement 'send and forget' in Python API and use it here
            # super().CloseSession(options = {True, 0, 0})  # send and forget
        except TimeoutError:
            pass
        except Exception as ex:
            print("[{}.{}] super().CloseSession() failed with: {}".format(self.__class__.__name__, inspect.stack()[0][3], ex), file=sys.stderr, flush=True)

    def Hit(self, hit_type: FramePb.FrameTypes):
        if hit_type is FramePb.MSG_FRAME_RESPONSE:
            self.m_hasBeenReceived = True
        elif hit_type is FramePb.MSG_FRAME_REQUEST:
            self.m_hasBeenSent = True

class SessionValidationThread(threading.Thread):
    def __init__(self, event, parent):
        self.event = event
        self.parent = parent
        super().__init__()

    def run(self):
        try:
            lastRcvTimeStamp = time.time();
            lastSendTimeStamp = time.time();

            options = RouterClientSendOptions()
            options.timeout_ms = self.parent.m_keepAliveInterval_ms * 0.80

            keepAliveInterval_sec = self.parent.m_keepAliveInterval_ms / 1000
            inactivityTimeout_sec = self.parent.m_inactivityTimeout_ms / 1000   # Session inactivity timeout in second

            while not self.event.is_set():
                now = time.time()
                
                if not self.parent.m_hasBeenSent and (now - lastSendTimeStamp) > keepAliveInterval_sec:
                    lastSendTimeStamp = time.time() # reset the time stamp because missing non-blocking version
                    try:
                        self.parent.KeepAlive(options=options)
                    except TimeoutError:
                        pass
                elif self.parent.m_hasBeenSent:
                    lastSendTimeStamp = time.time()
                    self.parent.m_hasBeenSent = False

                if not self.parent.m_hasBeenReceived and inactivityTimeout_sec and (now - lastRcvTimeStamp) > inactivityTimeout_sec:
                    lastRcvTimeStamp = time.time()      # to make sure we don't trig the connection timeout callback every iteration after the first timeout
                    self.parent.m_connectionTimeoutCallback()
                elif self.parent.m_hasBeenReceived:
                    lastRcvTimeStamp = time.time()
                    self.parent.m_hasBeenReceived = False

                time.sleep(0.010)   # 10 milliseconds

        except KServerException as e:
            print(e)
