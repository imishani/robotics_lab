import sys
import threading
from .autogen.messages import Frame_pb2 as FramePb  # NOQA
from .autogen.messages import Errors_pb2 as ErrorsPb  # NOQA
from . import BitMaskTools

from google.protobuf.message import DecodeError

from .Exceptions.KClientException import KClientException
from .Exceptions.KException import KException
from .FrameHandler import FrameHandler
from functools import partial
from concurrent.futures import Future

class RouterClientSendOptions:

    def __init__(self):
        self.andForget = False
        self.delay_ms = 0
        self.timeout_ms = 10000

    def getTimeoutInSecond(self):
        return self.timeout_ms / 1000


class RouterClient:
    notificationService = {}

    def __init__(self, transport, errorCallback=None):
        self.transport = transport
        self.sessionId = 0
        self.m_isActive = True
        self.callback = partial(self.onFrameCallback)
        self.transport.registerOnFrameCallback(self.callback)
        self.errorCallback = errorCallback
        self.frameHandler = FrameHandler()
        self.m_hitCallback = None

        # If no error callback is provided, default is the basic one
        if self.errorCallback is None:
            self.errorCallback = self.basicErrorCallback

    # @staticmethod
    def basicErrorCallback(self, kException):
        print("[{}] {}".format(self.__class__.__name__, kException), file=sys.stderr)

    def SetActivationStatus(self, isActive):
        self.m_isActive = isActive

    
    def send(self, payloadData, serviceVersion, functionUid, deviceId, options:RouterClientSendOptions=None):
        
        if not self.m_isActive:
            future = Future()
            future.set_exception(KClientException(ErrorsPb.ERROR_PROTOCOL_CLIENT, "Activation Status set to False"))
            return future
      
        payloadMsgFrame, msgFrameId, future = self.frameHandler.registerFrame(payloadData, serviceVersion, functionUid, deviceId)

        maxSize = self.transport.getMaxTxBufferSize()
        
        if maxSize and len(payloadMsgFrame) > maxSize:
            clientException = KClientException( ErrorsPb.TOO_LARGE_ENCODED_FRAME_BUFFER,
                                                "Serialized message data is bigger than maximum acceptable size: size={0} > max={1}" 
                                                .format(len(payloadMsgFrame), self.transport.maxTxBufferSize))
            self.frameHandler.forcePromiseException(msgFrameId, clientException)
        
        else:

            try:
                self.transport.send(payloadMsgFrame)

                if self.m_hitCallback is not None:
                    self.m_hitCallback(FramePb.MSG_FRAME_REQUEST)
            
            except:

                clientException = KClientException(ErrorsPb.ERROR_PROTOCOL_CLIENT, "Couldn't send message frame")
                self.frameHandler.forcePromiseException(msgFrameId, clientException)
                raise

        return future

    def getConnectionId(self):
        return self.sessionId

    def onFrameCallback(self, databuffer):
        if self.m_isActive:
            frame = FramePb.Frame()
            clientException = None

            try:

                frame.ParseFromString(databuffer)

                frameType = BitMaskTools.extractFrameType(frame.header.frame_info)
                header = frame.header
                
                if (frameType == FramePb.MSG_FRAME_RESPONSE):
                    if self.m_hitCallback is not None:
                        self.m_hitCallback(frameType)

                    errStatus = self.frameHandler.manageReceivedFrame(frame)

                    if (errStatus.error_code != ErrorsPb.ERROR_NONE):
                        clientException = KClientException.createFromError(errStatus)

                elif (frameType == FramePb.MSG_FRAME_NOTIFICATION):
                    serviceId = BitMaskTools.extractServiceId(header.service_info)
                    if list(self.notificationService.keys()).count(serviceId) > 0:
                        notifCallback = self.notificationService.get(serviceId)
                        notifCallback(frame)
                    else:
                        # Notif callback not found
                        clientException = KClientException(ErrorsPb.UNREGISTERED_NOTIFICATION_RECEIVED,
                                                        "Received a notification for a service which we aren't subscribed to: serviceID={0}"
                                                        .format(serviceId))
                else:
                    # Notify error
                    clientException = KClientException(ErrorsPb.UNSUPPORTED_FRAME_TYPE,
                                                    "Unrecognized frame type received {0}"
                                                    .format(frameType))
            except DecodeError:
                clientException = KClientException(ErrorsPb.FRAME_DECODING_ERR, "Couldn't decode message frame")

            if clientException and self.errorCallback is not None:
                self.errorCallback(clientException)

            # Store the sessionId of the first valid frame received
            if (clientException == None and self.sessionId == 0):
                self.sessionId = self.frameHandler.sessionId

    def registerNotificationCallback(self, serviceId, notificationService):
        self.notificationService[serviceId] = notificationService

    def registerHitCallback(self, cb):
        self.m_hitCallback = cb
