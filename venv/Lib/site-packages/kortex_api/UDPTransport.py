import errno
import inspect
import select
import socket
import sys
import threading
import time

from kortex_api.ITransportClient import ITransportClient
from .Exceptions.KException import KException
from .Exceptions.KClientException import KClientException

class UDPTransport(ITransportClient):

    address = ('', 0)

    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_transport_thread = UDPTransportThread()
        self.udp_transport_thread.daemon = True
        self.event = threading.Event()

        # 65535 - 20 (ip header) - 8 (udp header) = 65507 bytes
        self.maxTxBufferSize = 65507

        self.sock.setblocking(0)
        super().__init__()

    def __del__(self):
        self.event.set()
        if self.udp_transport_thread.is_alive():
            self.udp_transport_thread.join(timeout=2)

    def connect(self, ip, port):
        # Connect to ip
        self.address = (ip, port)
        try:
            self.sock.connect(self.address)
            self.udp_transport_thread.setup(sock=self.sock, event=self.event)
            self.udp_transport_thread.start()
        except OSError as ex:
            self.sock.close()
            print("[{}.{}] ERROR: {}".format(self.__class__.__name__, inspect.stack()[0][3], ex.strerror), file=sys.stderr, flush=True)
            raise ex

    def disconnect(self):
        self.event.set()
        self.udp_transport_thread.join()
        self.sock.close()

    def send(self, payload):
        try:
            # Send data from payload until either all data has been sent or an error occurs.
            self.sock.sendall(payload)

        except Exception as ex:
            print("[{}.{}] ERROR: {}".format(self.__class__.__name__, inspect.stack()[0][3], ex.strerror), file=sys.stderr, flush=True)
            raise ex

    def getMaxTxBufferSize(self):
        return self.maxTxBufferSize

    def registerOnFrameCallback(self, callback):
        if callback is None:
            raise KException("Frame callback registration error: callback is null")
        else:
            self.udp_transport_thread.set_frame_callback(callback)

class UDPTransportThread(threading.Thread):
    def __init__(self):
        self.sock = None
        self.event = None
        self.onFrameCallback = None
        super().__init__()

    def setup(self, sock, event):
        self.sock = sock
        self.event = event

    def set_frame_callback(self, cb):
        self.onFrameCallback = cb

    def run(self):
        while not self.event.is_set():
            try:
                '''
                    The dead code in this block was conserved because it had a faster response time 
                    on a wired network. While on Wi-Fi, the code using select performed roughly 10 ms slower
                    over 150 iterations of BaseCyclic.RefreshFeedback.
                    
                    If ever we need an extra boost this would be a good place to investigate.
                    CL 

                    # 65535 - 20 (ip header) - 8 (udp header) = 65507 bytes
                    #data, address = self.sock.recvfrom(65507)
                    #if len(data) > 0:
                '''

                # SELECT with a 500 ms timeout
                readable, writable, exceptional = select.select([self.sock], [], [], 0.5)
                
                # If the socket is not ready to be read, we continue to the next loop cycle.
                if (readable or writable or exceptional):
                    if self.onFrameCallback:
                        try:
                            # 65535 - 20 (ip header) - 8 (udp header) = 65507 bytes
                            data = self.sock.recv(65507)
                        
                        except Exception as exception:
                            print("recv() failed with {}".format(exception))
                            
                            continue    # abort current receive loop iteration
            
                        self.onFrameCallback(data)

            except IOError as e:
                # Error occurred in SELECT
                print("select()  failed with {}".format(e))
