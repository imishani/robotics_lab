import errno
import inspect
import select
import socket
import sys
import threading
import array
import time

from .ITransportClient import ITransportClient
from .Exceptions.KException import KException
from .Exceptions.KClientException import KClientException

class KinovaTCPSplitter:
    def __init__(self):
        self.KINOVA_MAGIC_STRING = str('\x07xEtRoK\x07')
        self.buffer = bytes(self.KINOVA_MAGIC_STRING, 'utf-8')
        self.KINOVA_HEADER_SIZE = len(self.KINOVA_MAGIC_STRING) + 4

    def prepend_header(self, payload):
        self.buffer = bytes(self.KINOVA_MAGIC_STRING, 'utf-8')
        payload_length = len(payload)
        data_size = payload_length.to_bytes(4, byteorder="big")

        self.buffer += data_size
        self.buffer += payload
        return self.buffer

    def parse_header(self, buff):
        bytes_to_read = 0
        magic_string = buff[0:len(self.KINOVA_MAGIC_STRING)]

        if self.KINOVA_MAGIC_STRING == str(magic_string, 'utf-8'):
            bytes_to_read = bytes_to_read.from_bytes(buff[len(self.KINOVA_MAGIC_STRING): self.KINOVA_HEADER_SIZE]
                                                     , byteorder="big")
        return bytes_to_read


class TCPTransport(ITransportClient):
    address = ('', 0)

    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.splitter = KinovaTCPSplitter()
        self.tcp_transport_thread = TCPTransportThread(self.splitter)
        self.tcp_transport_thread.daemon = True
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.event = threading.Event()
        super().__init__()

    def __del__(self):
        self.event.set()
        if self.tcp_transport_thread.is_alive():
            self.tcp_transport_thread.join(timeout=2)

    def connect(self, ip, port):
        # Connect to ip
        self.address = (ip, port)
        try:
            self.sock.connect(self.address)
            self.tcp_transport_thread.setup(sock=self.sock, event=self.event)
            self.tcp_transport_thread.start()
        except OSError as ex:
            self.sock.close()
            print("[{}.{}] ERROR: {}".format(self.__class__.__name__, inspect.stack()[0][3], ex.strerror),
                  file=sys.stderr, flush=True)
            raise ex

    def disconnect(self):
        self.event.set()
        self.tcp_transport_thread.join()
        self.sock.shutdown(socket.SHUT_RDWR)
        self.sock.close()

    def send(self, payload):
        try:
            payload = self.splitter.prepend_header(payload)

            # Send data from payload until either all data has been sent or an error occurs.
            self.sock.sendall(payload)

        except Exception as ex:
            print("[{}.{}] ERROR: {}".format(self.__class__.__name__, inspect.stack()[0][3], print(ex)),
                  file=sys.stderr, flush=True)
            raise ex


    def getMaxTxBufferSize(self):
        return None

    def registerOnFrameCallback(self, callback):
        if callback is None:
            raise KException("Frame callback registration error: callback is null")
        else:
            self.tcp_transport_thread.set_frame_callback(callback)


class TCPTransportThread(threading.Thread):
    def __init__(self, splitter):
        self.sock = None
        self.event = None
        self.onFrameCallback = None

        self.splitter = splitter
        self.isReceiving = False
        self.totalBytesRead = 0
        self.totalBytesToRead = 0

        super().__init__()

    def setup(self, sock, event):
        self.sock = sock
        self.event = event

    def set_frame_callback(self, cb):
        self.onFrameCallback = cb

    def run(self):
        while not self.event.is_set():
            try:
                # SELECT with a 500 ms timeout
                readable, writable, exceptional = select.select([self.sock], [], [], 0.5)

                # If the socket is not ready to be read, we continue to the next loop cycle.
                if (readable or writable or exceptional):
                    if self.onFrameCallback:
                        try:
                            actual_payload_data = bytes()
                            if not self.isReceiving:
                                # 65535 - 20 (ip header) - 20 (tcp header) = 65495 bytes
                                data = self.sock.recv(self.splitter.KINOVA_HEADER_SIZE)
                                self.totalBytesRead += len(data)
                                if len(data) == self.splitter.KINOVA_HEADER_SIZE:
                                    self.totalBytesToRead = self.splitter.parse_header(data)
                                    if self.totalBytesToRead <= 0:
                                        print("Incorrect token, flushing buffer")
                                        self.isReceiving = False
                                        self.totalBytesToRead = 0
                                    else:
                                        self.isReceiving = True
                                    self.totalBytesRead = 0

                            if self.isReceiving and self.totalBytesToRead > 0:
                                while self.totalBytesToRead > 0:
                                    socket_bytes = self.sock.recv(self.totalBytesToRead)
                                    actual_payload_data += socket_bytes
                                    self.totalBytesRead += len(socket_bytes)
                                    self.totalBytesToRead -= len(socket_bytes)

                            if len(actual_payload_data) == self.totalBytesRead and self.isReceiving:
                                self.onFrameCallback(actual_payload_data)
                                self.isReceiving = False
                                self.totalBytesRead = 0
                                self.totalBytesToRead = 0

                        except Exception as exception:
                            print(exception)

                            continue  # abort current receive loop iteration




            except IOError as e:
                # Error occurred in SELECT
                print("select()  failed with {}".format(e))