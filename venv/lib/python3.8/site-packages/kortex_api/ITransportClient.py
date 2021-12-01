from enum import Enum
from abc import ABC, abstractmethod


class TransportReadStateEnum(Enum):
    CONNECTING = 0
    OPEN = 1
    CLOSING = 2
    CLOSED = 3
    UNINITIALIZED = 4
    RECONNECTIONG = 5


class ITransportClient(ABC):
    def __init__(self):
        pass

    @abstractmethod
    def connect(self, host, port):
        pass

    @abstractmethod
    def disconnect(self):
        pass

    @abstractmethod
    def send(self, payload):
        pass

    @abstractmethod
    def getMaxTxBufferSize(self):
        pass

    @abstractmethod
    def registerOnFrameCallback(self, callback):
        pass

    # @abstractmethod
    # def getTxBuffer(self):
    #     pass

    # @abstractmethod
    # def getMaxTxBufferSize(self):
    #     pass