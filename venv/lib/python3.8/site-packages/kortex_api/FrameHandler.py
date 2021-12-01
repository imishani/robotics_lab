import threading
from concurrent.futures import Future

from .Exceptions.KException import KException
from .Exceptions.KClientException import KClientException
from .Exceptions.KServerException import KServerException

from .autogen.messages import Frame_pb2 as FramePb
from .autogen.messages import Errors_pb2 as ErrorsPb
from . import BitMaskTools

class FrameHandler:
    def __init__(self):
        self.framePromise = {}
        self.m_mutex = threading.Lock()
        self.currentFrameId = 0
        self.sessionId = 0

    def registerFrame(self, payload, serviceVersion, functionUid, deviceId):
        with self.m_mutex:

            # Wrap around currentFrameId to fit bits defined in header
            self.currentFrameId += 1 
            self.currentFrameId %= (BitMaskTools.message_infoMask_messageId + 1)

            frame = FramePb.Frame()

            serviceInfo = (serviceVersion << 28) + (functionUid.value & 0x0fffffff)
            frame.header.service_info = serviceInfo

            frame.header.message_info = BitMaskTools.changeFrameId(self.currentFrameId, frame.header.message_info)
            frame.header.message_info = BitMaskTools.changeSessionId(self.sessionId, frame.header.message_info)

            frame.header.frame_info = BitMaskTools.changeHeaderVersion(1, frame.header.frame_info)
            frame.header.frame_info = BitMaskTools.changeFrameType(FramePb.MSG_FRAME_REQUEST, frame.header.frame_info)
            frame.header.frame_info = BitMaskTools.changeDeviceId(deviceId, frame.header.frame_info)

            if payload is not None:
                frame.header.payload_info = BitMaskTools.changePayloadLength(len(payload), frame.header.payload_info)
                frame.payload = payload
            else:
                frame.header.payload_info = BitMaskTools.changePayloadLength(0, frame.header.payload_info)
                # frame.payload = "" ???

            self.framePromise[self.currentFrameId] = Future()
            return (frame.SerializeToString(), self.currentFrameId, self.framePromise[self.currentFrameId])

    def manageReceivedFrame(self, frame: FramePb):
        with self.m_mutex:
            cmdPromise = Future()
            error = FramePb.Error()

            frameId = BitMaskTools.extractFrameId(frame.header.message_info)
            self.sessionId = BitMaskTools.extractSessionId(frame.header.message_info)

            if list(self.framePromise.keys()).count(frameId) > 0:
                cmdPromise = self.framePromise[frameId]

                if BitMaskTools.extractErrorCode(frame.header.frame_info) != ErrorsPb.ERROR_NONE:
                    server_ex = KServerException(frame)
                    cmdPromise.set_exception(server_ex)
                else:
                    cmdPromise.set_result(frame)

                del self.framePromise[frameId]
            else:
                error.error_code = ErrorsPb.ERROR_PROTOCOL_CLIENT
                error.error_sub_code = ErrorsPb.UNREGISTERED_FRAME_RECEIVED
                error.error_sub_string = "Unexpected frame received from server, frameId={}".format(frameId)

            return error

    def forcePromiseException(self, frameId, errorEx: KClientException):
        with self.m_mutex:
            if list(self.framePromise.keys()).count(frameId) > 0:
                cmdPromise = self.framePromise[frameId]
                cmdPromise.set_exception(errorEx)
                del self.framePromise[frameId]
            else:
                raise KException("Programmer error: forcePromiseException on non-existing frame msg id")
