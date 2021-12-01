from ..autogen.messages import Frame_pb2 as FramePb
from ..autogen.messages import Errors_pb2 as ErrorsPb
from .KException import KException

class KClientException(KException):
    def __init__(self, sub_code, description):
        self.error = FramePb.Error()
        self.error.error_code = ErrorsPb.ERROR_PROTOCOL_CLIENT
        self.error.error_sub_code = sub_code
        self.error.error_sub_string = description
        super().__init__(self.error.error_sub_string)

    def createFromError(errorFrom):
        return KClientException(errorFrom.error_sub_code, errorFrom.error_sub_string)

    def __str__(self):
        return 'Clientside error encountered: Type: {0} -> SUB_NAME: {1} -> {2}'.format(
                            ErrorsPb.ErrorCodes.Name(self.error.error_code),
                            ErrorsPb.SubErrorCodes.Name(self.error.error_sub_code),
                            self.error.error_sub_string)
