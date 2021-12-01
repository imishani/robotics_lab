# pylint: disable=E0402
from .KException import KException 
from ..autogen.messages import Frame_pb2 as FramePb
from ..autogen.messages import Errors_pb2 as ErrorsPb
from .. import BitMaskTools

class KServerException(KException):
    def __init__(self, frame):
        self.frame = frame
        self.error = FramePb.Error()
        try:
            self.error.ParseFromString(frame.payload)

            if self.error.error_code == ErrorsPb.ERROR_NONE:
                self.error.error_code = BitMaskTools.extractErrorCode(frame.header.frame_info)
                self.error.error_sub_code = BitMaskTools.extractErrorSubCode(frame.header.frame_info)
                self.error.error_sub_string = "Error details were not received from server."

            self.error_code = self.error.error_code
            self.error_sub_code = self.error.error_sub_code

            super().__init__(self.error.error_sub_string)
        except:
            # This should never happen in production
            self.error_code = ErrorsPb.ErrorCodes.Value("ERROR_INTERNAL")
            self.error_sub_code = ErrorsPb.SubErrorCodes.Value("FRAME_DECODING_ERR")
            super().__init__("Non-parsable payload received from device.")

    def __str__(self):
        code_name = ErrorsPb.ErrorCodes.Name(self.error.error_code)
        sub_code_name = ErrorsPb.SubErrorCodes.Name(self.error.error_sub_code)

        return 'Server error name={0}, sub name={1} => {2} \n'.format(code_name, sub_code_name, self.description)

    def get_error_code(self):
        return self.error_code

    def get_error_sub_code(self):
        return self.error_sub_code
