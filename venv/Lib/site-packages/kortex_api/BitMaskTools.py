
# Header frame_info mask
frame_infoMask_headerVersion        = 0xF0000000
frame_infoMask_frameType            = 0x0F000000
frame_infoMask_deviceId             = 0x00FF0000
frame_infoMask_errorCode            = 0x0000F000
frame_infoMask_subErrorCode         = 0x00000FFF

# Header service_info mask
service_infoMask_serviceVersion     = 0xF0000000
service_infoMask_serviceId          = 0x0FFF0000
service_infoMask_functionId         = 0x0000FFFF

# Header message_info mask
message_infoMask_sessionId          = 0xFFFF0000
message_infoMask_messageId          = 0x0000FFFF

# Header payload_info mask
payload_infoMask_reserved           = 0xFF000000
payload_infoMask_payloadLength      = 0x00FFFFFF

def printInBit(data):
    print ('{0:032b}'.format(data))


def extractHeaderVersion(frame_info):
    return int((('{0:032b}'.format(frame_info))[0:4]),2)

def extractFrameType(frame_info):
    return int((('{0:032b}'.format(frame_info))[4:8]),2)

def extractDeviceId(frame_info):
    return int((('{0:032b}'.format(frame_info))[8:16]),2)

def extractErrorCode(frame_info):
    return int((('{0:032b}'.format(frame_info))[16:20]),2)

def extractErrorSubCode(frame_info):
    return int((('{0:032b}'.format(frame_info))[20:32]),2)



def extractServiceVersion(service_info):
    return int((('{0:032b}'.format(service_info))[0:4]),2)

def extractServiceId(service_info):
    return int((('{0:032b}'.format(service_info))[4:16]),2)

def extractFunctionId(service_info):
    return int((('{0:032b}'.format(service_info))[16:32]),2)

def extractFunctionUniqueId(service_info):
    return int((('{0:032b}'.format(service_info))[4:32]),2)



def extractSessionId(message_info):
    return int((('{0:032b}'.format(message_info))[0:16]),2)

def extractFrameId(message_info):
    return int((('{0:032b}'.format(message_info))[16:32]),2)



def extractPayloadReserved(payload_info):
    return int((('{0:032b}'.format(payload_info))[0:8]),2)

def extractPayloadLength(payload_info):
    return int((('{0:032b}'.format(payload_info))[8:32]),2)



def changeHeaderVersion(headerVersion:int, frame_info=None):
    if (frame_info == None):
        frame_info = 0

    sframe_info = str('{0:032b}'.format(frame_info))
    newData = str('{0:04b}'.format(headerVersion))
    newframe_info = sframe_info[:0] + newData +sframe_info[4:]
    return int(newframe_info,2)

def changeFrameType(frameType:int, frame_info=None):
    if (frame_info == None):
        frame_info = 0

    sframe_info = str('{0:032b}'.format(frame_info))
    newData = str('{0:04b}'.format(frameType))
    newframe_info = sframe_info[:4] + newData + sframe_info[8:]
    return int(newframe_info,2)

def changeDeviceId(deviceId:int, frame_info=None):
    if (frame_info == None):
        frame_info = 0

    sframe_info = str('{0:032b}'.format(frame_info))
    newData = str('{0:08b}'.format(deviceId))
    newframe_info = sframe_info[:8] + newData +sframe_info[16:]
    return int(newframe_info,2)

def changeErrorCode(errorCode:int, frame_info=None):
    if (frame_info == None):
        frame_info = 0

    sframe_info = str('{0:032b}'.format(frame_info))
    newData = str('{0:04b}'.format(errorCode))
    newframe_info = sframe_info[:16] + newData +sframe_info[20:]
    return int(newframe_info,2)

def changeErrorSubCode(subCode:int, frame_info=None):
    if (frame_info == None):
        frame_info = 0

    sframe_info = str('{0:032b}'.format(frame_info))
    newData = str('{0:012b}'.format(subCode))
    newframe_info = sframe_info[:20] + newData +sframe_info[32:]
    return int(newframe_info,2)



def changeServiceVersion(serviceVersion:int, service_info=None):
    if (service_info == None):
        service_info = 0

    sservice_info = str('{0:032b}'.format(service_info))
    newData = str('{0:04b}'.format(serviceVersion))
    newframe_info = sservice_info[:0] + newData +sservice_info[4:]
    return int(newframe_info,2)

def changeServiceId(serviceId:int, service_info=None):
    if (service_info == None):
        service_info = 0

    sservice_info = str('{0:032b}'.format(service_info))
    newData = str('{0:012b}'.format(serviceId))
    newframe_info = sservice_info[:4] + newData +sservice_info[16:]
    return int(newframe_info,2)

def changeFunctionId(functionId:int, service_info=None):
    if (service_info == None):
        service_info = 0

    sservice_info = str('{0:032b}'.format(service_info))
    newData = str('{0:016b}'.format(functionId))
    newframe_info = sservice_info[:16] + newData +sservice_info[32:]
    return int(newframe_info,2)



def changeSessionId(sessionId:int, message_info=None):
    if (message_info == None):
        message_info = 0

    smessage_info = str('{0:032b}'.format(message_info))
    newData = str('{0:016b}'.format(sessionId))
    newframe_info = smessage_info[:0] + newData +smessage_info[16:]
    return int(newframe_info,2)

def changeFrameId(messageId:int, message_info=None):
    if (message_info == None):
        message_info = 0

    smessage_info = str('{0:032b}'.format(message_info))
    newData = str('{0:016b}'.format(messageId))
    newframe_info = smessage_info[:16] + newData +smessage_info[32:]
    return int(newframe_info,2)



def changePayloadReserved(value:int, payload_info=None):
    if (payload_info == None):
        payload_info = 0

    str_payload_info = str('{0:032b}'.format(payload_info))
    newData = str('{0:08b}'.format(value))
    newframe_info = str_payload_info[:0] + newData + str_payload_info[8:]
    return int(newframe_info,2)

def changePayloadLength(payloadLength:int, payload_info=None):
    if (payload_info == None):
        payload_info = 0

    str_payload_info = str('{0:032b}'.format(payload_info))
    newData = str('{0:024b}'.format(payloadLength))
    newframe_info = str_payload_info[:8] + newData + str_payload_info[32:]
    return int(newframe_info,2)



def createFrameInfo(headerVersion:int, frameType:int, errorCode:int, errorSubCode:int, deviceId:int=0):
    if(headerVersion.bit_length() > 4) or (frameType.bit_length() > 4) or (deviceId.bit_length() > 8) \
            or (errorCode.bit_length() > 4) or (errorSubCode.bit_length() > 12):
        raise OverflowError
    else:
        frame_info = (headerVersion << 28) + (frameType << 24) + (deviceId << 16) + (errorCode << 12) + errorSubCode

    return frame_info

def createServiceInfo(serviceVersion:int, functionUid:int):
    if(serviceVersion.bit_length() > 4) or (functionUid.bit_length() > 28):
        raise OverflowError
    else:
        service_info = (serviceVersion << 28) + functionUid

    return service_info

def createMessageInfo(sessionId:int, messageId:int):
    if(sessionId.bit_length() > 16) or (messageId.bit_length() > 16):
        raise OverflowError
    else:
        message_info = (sessionId << 16) + messageId

    return message_info

def createPayloadInfo(reservedValue:int, payloadLength:int):
    if(reservedValue.bit_length() > 8) or (payloadLength.bit_length() > 24):
        raise OverflowError
    else:
        payload_info = (reservedValue << 24) + payloadLength

    return payload_info
