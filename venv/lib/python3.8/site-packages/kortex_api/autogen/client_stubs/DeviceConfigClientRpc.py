
from concurrent.futures import Future, TimeoutError
from functools import partial
from deprecated import deprecated
from enum import Enum
from ... import  RouterClient
from ...NotificationHandler import NotificationHandler
from ... import BitMaskTools
from ..messages import DeviceConfig_pb2 as DeviceConfigPb  # NOQA


class DeviceConfigFunctionUid(Enum):
    uidGetRunMode = 0x90001
    uidSetRunMode = 0x90002
    uidGetDeviceType = 0x90003
    uidGetFirmwareVersion = 0x90004
    uidGetBootloaderVersion = 0x90005
    uidGetModelNumber = 0x90006
    uidGetPartNumber = 0x90007
    uidGetSerialNumber = 0x90008
    uidGetMACAddress = 0x90009
    uidGetIPv4Settings = 0x9000a
    uidSetIPv4Settings = 0x9000b
    uidGetPartNumberRevision = 0x9000c
    uidRebootRequest = 0x9000e
    uidSetSafetyEnable = 0x9000f
    uidSetSafetyErrorThreshold = 0x90010
    uidSetSafetyWarningThreshold = 0x90011
    uidSetSafetyConfiguration = 0x90012
    uidGetSafetyConfiguration = 0x90013
    uidGetSafetyInformation = 0x90014
    uidGetSafetyEnable = 0x90015
    uidGetSafetyStatus = 0x90016
    uidClearAllSafetyStatus = 0x90017
    uidClearSafetyStatus = 0x90018
    uidGetAllSafetyConfiguration = 0x90019
    uidGetAllSafetyInformation = 0x9001a
    uidResetSafetyDefaults = 0x9001b
    uidOnNotificationSafetyTopic = 0x9001c
    uidExecuteCalibration = 0x90022
    uidGetCalibrationResult = 0x90023
    uidStopCalibration = 0x90024
    uidSetCapSenseConfig = 0x90025
    uidGetCapSenseConfig = 0x90026
    uidReadCapSenseRegister = 0x90027
    uidWriteCapSenseRegister = 0x90028



class DeviceConfigClient():
    
    serviceVersion = 1
    serviceId = 9
    router = RouterClient.RouterClient

    def __init__(self, router: RouterClient.RouterClient):
        self.router = router
        self.notificationHandler = NotificationHandler()
        callback = partial(self.ExecuteRouterNotification)
        self.router.registerNotificationCallback(self.serviceId, callback)

    def ExecuteRouterNotification(self, message):
        self.notificationHandler.call(BitMaskTools.extractFrameId(message.header.message_info), message.payload)


    def GetRunMode(self, deviceId = 0, options = RouterClient.RouterClientSendOptions()):

        future = self.router.send(None, 1, DeviceConfigFunctionUid.uidGetRunMode, deviceId, options)

        ansPayload = DeviceConfigPb.RunMode()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

    def SetRunMode(self, runmode, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = runmode.SerializeToString()

        future = self.router.send(reqPayload, 1, DeviceConfigFunctionUid.uidSetRunMode, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





    def GetDeviceType(self, deviceId = 0, options = RouterClient.RouterClientSendOptions()):

        future = self.router.send(None, 1, DeviceConfigFunctionUid.uidGetDeviceType, deviceId, options)

        ansPayload = DeviceConfigPb.DeviceType()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

    def GetFirmwareVersion(self, deviceId = 0, options = RouterClient.RouterClientSendOptions()):

        future = self.router.send(None, 1, DeviceConfigFunctionUid.uidGetFirmwareVersion, deviceId, options)

        ansPayload = DeviceConfigPb.FirmwareVersion()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

    def GetBootloaderVersion(self, deviceId = 0, options = RouterClient.RouterClientSendOptions()):

        future = self.router.send(None, 1, DeviceConfigFunctionUid.uidGetBootloaderVersion, deviceId, options)

        ansPayload = DeviceConfigPb.BootloaderVersion()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

    def GetModelNumber(self, deviceId = 0, options = RouterClient.RouterClientSendOptions()):

        future = self.router.send(None, 1, DeviceConfigFunctionUid.uidGetModelNumber, deviceId, options)

        ansPayload = DeviceConfigPb.ModelNumber()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

    def GetPartNumber(self, deviceId = 0, options = RouterClient.RouterClientSendOptions()):

        future = self.router.send(None, 1, DeviceConfigFunctionUid.uidGetPartNumber, deviceId, options)

        ansPayload = DeviceConfigPb.PartNumber()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

    def GetSerialNumber(self, deviceId = 0, options = RouterClient.RouterClientSendOptions()):

        future = self.router.send(None, 1, DeviceConfigFunctionUid.uidGetSerialNumber, deviceId, options)

        ansPayload = DeviceConfigPb.SerialNumber()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

    def GetMACAddress(self, deviceId = 0, options = RouterClient.RouterClientSendOptions()):

        future = self.router.send(None, 1, DeviceConfigFunctionUid.uidGetMACAddress, deviceId, options)

        ansPayload = DeviceConfigPb.MACAddress()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

    def GetIPv4Settings(self, deviceId = 0, options = RouterClient.RouterClientSendOptions()):

        future = self.router.send(None, 1, DeviceConfigFunctionUid.uidGetIPv4Settings, deviceId, options)

        ansPayload = DeviceConfigPb.IPv4Settings()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

    def SetIPv4Settings(self, ipv4settings, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = ipv4settings.SerializeToString()

        future = self.router.send(reqPayload, 1, DeviceConfigFunctionUid.uidSetIPv4Settings, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





    def GetPartNumberRevision(self, deviceId = 0, options = RouterClient.RouterClientSendOptions()):

        future = self.router.send(None, 1, DeviceConfigFunctionUid.uidGetPartNumberRevision, deviceId, options)

        ansPayload = DeviceConfigPb.PartNumberRevision()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

    def RebootRequest(self, rebootrqst, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = rebootrqst.SerializeToString()

        future = self.router.send(reqPayload, 1, DeviceConfigFunctionUid.uidRebootRequest, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





    def SetSafetyEnable(self, safetyenable, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = safetyenable.SerializeToString()

        future = self.router.send(reqPayload, 1, DeviceConfigFunctionUid.uidSetSafetyEnable, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





    def SetSafetyErrorThreshold(self, safetythreshold, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = safetythreshold.SerializeToString()

        future = self.router.send(reqPayload, 1, DeviceConfigFunctionUid.uidSetSafetyErrorThreshold, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





    def SetSafetyWarningThreshold(self, safetythreshold, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = safetythreshold.SerializeToString()

        future = self.router.send(reqPayload, 1, DeviceConfigFunctionUid.uidSetSafetyWarningThreshold, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





    def SetSafetyConfiguration(self, safetyconfiguration, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = safetyconfiguration.SerializeToString()

        future = self.router.send(reqPayload, 1, DeviceConfigFunctionUid.uidSetSafetyConfiguration, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





    def GetSafetyConfiguration(self, safetyhandle, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = safetyhandle.SerializeToString()

        future = self.router.send(reqPayload, 1, DeviceConfigFunctionUid.uidGetSafetyConfiguration, deviceId, options)

        ansPayload = DeviceConfigPb.SafetyConfiguration()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

    def GetSafetyInformation(self, safetyhandle, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = safetyhandle.SerializeToString()

        future = self.router.send(reqPayload, 1, DeviceConfigFunctionUid.uidGetSafetyInformation, deviceId, options)

        ansPayload = DeviceConfigPb.SafetyInformation()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

    def GetSafetyEnable(self, safetyhandle, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = safetyhandle.SerializeToString()

        future = self.router.send(reqPayload, 1, DeviceConfigFunctionUid.uidGetSafetyEnable, deviceId, options)

        ansPayload = DeviceConfigPb.SafetyEnable()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

    def GetSafetyStatus(self, safetyhandle, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = safetyhandle.SerializeToString()

        future = self.router.send(reqPayload, 1, DeviceConfigFunctionUid.uidGetSafetyStatus, deviceId, options)

        ansPayload = DeviceConfigPb.SafetyStatus()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

    def ClearAllSafetyStatus(self, deviceId = 0, options = RouterClient.RouterClientSendOptions()):

        future = self.router.send(None, 1, DeviceConfigFunctionUid.uidClearAllSafetyStatus, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





    def ClearSafetyStatus(self, safetyhandle, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = safetyhandle.SerializeToString()

        future = self.router.send(reqPayload, 1, DeviceConfigFunctionUid.uidClearSafetyStatus, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





    def GetAllSafetyConfiguration(self, deviceId = 0, options = RouterClient.RouterClientSendOptions()):

        future = self.router.send(None, 1, DeviceConfigFunctionUid.uidGetAllSafetyConfiguration, deviceId, options)

        ansPayload = DeviceConfigPb.SafetyConfigurationList()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

    def GetAllSafetyInformation(self, deviceId = 0, options = RouterClient.RouterClientSendOptions()):

        future = self.router.send(None, 1, DeviceConfigFunctionUid.uidGetAllSafetyInformation, deviceId, options)

        ansPayload = DeviceConfigPb.SafetyInformationList()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

    def ResetSafetyDefaults(self, deviceId = 0, options = RouterClient.RouterClientSendOptions()):

        future = self.router.send(None, 1, DeviceConfigFunctionUid.uidResetSafetyDefaults, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





    def OnNotificationSafetyTopic(self, callback, notificationoptions, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = notificationoptions.SerializeToString()

        future = self.router.send(reqPayload, 1, DeviceConfigFunctionUid.uidOnNotificationSafetyTopic, deviceId, options)

        ansPayload = DeviceConfigPb.NotificationHandle()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)

        def parseNotifDataFromString(payload):
            obj = DeviceConfigPb.SafetyNotification()
            obj.ParseFromString(payload)
            return obj

        self.notificationHandler.addCallback(ansPayload.identifier, parseNotifDataFromString, callback)
        return ansPayload

    def ExecuteCalibration(self, calibration, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = calibration.SerializeToString()

        future = self.router.send(reqPayload, 1, DeviceConfigFunctionUid.uidExecuteCalibration, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





    def GetCalibrationResult(self, calibrationelement, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = calibrationelement.SerializeToString()

        future = self.router.send(reqPayload, 1, DeviceConfigFunctionUid.uidGetCalibrationResult, deviceId, options)

        ansPayload = DeviceConfigPb.CalibrationResult()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

    def StopCalibration(self, calibration, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = calibration.SerializeToString()

        future = self.router.send(reqPayload, 1, DeviceConfigFunctionUid.uidStopCalibration, deviceId, options)

        ansPayload = DeviceConfigPb.CalibrationResult()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

    def SetCapSenseConfig(self, capsenseconfig, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = capsenseconfig.SerializeToString()

        future = self.router.send(reqPayload, 1, DeviceConfigFunctionUid.uidSetCapSenseConfig, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





    def GetCapSenseConfig(self, deviceId = 0, options = RouterClient.RouterClientSendOptions()):

        future = self.router.send(None, 1, DeviceConfigFunctionUid.uidGetCapSenseConfig, deviceId, options)

        ansPayload = DeviceConfigPb.CapSenseConfig()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

    def ReadCapSenseRegister(self, capsenseregister, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = capsenseregister.SerializeToString()

        future = self.router.send(reqPayload, 1, DeviceConfigFunctionUid.uidReadCapSenseRegister, deviceId, options)

        ansPayload = DeviceConfigPb.CapSenseRegister()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

    def WriteCapSenseRegister(self, capsenseregister, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = capsenseregister.SerializeToString()

        future = self.router.send(reqPayload, 1, DeviceConfigFunctionUid.uidWriteCapSenseRegister, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





