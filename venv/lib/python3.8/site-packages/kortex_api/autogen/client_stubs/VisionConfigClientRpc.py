
from concurrent.futures import Future, TimeoutError
from functools import partial
from deprecated import deprecated
from enum import Enum
from ... import  RouterClient
from ...NotificationHandler import NotificationHandler
from ... import BitMaskTools
from ..messages import VisionConfig_pb2 as VisionConfigPb  # NOQA


class VisionConfigFunctionUid(Enum):
    uidSetSensorSettings = 0x50001
    uidGetSensorSettings = 0x50002
    uidGetOptionValue = 0x50003
    uidSetOptionValue = 0x50004
    uidGetOptionInformation = 0x50005
    uidOnNotificationVisionTopic = 0x50006
    uidDoSensorFocusAction = 0x50007
    uidGetIntrinsicParameters = 0x50008
    uidGetIntrinsicParametersProfile = 0x50009
    uidSetIntrinsicParameters = 0x5000a
    uidGetExtrinsicParameters = 0x5000b
    uidSetExtrinsicParameters = 0x5000c



class VisionConfigClient():
    
    serviceVersion = 1
    serviceId = 5
    router = RouterClient.RouterClient

    def __init__(self, router: RouterClient.RouterClient):
        self.router = router
        self.notificationHandler = NotificationHandler()
        callback = partial(self.ExecuteRouterNotification)
        self.router.registerNotificationCallback(self.serviceId, callback)

    def ExecuteRouterNotification(self, message):
        self.notificationHandler.call(BitMaskTools.extractFrameId(message.header.message_info), message.payload)


    def SetSensorSettings(self, sensorsettings, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = sensorsettings.SerializeToString()

        future = self.router.send(reqPayload, 1, VisionConfigFunctionUid.uidSetSensorSettings, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





    def GetSensorSettings(self, sensoridentifier, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = sensoridentifier.SerializeToString()

        future = self.router.send(reqPayload, 1, VisionConfigFunctionUid.uidGetSensorSettings, deviceId, options)

        ansPayload = VisionConfigPb.SensorSettings()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

    def GetOptionValue(self, optionidentifier, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = optionidentifier.SerializeToString()

        future = self.router.send(reqPayload, 1, VisionConfigFunctionUid.uidGetOptionValue, deviceId, options)

        ansPayload = VisionConfigPb.OptionValue()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

    def SetOptionValue(self, optionvalue, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = optionvalue.SerializeToString()

        future = self.router.send(reqPayload, 1, VisionConfigFunctionUid.uidSetOptionValue, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





    def GetOptionInformation(self, optionidentifier, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = optionidentifier.SerializeToString()

        future = self.router.send(reqPayload, 1, VisionConfigFunctionUid.uidGetOptionInformation, deviceId, options)

        ansPayload = VisionConfigPb.OptionInformation()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

    def OnNotificationVisionTopic(self, callback, notificationoptions, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = notificationoptions.SerializeToString()

        future = self.router.send(reqPayload, 1, VisionConfigFunctionUid.uidOnNotificationVisionTopic, deviceId, options)

        ansPayload = VisionConfigPb.NotificationHandle()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)

        def parseNotifDataFromString(payload):
            obj = VisionConfigPb.VisionNotification()
            obj.ParseFromString(payload)
            return obj

        self.notificationHandler.addCallback(ansPayload.identifier, parseNotifDataFromString, callback)
        return ansPayload

    def DoSensorFocusAction(self, sensorfocusaction, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = sensorfocusaction.SerializeToString()

        future = self.router.send(reqPayload, 1, VisionConfigFunctionUid.uidDoSensorFocusAction, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





    def GetIntrinsicParameters(self, sensoridentifier, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = sensoridentifier.SerializeToString()

        future = self.router.send(reqPayload, 1, VisionConfigFunctionUid.uidGetIntrinsicParameters, deviceId, options)

        ansPayload = VisionConfigPb.IntrinsicParameters()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

    def GetIntrinsicParametersProfile(self, intrinsicprofileidentifier, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = intrinsicprofileidentifier.SerializeToString()

        future = self.router.send(reqPayload, 1, VisionConfigFunctionUid.uidGetIntrinsicParametersProfile, deviceId, options)

        ansPayload = VisionConfigPb.IntrinsicParameters()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

    def SetIntrinsicParameters(self, intrinsicparameters, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = intrinsicparameters.SerializeToString()

        future = self.router.send(reqPayload, 1, VisionConfigFunctionUid.uidSetIntrinsicParameters, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





    def GetExtrinsicParameters(self, deviceId = 0, options = RouterClient.RouterClientSendOptions()):

        future = self.router.send(None, 1, VisionConfigFunctionUid.uidGetExtrinsicParameters, deviceId, options)

        ansPayload = VisionConfigPb.ExtrinsicParameters()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

    def SetExtrinsicParameters(self, extrinsicparameters, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = extrinsicparameters.SerializeToString()

        future = self.router.send(reqPayload, 1, VisionConfigFunctionUid.uidSetExtrinsicParameters, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





