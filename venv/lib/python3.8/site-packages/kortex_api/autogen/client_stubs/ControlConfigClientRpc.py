
from concurrent.futures import Future, TimeoutError
from functools import partial
from deprecated import deprecated
from enum import Enum
from ... import  RouterClient
from ...NotificationHandler import NotificationHandler
from ... import BitMaskTools
from ..messages import ControlConfig_pb2 as ControlConfigPb  # NOQA


class ControlConfigFunctionUid(Enum):
    uidSetGravityVector = 0x100001
    uidGetGravityVector = 0x100002
    uidSetPayloadInformation = 0x100003
    uidGetPayloadInformation = 0x100004
    uidSetToolConfiguration = 0x100005
    uidGetToolConfiguration = 0x100006
    uidOnNotificationControlConfigurationTopic = 0x100007
    uidUnsubscribe = 0x100008
    uidSetCartesianReferenceFrame = 0x100009
    uidGetCartesianReferenceFrame = 0x10000a
    uidGetControlMode = 0x10000d
    uidSetJointSpeedSoftLimits = 0x10000e
    uidSetTwistLinearSoftLimit = 0x10000f
    uidSetTwistAngularSoftLimit = 0x100010
    uidSetJointAccelerationSoftLimits = 0x100011
    uidGetKinematicHardLimits = 0x100012
    uidGetKinematicSoftLimits = 0x100013
    uidGetAllKinematicSoftLimits = 0x100014
    uidSetDesiredLinearTwist = 0x100015
    uidSetDesiredAngularTwist = 0x100016
    uidSetDesiredJointSpeeds = 0x100017
    uidGetDesiredSpeeds = 0x100018
    uidResetGravityVector = 0x100019
    uidResetPayloadInformation = 0x10001a
    uidResetToolConfiguration = 0x10001b
    uidResetJointSpeedSoftLimits = 0x10001c
    uidResetTwistLinearSoftLimit = 0x10001d
    uidResetTwistAngularSoftLimit = 0x10001e
    uidResetJointAccelerationSoftLimits = 0x10001f
    uidOnNotificationControlModeTopic = 0x100020



class ControlConfigClient():
    
    serviceVersion = 1
    serviceId = 16
    router = RouterClient.RouterClient

    def __init__(self, router: RouterClient.RouterClient):
        self.router = router
        self.notificationHandler = NotificationHandler()
        callback = partial(self.ExecuteRouterNotification)
        self.router.registerNotificationCallback(self.serviceId, callback)

    def ExecuteRouterNotification(self, message):
        self.notificationHandler.call(BitMaskTools.extractFrameId(message.header.message_info), message.payload)


    def SetGravityVector(self, gravityvector, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = gravityvector.SerializeToString()

        future = self.router.send(reqPayload, 1, ControlConfigFunctionUid.uidSetGravityVector, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





    def GetGravityVector(self, deviceId = 0, options = RouterClient.RouterClientSendOptions()):

        future = self.router.send(None, 1, ControlConfigFunctionUid.uidGetGravityVector, deviceId, options)

        ansPayload = ControlConfigPb.GravityVector()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

    def SetPayloadInformation(self, payloadinformation, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = payloadinformation.SerializeToString()

        future = self.router.send(reqPayload, 1, ControlConfigFunctionUid.uidSetPayloadInformation, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





    def GetPayloadInformation(self, deviceId = 0, options = RouterClient.RouterClientSendOptions()):

        future = self.router.send(None, 1, ControlConfigFunctionUid.uidGetPayloadInformation, deviceId, options)

        ansPayload = ControlConfigPb.PayloadInformation()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

    def SetToolConfiguration(self, toolconfiguration, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = toolconfiguration.SerializeToString()

        future = self.router.send(reqPayload, 1, ControlConfigFunctionUid.uidSetToolConfiguration, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





    def GetToolConfiguration(self, deviceId = 0, options = RouterClient.RouterClientSendOptions()):

        future = self.router.send(None, 1, ControlConfigFunctionUid.uidGetToolConfiguration, deviceId, options)

        ansPayload = ControlConfigPb.ToolConfiguration()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

    def OnNotificationControlConfigurationTopic(self, callback, notificationoptions, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = notificationoptions.SerializeToString()

        future = self.router.send(reqPayload, 1, ControlConfigFunctionUid.uidOnNotificationControlConfigurationTopic, deviceId, options)

        ansPayload = ControlConfigPb.NotificationHandle()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)

        def parseNotifDataFromString(payload):
            obj = ControlConfigPb.ControlConfigurationNotification()
            obj.ParseFromString(payload)
            return obj

        self.notificationHandler.addCallback(ansPayload.identifier, parseNotifDataFromString, callback)
        return ansPayload

    def Unsubscribe(self, notificationhandle, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = notificationhandle.SerializeToString()

        future = self.router.send(reqPayload, 1, ControlConfigFunctionUid.uidUnsubscribe, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





    def SetCartesianReferenceFrame(self, cartesianreferenceframeinfo, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = cartesianreferenceframeinfo.SerializeToString()

        future = self.router.send(reqPayload, 1, ControlConfigFunctionUid.uidSetCartesianReferenceFrame, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





    def GetCartesianReferenceFrame(self, deviceId = 0, options = RouterClient.RouterClientSendOptions()):

        future = self.router.send(None, 1, ControlConfigFunctionUid.uidGetCartesianReferenceFrame, deviceId, options)

        ansPayload = ControlConfigPb.CartesianReferenceFrameInfo()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

    def GetControlMode(self, deviceId = 0, options = RouterClient.RouterClientSendOptions()):

        future = self.router.send(None, 1, ControlConfigFunctionUid.uidGetControlMode, deviceId, options)

        ansPayload = ControlConfigPb.ControlModeInformation()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

    def SetJointSpeedSoftLimits(self, jointspeedsoftlimits, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = jointspeedsoftlimits.SerializeToString()

        future = self.router.send(reqPayload, 1, ControlConfigFunctionUid.uidSetJointSpeedSoftLimits, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





    def SetTwistLinearSoftLimit(self, twistlinearsoftlimit, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = twistlinearsoftlimit.SerializeToString()

        future = self.router.send(reqPayload, 1, ControlConfigFunctionUid.uidSetTwistLinearSoftLimit, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





    def SetTwistAngularSoftLimit(self, twistangularsoftlimit, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = twistangularsoftlimit.SerializeToString()

        future = self.router.send(reqPayload, 1, ControlConfigFunctionUid.uidSetTwistAngularSoftLimit, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





    def SetJointAccelerationSoftLimits(self, jointaccelerationsoftlimits, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = jointaccelerationsoftlimits.SerializeToString()

        future = self.router.send(reqPayload, 1, ControlConfigFunctionUid.uidSetJointAccelerationSoftLimits, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





    def GetKinematicHardLimits(self, deviceId = 0, options = RouterClient.RouterClientSendOptions()):

        future = self.router.send(None, 1, ControlConfigFunctionUid.uidGetKinematicHardLimits, deviceId, options)

        ansPayload = ControlConfigPb.KinematicLimits()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

    def GetKinematicSoftLimits(self, controlmodeinformation, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = controlmodeinformation.SerializeToString()

        future = self.router.send(reqPayload, 1, ControlConfigFunctionUid.uidGetKinematicSoftLimits, deviceId, options)

        ansPayload = ControlConfigPb.KinematicLimits()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

    def GetAllKinematicSoftLimits(self, deviceId = 0, options = RouterClient.RouterClientSendOptions()):

        future = self.router.send(None, 1, ControlConfigFunctionUid.uidGetAllKinematicSoftLimits, deviceId, options)

        ansPayload = ControlConfigPb.KinematicLimitsList()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

    def SetDesiredLinearTwist(self, lineartwist, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = lineartwist.SerializeToString()

        future = self.router.send(reqPayload, 1, ControlConfigFunctionUid.uidSetDesiredLinearTwist, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





    def SetDesiredAngularTwist(self, angulartwist, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = angulartwist.SerializeToString()

        future = self.router.send(reqPayload, 1, ControlConfigFunctionUid.uidSetDesiredAngularTwist, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





    def SetDesiredJointSpeeds(self, jointspeeds, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = jointspeeds.SerializeToString()

        future = self.router.send(reqPayload, 1, ControlConfigFunctionUid.uidSetDesiredJointSpeeds, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





    def GetDesiredSpeeds(self, deviceId = 0, options = RouterClient.RouterClientSendOptions()):

        future = self.router.send(None, 1, ControlConfigFunctionUid.uidGetDesiredSpeeds, deviceId, options)

        ansPayload = ControlConfigPb.DesiredSpeeds()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

    def ResetGravityVector(self, deviceId = 0, options = RouterClient.RouterClientSendOptions()):

        future = self.router.send(None, 1, ControlConfigFunctionUid.uidResetGravityVector, deviceId, options)

        ansPayload = ControlConfigPb.GravityVector()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

    def ResetPayloadInformation(self, deviceId = 0, options = RouterClient.RouterClientSendOptions()):

        future = self.router.send(None, 1, ControlConfigFunctionUid.uidResetPayloadInformation, deviceId, options)

        ansPayload = ControlConfigPb.PayloadInformation()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

    def ResetToolConfiguration(self, deviceId = 0, options = RouterClient.RouterClientSendOptions()):

        future = self.router.send(None, 1, ControlConfigFunctionUid.uidResetToolConfiguration, deviceId, options)

        ansPayload = ControlConfigPb.ToolConfiguration()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

    def ResetJointSpeedSoftLimits(self, controlmodeinformation, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = controlmodeinformation.SerializeToString()

        future = self.router.send(reqPayload, 1, ControlConfigFunctionUid.uidResetJointSpeedSoftLimits, deviceId, options)

        ansPayload = ControlConfigPb.JointSpeedSoftLimits()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

    def ResetTwistLinearSoftLimit(self, controlmodeinformation, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = controlmodeinformation.SerializeToString()

        future = self.router.send(reqPayload, 1, ControlConfigFunctionUid.uidResetTwistLinearSoftLimit, deviceId, options)

        ansPayload = ControlConfigPb.TwistLinearSoftLimit()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

    def ResetTwistAngularSoftLimit(self, controlmodeinformation, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = controlmodeinformation.SerializeToString()

        future = self.router.send(reqPayload, 1, ControlConfigFunctionUid.uidResetTwistAngularSoftLimit, deviceId, options)

        ansPayload = ControlConfigPb.TwistAngularSoftLimit()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

    def ResetJointAccelerationSoftLimits(self, controlmodeinformation, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = controlmodeinformation.SerializeToString()

        future = self.router.send(reqPayload, 1, ControlConfigFunctionUid.uidResetJointAccelerationSoftLimits, deviceId, options)

        ansPayload = ControlConfigPb.JointAccelerationSoftLimits()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

    def OnNotificationControlModeTopic(self, callback, notificationoptions, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = notificationoptions.SerializeToString()

        future = self.router.send(reqPayload, 1, ControlConfigFunctionUid.uidOnNotificationControlModeTopic, deviceId, options)

        ansPayload = ControlConfigPb.NotificationHandle()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)

        def parseNotifDataFromString(payload):
            obj = ControlConfigPb.ControlModeNotification()
            obj.ParseFromString(payload)
            return obj

        self.notificationHandler.addCallback(ansPayload.identifier, parseNotifDataFromString, callback)
        return ansPayload

