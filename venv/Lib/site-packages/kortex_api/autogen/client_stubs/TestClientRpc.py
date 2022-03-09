
from concurrent.futures import Future, TimeoutError
from functools import partial
from deprecated import deprecated
from enum import Enum
from ... import  RouterClient
from ...NotificationHandler import NotificationHandler
from ... import BitMaskTools
from ..messages import Test_pb2 as TestPb  # NOQA


class TestFunctionUid(Enum):
    uidSetMockValidationStruct = 0xff00001
    uidTestParamAndReturn = 0xff00002
    uidTestParamOnly = 0xff00003
    uidTestReturnOnly = 0xff00004
    uidTestTimeout = 0xff00005
    uidOnNotificationTestNotif = 0xff00006
    uidTestNotifUnsubscribe = 0xff00007
    uidTestAsync = 0xff00008
    uidTestConcurrence = 0xff00009
    uidTestTriggerNotif = 0xff0000a
    uidTestNotImplemented = 0xff0000b
    uidServerError = 0xff0000c
    uidUnsubscribe = 0xff0001e
    uidOnNotificationSomethingChangeTopic = 0xff0001f
    uidTriggerSomethingChangeTopic = 0xff00020
    uidWait = 0xff00021
    uidThrow = 0xff00022
    uidDisconnect = 0xff00023
    uidForget = 0xff00024
    uidNotImplemented = 0xff00025
    uidDeprecated = 0xff00027
    uidDeprecatedWithMessage = 0xff00028



class TestClient():
    
    serviceVersion = 1
    serviceId = 4080
    router = RouterClient.RouterClient

    def __init__(self, router: RouterClient.RouterClient):
        self.router = router
        self.notificationHandler = NotificationHandler()
        callback = partial(self.ExecuteRouterNotification)
        self.router.registerNotificationCallback(self.serviceId, callback)

    def ExecuteRouterNotification(self, message):
        self.notificationHandler.call(BitMaskTools.extractFrameId(message.header.message_info), message.payload)


    def SetMockValidationStruct(self, validatestruct, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = validatestruct.SerializeToString()

        future = self.router.send(reqPayload, 1, TestFunctionUid.uidSetMockValidationStruct, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





    def TestParamAndReturn(self, sendstruct, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = sendstruct.SerializeToString()

        future = self.router.send(reqPayload, 1, TestFunctionUid.uidTestParamAndReturn, deviceId, options)

        ansPayload = TestPb.RcvStruct()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

    def TestParamOnly(self, sendstruct, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = sendstruct.SerializeToString()

        future = self.router.send(reqPayload, 1, TestFunctionUid.uidTestParamOnly, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





    def TestReturnOnly(self, deviceId = 0, options = RouterClient.RouterClientSendOptions()):

        future = self.router.send(None, 1, TestFunctionUid.uidTestReturnOnly, deviceId, options)

        ansPayload = TestPb.RcvStruct()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

    def TestTimeout(self, deviceId = 0, options = RouterClient.RouterClientSendOptions()):

        future = self.router.send(None, 1, TestFunctionUid.uidTestTimeout, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





    def OnNotificationTestNotif(self, callback, notifoption, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = notifoption.SerializeToString()

        future = self.router.send(reqPayload, 1, TestFunctionUid.uidOnNotificationTestNotif, deviceId, options)

        ansPayload = TestPb.NotificationHandle()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)

        def parseNotifDataFromString(payload):
            obj = TestPb.TestNotification()
            obj.ParseFromString(payload)
            return obj

        self.notificationHandler.addCallback(ansPayload.identifier, parseNotifDataFromString, callback)
        return ansPayload

    def TestNotifUnsubscribe(self, notificationhandle, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = notificationhandle.SerializeToString()

        future = self.router.send(reqPayload, 1, TestFunctionUid.uidTestNotifUnsubscribe, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





    def TestAsync(self, timetoresponse, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = timetoresponse.SerializeToString()

        future = self.router.send(reqPayload, 1, TestFunctionUid.uidTestAsync, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





    def TestConcurrence(self, deviceId = 0, options = RouterClient.RouterClientSendOptions()):

        future = self.router.send(None, 1, TestFunctionUid.uidTestConcurrence, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





    def TestTriggerNotif(self, deviceId = 0, options = RouterClient.RouterClientSendOptions()):

        future = self.router.send(None, 1, TestFunctionUid.uidTestTriggerNotif, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





    def TestNotImplemented(self, deviceId = 0, options = RouterClient.RouterClientSendOptions()):

        future = self.router.send(None, 1, TestFunctionUid.uidTestNotImplemented, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





    def ServerError(self, testerror, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = testerror.SerializeToString()

        future = self.router.send(reqPayload, 1, TestFunctionUid.uidServerError, deviceId, options)

        ansPayload = TestPb.TestError()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

    def Unsubscribe(self, notificationhandle, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = notificationhandle.SerializeToString()

        future = self.router.send(reqPayload, 1, TestFunctionUid.uidUnsubscribe, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





    def OnNotificationSomethingChangeTopic(self, callback, notificationoptions, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = notificationoptions.SerializeToString()

        future = self.router.send(reqPayload, 1, TestFunctionUid.uidOnNotificationSomethingChangeTopic, deviceId, options)

        ansPayload = TestPb.NotificationHandle()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)

        def parseNotifDataFromString(payload):
            obj = TestPb.SomethingChanged()
            obj.ParseFromString(payload)
            return obj

        self.notificationHandler.addCallback(ansPayload.identifier, parseNotifDataFromString, callback)
        return ansPayload

    def TriggerSomethingChangeTopic(self, deviceId = 0, options = RouterClient.RouterClientSendOptions()):

        future = self.router.send(None, 1, TestFunctionUid.uidTriggerSomethingChangeTopic, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





    def Wait(self, delay, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = delay.SerializeToString()

        future = self.router.send(reqPayload, 1, TestFunctionUid.uidWait, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





    def Throw(self, error, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = error.SerializeToString()

        future = self.router.send(reqPayload, 1, TestFunctionUid.uidThrow, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





    def Disconnect(self, deviceId = 0, options = RouterClient.RouterClientSendOptions()):

        future = self.router.send(None, 1, TestFunctionUid.uidDisconnect, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





    def Forget(self, deviceId = 0, options = RouterClient.RouterClientSendOptions()):

        future = self.router.send(None, 1, TestFunctionUid.uidForget, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





    def NotImplemented(self, deviceId = 0, options = RouterClient.RouterClientSendOptions()):

        future = self.router.send(None, 1, TestFunctionUid.uidNotImplemented, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





    @deprecated
    def Deprecated(self, deviceId = 0, options = RouterClient.RouterClientSendOptions()):

        future = self.router.send(None, 1, TestFunctionUid.uidDeprecated, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





    @deprecated("Purposely deprecated for test")
    def DeprecatedWithMessage(self, deviceId = 0, options = RouterClient.RouterClientSendOptions()):

        future = self.router.send(None, 1, TestFunctionUid.uidDeprecatedWithMessage, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





