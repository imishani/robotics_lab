
from concurrent.futures import Future, TimeoutError
from functools import partial
from deprecated import deprecated
from enum import Enum
from ... import  RouterClient
from ...NotificationHandler import NotificationHandler
from ... import BitMaskTools
from ..messages import BaseCyclic_pb2 as BaseCyclicPb  # NOQA


class BaseCyclicFunctionUid(Enum):
    uidRefresh = 0x30001
    uidRefreshCommand = 0x30002
    uidRefreshFeedback = 0x30003
    uidRefreshCustomData = 0x30004



class BaseCyclicClient():
    
    serviceVersion = 1
    serviceId = 3
    router = RouterClient.RouterClient

    def __init__(self, router: RouterClient.RouterClient):
        self.router = router
        self.notificationHandler = NotificationHandler()
        callback = partial(self.ExecuteRouterNotification)
        self.router.registerNotificationCallback(self.serviceId, callback)

    def ExecuteRouterNotification(self, message):
        self.notificationHandler.call(BitMaskTools.extractFrameId(message.header.message_info), message.payload)


    def Refresh(self, command, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = command.SerializeToString()

        future = self.router.send(reqPayload, 1, BaseCyclicFunctionUid.uidRefresh, deviceId, options)

        ansPayload = BaseCyclicPb.Feedback()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

    def RefreshCommand(self, command, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = command.SerializeToString()

        future = self.router.send(reqPayload, 1, BaseCyclicFunctionUid.uidRefreshCommand, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





    def RefreshFeedback(self, deviceId = 0, options = RouterClient.RouterClientSendOptions()):

        future = self.router.send(None, 1, BaseCyclicFunctionUid.uidRefreshFeedback, deviceId, options)

        ansPayload = BaseCyclicPb.Feedback()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

    def RefreshCustomData(self, customdata, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = customdata.SerializeToString()

        future = self.router.send(reqPayload, 1, BaseCyclicFunctionUid.uidRefreshCustomData, deviceId, options)

        ansPayload = BaseCyclicPb.CustomData()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

