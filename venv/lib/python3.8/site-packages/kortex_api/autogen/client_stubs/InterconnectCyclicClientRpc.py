
from concurrent.futures import Future, TimeoutError
from functools import partial
from deprecated import deprecated
from enum import Enum
from ... import  RouterClient
from ...NotificationHandler import NotificationHandler
from ... import BitMaskTools
from ..messages import InterconnectCyclic_pb2 as InterconnectCyclicPb  # NOQA


class InterconnectCyclicFunctionUid(Enum):
    uidRefresh = 0xf0001
    uidRefreshCommand = 0xf0002
    uidRefreshFeedback = 0xf0003
    uidRefreshCustomData = 0xf0004



class InterconnectCyclicClient():
    
    serviceVersion = 1
    serviceId = 15
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

        future = self.router.send(reqPayload, 1, InterconnectCyclicFunctionUid.uidRefresh, deviceId, options)

        ansPayload = InterconnectCyclicPb.Feedback()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

    def RefreshCommand(self, command, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = command.SerializeToString()

        future = self.router.send(reqPayload, 1, InterconnectCyclicFunctionUid.uidRefreshCommand, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





    def RefreshFeedback(self, messageid, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = messageid.SerializeToString()

        future = self.router.send(reqPayload, 1, InterconnectCyclicFunctionUid.uidRefreshFeedback, deviceId, options)

        ansPayload = InterconnectCyclicPb.Feedback()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

    def RefreshCustomData(self, messageid, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = messageid.SerializeToString()

        future = self.router.send(reqPayload, 1, InterconnectCyclicFunctionUid.uidRefreshCustomData, deviceId, options)

        ansPayload = InterconnectCyclicPb.CustomData()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

