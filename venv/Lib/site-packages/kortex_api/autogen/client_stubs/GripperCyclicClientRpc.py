
from concurrent.futures import Future, TimeoutError
from functools import partial
from deprecated import deprecated
from enum import Enum
from ... import  RouterClient
from ...NotificationHandler import NotificationHandler
from ... import BitMaskTools
from ..messages import GripperCyclic_pb2 as GripperCyclicPb  # NOQA


class GripperCyclicFunctionUid(Enum):
    uidRefresh = 0x110001
    uidRefreshCommand = 0x110002
    uidRefreshFeedback = 0x110003
    uidRefreshCustomData = 0x110004



class GripperCyclicClient():
    
    serviceVersion = 1
    serviceId = 17
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

        future = self.router.send(reqPayload, 1, GripperCyclicFunctionUid.uidRefresh, deviceId, options)

        ansPayload = GripperCyclicPb.Feedback()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

    def RefreshCommand(self, command, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = command.SerializeToString()

        future = self.router.send(reqPayload, 1, GripperCyclicFunctionUid.uidRefreshCommand, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





    def RefreshFeedback(self, messageid, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = messageid.SerializeToString()

        future = self.router.send(reqPayload, 1, GripperCyclicFunctionUid.uidRefreshFeedback, deviceId, options)

        ansPayload = GripperCyclicPb.Feedback()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

    def RefreshCustomData(self, messageid, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = messageid.SerializeToString()

        future = self.router.send(reqPayload, 1, GripperCyclicFunctionUid.uidRefreshCustomData, deviceId, options)

        ansPayload = GripperCyclicPb.CustomData()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

