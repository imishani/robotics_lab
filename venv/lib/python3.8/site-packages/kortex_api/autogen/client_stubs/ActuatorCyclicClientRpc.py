
from concurrent.futures import Future, TimeoutError
from functools import partial
from deprecated import deprecated
from enum import Enum
from ... import  RouterClient
from ...NotificationHandler import NotificationHandler
from ... import BitMaskTools
from ..messages import ActuatorCyclic_pb2 as ActuatorCyclicPb  # NOQA


class ActuatorCyclicFunctionUid(Enum):
    uidRefresh = 0xb0001
    uidRefreshCommand = 0xb0002
    uidRefreshFeedback = 0xb0003
    uidRefreshCustomData = 0xb0004



class ActuatorCyclicClient():
    
    serviceVersion = 1
    serviceId = 11
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

        future = self.router.send(reqPayload, 1, ActuatorCyclicFunctionUid.uidRefresh, deviceId, options)

        ansPayload = ActuatorCyclicPb.Feedback()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

    def RefreshCommand(self, command, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = command.SerializeToString()

        future = self.router.send(reqPayload, 1, ActuatorCyclicFunctionUid.uidRefreshCommand, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





    def RefreshFeedback(self, messageid, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = messageid.SerializeToString()

        future = self.router.send(reqPayload, 1, ActuatorCyclicFunctionUid.uidRefreshFeedback, deviceId, options)

        ansPayload = ActuatorCyclicPb.Feedback()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

    def RefreshCustomData(self, messageid, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = messageid.SerializeToString()

        future = self.router.send(reqPayload, 1, ActuatorCyclicFunctionUid.uidRefreshCustomData, deviceId, options)

        ansPayload = ActuatorCyclicPb.CustomData()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

