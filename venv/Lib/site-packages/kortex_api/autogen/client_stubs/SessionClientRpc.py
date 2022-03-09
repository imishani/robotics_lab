
from concurrent.futures import Future, TimeoutError
from functools import partial
from deprecated import deprecated
from enum import Enum
from ... import  RouterClient
from ...NotificationHandler import NotificationHandler
from ... import BitMaskTools
from ..messages import Session_pb2 as SessionPb  # NOQA


class SessionFunctionUid(Enum):
    uidCreateSession = 0x10001
    uidCloseSession = 0x10002
    uidKeepAlive = 0x10003
    uidGetConnections = 0x10004



class SessionClient():
    
    serviceVersion = 1
    serviceId = 1
    router = RouterClient.RouterClient

    def __init__(self, router: RouterClient.RouterClient):
        self.router = router
        self.notificationHandler = NotificationHandler()
        callback = partial(self.ExecuteRouterNotification)
        self.router.registerNotificationCallback(self.serviceId, callback)

    def ExecuteRouterNotification(self, message):
        self.notificationHandler.call(BitMaskTools.extractFrameId(message.header.message_info), message.payload)


    def CreateSession(self, createsessioninfo, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = createsessioninfo.SerializeToString()

        future = self.router.send(reqPayload, 1, SessionFunctionUid.uidCreateSession, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





    def CloseSession(self, deviceId = 0, options = RouterClient.RouterClientSendOptions()):

        future = self.router.send(None, 1, SessionFunctionUid.uidCloseSession, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





    def KeepAlive(self, deviceId = 0, options = RouterClient.RouterClientSendOptions()):

        future = self.router.send(None, 1, SessionFunctionUid.uidKeepAlive, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





    def GetConnections(self, deviceId = 0, options = RouterClient.RouterClientSendOptions()):

        future = self.router.send(None, 1, SessionFunctionUid.uidGetConnections, deviceId, options)

        ansPayload = SessionPb.ConnectionList()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

