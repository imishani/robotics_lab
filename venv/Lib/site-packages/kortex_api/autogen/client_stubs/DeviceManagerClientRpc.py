
from concurrent.futures import Future, TimeoutError
from functools import partial
from deprecated import deprecated
from enum import Enum
from ... import  RouterClient
from ...NotificationHandler import NotificationHandler
from ... import BitMaskTools
from ..messages import DeviceManager_pb2 as DeviceManagerPb  # NOQA


class DeviceManagerFunctionUid(Enum):
    uidReadAllDevices = 0x170001



class DeviceManagerClient():
    
    serviceVersion = 1
    serviceId = 23
    router = RouterClient.RouterClient

    def __init__(self, router: RouterClient.RouterClient):
        self.router = router
        self.notificationHandler = NotificationHandler()
        callback = partial(self.ExecuteRouterNotification)
        self.router.registerNotificationCallback(self.serviceId, callback)

    def ExecuteRouterNotification(self, message):
        self.notificationHandler.call(BitMaskTools.extractFrameId(message.header.message_info), message.payload)


    def ReadAllDevices(self, deviceId = 0, options = RouterClient.RouterClientSendOptions()):

        future = self.router.send(None, 1, DeviceManagerFunctionUid.uidReadAllDevices, deviceId, options)

        ansPayload = DeviceManagerPb.DeviceHandles()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

