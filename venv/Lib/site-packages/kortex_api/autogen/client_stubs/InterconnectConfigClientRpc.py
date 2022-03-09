
from concurrent.futures import Future, TimeoutError
from functools import partial
from deprecated import deprecated
from enum import Enum
from ... import  RouterClient
from ...NotificationHandler import NotificationHandler
from ... import BitMaskTools
from ..messages import InterconnectConfig_pb2 as InterconnectConfigPb  # NOQA


class InterconnectConfigFunctionUid(Enum):
    uidGetUARTConfiguration = 0xe0001
    uidSetUARTConfiguration = 0xe0002
    uidGetEthernetConfiguration = 0xe0003
    uidSetEthernetConfiguration = 0xe0004
    uidGetGPIOConfiguration = 0xe0005
    uidSetGPIOConfiguration = 0xe0006
    uidGetGPIOState = 0xe0007
    uidSetGPIOState = 0xe0008
    uidGetI2CConfiguration = 0xe0009
    uidSetI2CConfiguration = 0xe000a
    uidI2CRead = 0xe000b
    uidI2CReadRegister = 0xe000c
    uidI2CWrite = 0xe000d
    uidI2CWriteRegister = 0xe000e



class InterconnectConfigClient():
    
    serviceVersion = 1
    serviceId = 14
    router = RouterClient.RouterClient

    def __init__(self, router: RouterClient.RouterClient):
        self.router = router
        self.notificationHandler = NotificationHandler()
        callback = partial(self.ExecuteRouterNotification)
        self.router.registerNotificationCallback(self.serviceId, callback)

    def ExecuteRouterNotification(self, message):
        self.notificationHandler.call(BitMaskTools.extractFrameId(message.header.message_info), message.payload)


    def GetUARTConfiguration(self, uartdeviceidentification, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = uartdeviceidentification.SerializeToString()

        future = self.router.send(reqPayload, 1, InterconnectConfigFunctionUid.uidGetUARTConfiguration, deviceId, options)

        ansPayload = InterconnectConfigPb.UARTConfiguration()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

    def SetUARTConfiguration(self, uartconfiguration, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = uartconfiguration.SerializeToString()

        future = self.router.send(reqPayload, 1, InterconnectConfigFunctionUid.uidSetUARTConfiguration, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





    def GetEthernetConfiguration(self, ethernetdeviceidentification, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = ethernetdeviceidentification.SerializeToString()

        future = self.router.send(reqPayload, 1, InterconnectConfigFunctionUid.uidGetEthernetConfiguration, deviceId, options)

        ansPayload = InterconnectConfigPb.EthernetConfiguration()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

    def SetEthernetConfiguration(self, ethernetconfiguration, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = ethernetconfiguration.SerializeToString()

        future = self.router.send(reqPayload, 1, InterconnectConfigFunctionUid.uidSetEthernetConfiguration, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





    def GetGPIOConfiguration(self, gpioidentification, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = gpioidentification.SerializeToString()

        future = self.router.send(reqPayload, 1, InterconnectConfigFunctionUid.uidGetGPIOConfiguration, deviceId, options)

        ansPayload = InterconnectConfigPb.GPIOConfiguration()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

    def SetGPIOConfiguration(self, gpioconfiguration, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = gpioconfiguration.SerializeToString()

        future = self.router.send(reqPayload, 1, InterconnectConfigFunctionUid.uidSetGPIOConfiguration, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





    def GetGPIOState(self, gpioidentification, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = gpioidentification.SerializeToString()

        future = self.router.send(reqPayload, 1, InterconnectConfigFunctionUid.uidGetGPIOState, deviceId, options)

        ansPayload = InterconnectConfigPb.GPIOState()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

    def SetGPIOState(self, gpiostate, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = gpiostate.SerializeToString()

        future = self.router.send(reqPayload, 1, InterconnectConfigFunctionUid.uidSetGPIOState, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





    def GetI2CConfiguration(self, i2cdeviceidentification, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = i2cdeviceidentification.SerializeToString()

        future = self.router.send(reqPayload, 1, InterconnectConfigFunctionUid.uidGetI2CConfiguration, deviceId, options)

        ansPayload = InterconnectConfigPb.I2CConfiguration()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

    def SetI2CConfiguration(self, i2cconfiguration, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = i2cconfiguration.SerializeToString()

        future = self.router.send(reqPayload, 1, InterconnectConfigFunctionUid.uidSetI2CConfiguration, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





    def I2CRead(self, i2creadparameter, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = i2creadparameter.SerializeToString()

        future = self.router.send(reqPayload, 1, InterconnectConfigFunctionUid.uidI2CRead, deviceId, options)

        ansPayload = InterconnectConfigPb.I2CData()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

    def I2CReadRegister(self, i2creadregisterparameter, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = i2creadregisterparameter.SerializeToString()

        future = self.router.send(reqPayload, 1, InterconnectConfigFunctionUid.uidI2CReadRegister, deviceId, options)

        ansPayload = InterconnectConfigPb.I2CData()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

    def I2CWrite(self, i2cwriteparameter, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = i2cwriteparameter.SerializeToString()

        future = self.router.send(reqPayload, 1, InterconnectConfigFunctionUid.uidI2CWrite, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





    def I2CWriteRegister(self, i2cwriteregisterparameter, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = i2cwriteregisterparameter.SerializeToString()

        future = self.router.send(reqPayload, 1, InterconnectConfigFunctionUid.uidI2CWriteRegister, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





