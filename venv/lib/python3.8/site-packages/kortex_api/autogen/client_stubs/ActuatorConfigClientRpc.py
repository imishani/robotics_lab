
from concurrent.futures import Future, TimeoutError
from functools import partial
from deprecated import deprecated
from enum import Enum
from ... import  RouterClient
from ...NotificationHandler import NotificationHandler
from ... import BitMaskTools
from ..messages import ActuatorConfig_pb2 as ActuatorConfigPb  # NOQA


class ActuatorConfigFunctionUid(Enum):
    uidGetAxisOffsets = 0xa0001
    uidSetAxisOffsets = 0xa0002
    uidReadTorqueCalibration = 0xa0003
    uidWriteTorqueCalibration = 0xa0004
    uidSetTorqueOffset = 0xa0005
    uidGetControlMode = 0xa0006
    uidSetControlMode = 0xa0007
    uidGetActivatedControlLoop = 0xa0008
    uidSetActivatedControlLoop = 0xa0009
    uidGetVectorDriveParameters = 0xa000a
    uidSetVectorDriveParameters = 0xa000b
    uidGetEncoderDerivativeParameters = 0xa000c
    uidSetEncoderDerivativeParameters = 0xa000d
    uidGetControlLoopParameters = 0xa000e
    uidSetControlLoopParameters = 0xa000f
    uidStartFrequencyResponse = 0xa0010
    uidStopFrequencyResponse = 0xa0011
    uidStartStepResponse = 0xa0012
    uidStopStepResponse = 0xa0013
    uidStartRampResponse = 0xa0014
    uidStopRampResponse = 0xa0015
    uidSelectCustomData = 0xa0016
    uidGetSelectedCustomData = 0xa0017
    uidSetCommandMode = 0xa0018
    uidClearFaults = 0xa0019
    uidSetServoing = 0xa001a
    uidMoveToPosition = 0xa001b
    uidGetCommandMode = 0xa001c
    uidGetServoing = 0xa001d
    uidGetTorqueOffset = 0xa001e
    uidSetCoggingFeedforwardMode = 0xa001f
    uidGetCoggingFeedforwardMode = 0xa0020



class ActuatorConfigClient():
    
    serviceVersion = 1
    serviceId = 10
    router = RouterClient.RouterClient

    def __init__(self, router: RouterClient.RouterClient):
        self.router = router
        self.notificationHandler = NotificationHandler()
        callback = partial(self.ExecuteRouterNotification)
        self.router.registerNotificationCallback(self.serviceId, callback)

    def ExecuteRouterNotification(self, message):
        self.notificationHandler.call(BitMaskTools.extractFrameId(message.header.message_info), message.payload)


    def GetAxisOffsets(self, deviceId = 0, options = RouterClient.RouterClientSendOptions()):

        future = self.router.send(None, 1, ActuatorConfigFunctionUid.uidGetAxisOffsets, deviceId, options)

        ansPayload = ActuatorConfigPb.AxisOffsets()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

    def SetAxisOffsets(self, axisposition, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = axisposition.SerializeToString()

        future = self.router.send(reqPayload, 1, ActuatorConfigFunctionUid.uidSetAxisOffsets, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





    def ReadTorqueCalibration(self, deviceId = 0, options = RouterClient.RouterClientSendOptions()):

        future = self.router.send(None, 1, ActuatorConfigFunctionUid.uidReadTorqueCalibration, deviceId, options)

        ansPayload = ActuatorConfigPb.TorqueCalibration()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

    def WriteTorqueCalibration(self, torquecalibration, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = torquecalibration.SerializeToString()

        future = self.router.send(reqPayload, 1, ActuatorConfigFunctionUid.uidWriteTorqueCalibration, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





    def SetTorqueOffset(self, torqueoffset, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = torqueoffset.SerializeToString()

        future = self.router.send(reqPayload, 1, ActuatorConfigFunctionUid.uidSetTorqueOffset, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





    def GetControlMode(self, deviceId = 0, options = RouterClient.RouterClientSendOptions()):

        future = self.router.send(None, 1, ActuatorConfigFunctionUid.uidGetControlMode, deviceId, options)

        ansPayload = ActuatorConfigPb.ControlModeInformation()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

    def SetControlMode(self, controlmodeinformation, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = controlmodeinformation.SerializeToString()

        future = self.router.send(reqPayload, 1, ActuatorConfigFunctionUid.uidSetControlMode, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





    def GetActivatedControlLoop(self, deviceId = 0, options = RouterClient.RouterClientSendOptions()):

        future = self.router.send(None, 1, ActuatorConfigFunctionUid.uidGetActivatedControlLoop, deviceId, options)

        ansPayload = ActuatorConfigPb.ControlLoop()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

    def SetActivatedControlLoop(self, controlloop, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = controlloop.SerializeToString()

        future = self.router.send(reqPayload, 1, ActuatorConfigFunctionUid.uidSetActivatedControlLoop, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





    def GetVectorDriveParameters(self, deviceId = 0, options = RouterClient.RouterClientSendOptions()):

        future = self.router.send(None, 1, ActuatorConfigFunctionUid.uidGetVectorDriveParameters, deviceId, options)

        ansPayload = ActuatorConfigPb.VectorDriveParameters()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

    def SetVectorDriveParameters(self, vectordriveparameters, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = vectordriveparameters.SerializeToString()

        future = self.router.send(reqPayload, 1, ActuatorConfigFunctionUid.uidSetVectorDriveParameters, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





    def GetEncoderDerivativeParameters(self, deviceId = 0, options = RouterClient.RouterClientSendOptions()):

        future = self.router.send(None, 1, ActuatorConfigFunctionUid.uidGetEncoderDerivativeParameters, deviceId, options)

        ansPayload = ActuatorConfigPb.EncoderDerivativeParameters()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

    def SetEncoderDerivativeParameters(self, encoderderivativeparameters, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = encoderderivativeparameters.SerializeToString()

        future = self.router.send(reqPayload, 1, ActuatorConfigFunctionUid.uidSetEncoderDerivativeParameters, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





    def GetControlLoopParameters(self, loopselection, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = loopselection.SerializeToString()

        future = self.router.send(reqPayload, 1, ActuatorConfigFunctionUid.uidGetControlLoopParameters, deviceId, options)

        ansPayload = ActuatorConfigPb.ControlLoopParameters()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

    def SetControlLoopParameters(self, controlloopparameters, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = controlloopparameters.SerializeToString()

        future = self.router.send(reqPayload, 1, ActuatorConfigFunctionUid.uidSetControlLoopParameters, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





    def StartFrequencyResponse(self, frequencyresponse, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = frequencyresponse.SerializeToString()

        future = self.router.send(reqPayload, 1, ActuatorConfigFunctionUid.uidStartFrequencyResponse, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





    def StopFrequencyResponse(self, deviceId = 0, options = RouterClient.RouterClientSendOptions()):

        future = self.router.send(None, 1, ActuatorConfigFunctionUid.uidStopFrequencyResponse, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





    def StartStepResponse(self, stepresponse, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = stepresponse.SerializeToString()

        future = self.router.send(reqPayload, 1, ActuatorConfigFunctionUid.uidStartStepResponse, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





    def StopStepResponse(self, deviceId = 0, options = RouterClient.RouterClientSendOptions()):

        future = self.router.send(None, 1, ActuatorConfigFunctionUid.uidStopStepResponse, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





    def StartRampResponse(self, rampresponse, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = rampresponse.SerializeToString()

        future = self.router.send(reqPayload, 1, ActuatorConfigFunctionUid.uidStartRampResponse, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





    def StopRampResponse(self, deviceId = 0, options = RouterClient.RouterClientSendOptions()):

        future = self.router.send(None, 1, ActuatorConfigFunctionUid.uidStopRampResponse, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





    def SelectCustomData(self, customdataselection, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = customdataselection.SerializeToString()

        future = self.router.send(reqPayload, 1, ActuatorConfigFunctionUid.uidSelectCustomData, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





    def GetSelectedCustomData(self, deviceId = 0, options = RouterClient.RouterClientSendOptions()):

        future = self.router.send(None, 1, ActuatorConfigFunctionUid.uidGetSelectedCustomData, deviceId, options)

        ansPayload = ActuatorConfigPb.CustomDataSelection()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

    def SetCommandMode(self, commandmodeinformation, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = commandmodeinformation.SerializeToString()

        future = self.router.send(reqPayload, 1, ActuatorConfigFunctionUid.uidSetCommandMode, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





    def ClearFaults(self, deviceId = 0, options = RouterClient.RouterClientSendOptions()):

        future = self.router.send(None, 1, ActuatorConfigFunctionUid.uidClearFaults, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





    def SetServoing(self, servoing, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = servoing.SerializeToString()

        future = self.router.send(reqPayload, 1, ActuatorConfigFunctionUid.uidSetServoing, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





    def MoveToPosition(self, positioncommand, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = positioncommand.SerializeToString()

        future = self.router.send(reqPayload, 1, ActuatorConfigFunctionUid.uidMoveToPosition, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





    def GetCommandMode(self, deviceId = 0, options = RouterClient.RouterClientSendOptions()):

        future = self.router.send(None, 1, ActuatorConfigFunctionUid.uidGetCommandMode, deviceId, options)

        ansPayload = ActuatorConfigPb.CommandModeInformation()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

    def GetServoing(self, deviceId = 0, options = RouterClient.RouterClientSendOptions()):

        future = self.router.send(None, 1, ActuatorConfigFunctionUid.uidGetServoing, deviceId, options)

        ansPayload = ActuatorConfigPb.Servoing()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

    def GetTorqueOffset(self, deviceId = 0, options = RouterClient.RouterClientSendOptions()):

        future = self.router.send(None, 1, ActuatorConfigFunctionUid.uidGetTorqueOffset, deviceId, options)

        ansPayload = ActuatorConfigPb.TorqueOffset()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

    def SetCoggingFeedforwardMode(self, coggingfeedforwardmodeinformation, deviceId = 0, options = RouterClient.RouterClientSendOptions()):
        reqPayload = coggingfeedforwardmodeinformation.SerializeToString()

        future = self.router.send(reqPayload, 1, ActuatorConfigFunctionUid.uidSetCoggingFeedforwardMode, deviceId, options)

        result = future.result(options.getTimeoutInSecond())





    def GetCoggingFeedforwardMode(self, deviceId = 0, options = RouterClient.RouterClientSendOptions()):

        future = self.router.send(None, 1, ActuatorConfigFunctionUid.uidGetCoggingFeedforwardMode, deviceId, options)

        ansPayload = ActuatorConfigPb.CoggingFeedforwardModeInformation()

        result = future.result(options.getTimeoutInSecond())

        ansPayload.ParseFromString(result.payload)



        return ansPayload

