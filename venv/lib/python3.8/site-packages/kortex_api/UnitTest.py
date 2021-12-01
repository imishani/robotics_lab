import sys, time
from concurrent.futures import TimeoutError

from TransportClientMock import TransportClientMock
from RouterClient import RouterClient, RouterClientSendOptions
from SessionManager import SessionManager
from .autogen.client_stubs.TestClientRpc import TestClient
from .autogen.messages import Test_pb2 as Test_pb2


def printRcvData(headerText, rcvData):
    print("-------- %s --------" % headerText)
    print("Bytes : ", rcvData.bytes_Value)
    print("uint32 : ", rcvData.uint32_Value)
    print("fixed32 : ", rcvData.fixed32_Value)
    print("String : ", rcvData.string_Value)
    print("Enum : ", rcvData.enum_Value)
    print("EmbeddedStruct : ", rcvData.embeddedStruct_Value.embeddedUint)
    print("repeated uint32 : ", rcvData.repeatedUint32_Value)
    print("\n")

def notifCallback(Data):
    print("---- Callback -----\n")
    notifData = Test_pb2.NotifData()
    notifData.ParseFromString(Data)
    print(notifData)
    print("--- End Callback ---\n")

def createTestingValue():
    #the struct declare here must be the same in UnitTest.py
    #if they differ the unit test will fail
    struct = Test_pb2.validateStruct()

    struct.clientStruct.bytes_Value = bytes('Test',"utf-8")
    struct.clientStruct.uint32_Value = 3000
    struct.clientStruct.fixed32_Value = 666
    struct.clientStruct.string_Value = 'Test'
    struct.clientStruct.enum_Value = Test_pb2.ENUM_0
    struct.clientStruct.embeddedStruct_Value.embeddedUint = 1000000
    struct.clientStruct.repeatedUint32_Value.extend([10,20,30])

    bytesValueToReturn = struct.clientStruct.bytes_Value + bytes('1','utf-8')
    struct.serverStruct.bytes_Value = bytesValueToReturn
    struct.serverStruct.uint32_Value = struct.clientStruct.uint32_Value + 1
    struct.serverStruct.fixed32_Value = struct.clientStruct.fixed32_Value + 1
    struct.serverStruct.string_Value = struct.clientStruct.string_Value + "1"
    struct.serverStruct.enum_Value = Test_pb2.ENUM_1
    struct.serverStruct.embeddedStruct_Value.embeddedUint = struct.clientStruct.embeddedStruct_Value.embeddedUint + 1
    struct.serverStruct.repeatedUint32_Value.extend([10,20,30,40])

    return struct

def checkDiffWithTestingValue(receivedData, referenceData, function):
    assert receivedData.bytes_Value == referenceData.bytes_Value, \
    "received bytes_value(%s) differ from is Mock counter part(%s) in function %s" % (receivedData.bytes_Value, referenceData.bytes_Value,function)

    assert receivedData.uint32_Value == referenceData.uint32_Value, \
    "received uint32_Value(%s) differ from is Mock counter part(%s) in function %s" % (receivedData.uint32_Value, referenceData.uint32_Value,function)
    
    assert receivedData.fixed32_Value == referenceData.fixed32_Value, \
    "received fixed32_Value(%s) differ from is Mock counter part(%s) in function %s" % (receivedData.fixed32_Value, referenceData.fixed32_Value,function)
    
    assert receivedData.string_Value == referenceData.string_Value, \
    "received string_Value(%s) differ from is Mock counter part(%s) in function %s" % (receivedData.string_Value, referenceData.string_Value,function)
    
    assert receivedData.enum_Value == referenceData.enum_Value, \
    "received enum_Value(%s) differ from is Mock counter part(%s) in function %s" % (receivedData.enum_Value, referenceData.enum_Value,function)
    
    assert receivedData.embeddedStruct_Value == referenceData.embeddedStruct_Value, \
    "received bytes_embeddedStruct_Valuevalue(%s) differ from is Mock counter part(%s) in function %s" % (receivedData.embeddedStruct_Value, referenceData.embeddedStruct_Value,function)
    
    assert receivedData.repeatedUint32_Value == referenceData.repeatedUint32_Value, \
    "received repeatedUint32_Value(%s) differ from is Mock counter part(%s) in function %s" % (receivedData.repeatedUint32_Value, referenceData.repeatedUint32_Value,function)


def runTest():
    transport = TransportClientMock()
    transport.connect("localhost", 10000)

    router = RouterClient(transport, RouterClient.basicErrorCallback)
    testObj = TestClient(router)

    data = createTestingValue()
    testObj.SetMockValidationStruct(data)

    rcvData = Test_pb2.RcvStruct()
    rcvData = testObj.TestParamAndReturn(data.clientStruct)
    checkDiffWithTestingValue(rcvData, data.serverStruct, "TestParamAndReturn")
    printRcvData("Data from param and return",rcvData)
    
    testObj.TestParamOnly(data.clientStruct)

    rcvData = testObj.TestReturnOnly()
    checkDiffWithTestingValue(rcvData, data.serverStruct, "TestReturnOnly")
    printRcvData("Data from return only", rcvData)

    try:
        testObj.TestTimeout()
    except TimeoutError:
         print("TimeoutError trigered. Work as expected")
    except Exception:
        print("Unexpected exception occurs during test.TestTimeout")

    notifOptions = Test_pb2.NotifOption()
    notifOptions.dummyOption = 3
    notifId = Test_pb2.NotifId()
    notifId = testObj.OnNotificationTestNotif(notifCallback, notifOptions)

    testObj.TestTriggerNotif()

    print("Unsubscribe from notif")
    testObj.TestNotifUnsubscribe(notifId)

    print("Trigger notif, not supposed to received callback")
    #probably need to be surround with try catch
    testObj.TestTriggerNotif()

    print("Async answer under timeout value")
    delay = Test_pb2.timeToResponse()
    delay.time_ms = 500
    testObj.TestAsync(delay)

    print("Async answer above timeout value")
    delay.time_ms = 1500
    try:
        testObj.TestAsync(delay)
    except TimeoutError:
        print("Timeout error .. work as expected")
    except Exception:
        print("Another exception occurs test fail")


if __name__ == "__main__":
    runTest()