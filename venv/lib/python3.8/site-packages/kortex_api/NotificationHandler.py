import threading

from .Exceptions.KException import KException

class callbackObject:

    def __init__(self, parseNotifDataFromString, callbackFct):
        self.parseFromString = parseNotifDataFromString
        self.callback = callbackFct

    def call(self, payload):
        obj = self.parseFromString(payload)
        self.callback(obj)


class NotificationHandler:

    def __init__(self):
        self.callbackMap = {}
        self.mutex = threading.Lock()

    def __del(self):
        self.clearAll()

    def addCallback(self, idKey, parseFromString, callback):
        workingValue = self.callbackMap.get(idKey)
        if workingValue == None:
            workingValue = [callbackObject(parseFromString, callback)]
        else:
            workingValue.append(callbackObject(parseFromString, callback))

        self.callbackMap[idKey] = workingValue

    def clearIdKeyCallbacks(self, idKey):
        with self.mutex:
            del self.callbackMap[idKey]

    def clearAll(self):
        with self.mutex:
            self.callbackMap.clear()

    def call(self, idKey, msgNotif):
        with self.mutex:
            callbackList = self.callbackMap.get(idKey)

            if (callbackList == None):
                raise KException("No callback list instantiated")
            else:
                for callbackObj in callbackList:
                    callbackObj.call(msgNotif)
