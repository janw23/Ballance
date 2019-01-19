import ServoControllerModule as SCM
import DataLoggerModule as DLM
from time import sleep

dataLogger = DLM.DataLogger()

for i in range(4):
    a = 0
    for name in dataLogger.dataColumns:
        dataLogger.addRecord(name, 1234 + a + i)
        a += 1
        
    dataLogger.saveRecord()
    
dataLogger.saveToFile("dataTestLog")