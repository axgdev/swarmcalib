import calibrationOutput
import time

class KillLog:
    def __init__(self):
        self.initialTime = time.time()
        self.killSignalReceived = False
        self.outputFile = calibrationOutput.CSVWriter()
        self.outputFile.setFilenamePostLetters('_kill_log')
        self.outputFile.setHeader(['time','timeDifference','killSignalReceived','x','y','theta'])

    def setPosition(self, position):
        currentTime = time.time()
        timeDifference = currentTime - self.initialTime
        self.outputFile.append([currentTime,timeDifference,self.killSignalReceived,
                                position[0],position[1],position[2]])

    def setPositionThreaded(self, position):
        thread = threading.Thread(target=self.setPosition, args=(position,))
        thread.daemon = True
        thread.start()

    def saveLog(self):
        self.ouputFile.saveToFile()
