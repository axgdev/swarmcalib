import calibrationOutput
import time

class KillLog:
    def __init__(self):
        self.running = False
        self.initialTime = time.time()
        self.killSignalReceived = False
        self.position = []
        self.outputFile = calibrationOutput.CSVWriter()
        self.outputFile.setFilenamePostLetters('_kill_log')
        self.outputFile.setHeader('['time','timeDifference','killSignalReceived','x','y','theta']')

    def mainLoop(self):
        while(self.running):
            currentTime = time.time()
            timeDifference = currentTime - self.initialTime
            self.outputFile.append([currentTime,timeDifference,self.killSignalReceived,
                                   self.position[0],self.position[1],self.position[2]])
            time.sleep(0.0001)

    def mainLoopThreaded(self):
        thread = threading.Thread(target=self.mainLoop)
        thread.daemon = True
        thread.start()

    def setPosition(self, position):
        currentTime = time.time()
        timeDifference = currentTime - self.initialTime
        self.outputFile.append([currentTime,timeDifference,self.killSignalReceived,
                               self.position[0],self.position[1],self.position[2]])

    def setPositionThreaded(self, position):
        thread = threading.Thread(target=self.setPosition, args=(position,))
        thread.daemon = True
        thread.start()

    def startLog(self):
        self.running = True
        self.mainLoopThreaded()

    def stopLog(self):
        self.running = False
        self.ouputFile.saveToFile()
