from csv import reader
from matplotlib import pyplot
import math

with open('./csvOutput/RefeedRun7/2017-01-03_03-37-59.csv', 'r') as f:
    data = list(reader(f))

time = [i[1] for i in data[1::]]
errorx = [i[23] for i in data[1::]]
errory = [i[24] for i in data[1::]]

absX = [abs(float(x)) for x in errorx]
AvgX = 0.1*(math.floor(sum(absX)/len(absX)*10))
AvgX = str(AvgX)

absY = [abs(float(y)) for y in errory]
AvgY = 0.1*(math.floor(sum(absY)/len(absY)*10))
AvgY = str(AvgY)

pyplot.plot(time, errorx, label='Pitch')
pyplot.plot(time, errory, label='Roll')
pyplot.legend()
pyplot.figtext(0.5,0.8, "AvgX="+AvgX+"\nAvgY="+AvgY)
pyplot.title('Error over Time')
pyplot.xlabel('Time/seconds')
pyplot.savefig('Calibration values over Time RefeedRun 7.png')
pyplot.show()
