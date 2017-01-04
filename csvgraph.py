from csv import reader
from matplotlib import pyplot
import math


"""open file:
./csvOutput/RefeedRunX for refeed runs
./csvOutput/FreshRuns/ for fresh runs
"""
with open('./quickcsv/refeed7-1.csv', 'r') as f:
    data = list(reader(f))

"""select columns, 23/24 for pitch/roll, 9/10 for errorx/"""


time = [i[1] for i in data[1::]]
errorx = [i[9] for i in data[1::]]  
errory = [i[10] for i in data[1::]]
"""calculate average error values, comment out for calib values"""

absX = [abs(float(x)) for x in errorx]
AvgX = 0.1*(math.floor(sum(absX)/len(absX)*10))
AvgX = str(AvgX)

absY = [abs(float(y)) for y in errory]
AvgY = 0.1*(math.floor(sum(absY)/len(absY)*10))
AvgY = str(AvgY)
pyplot.figtext(0.5,0.8, "AvgX="+AvgX+"\nAvgY="+AvgY)


"""final calib values, comment out for error values"""
"""
finalPitch=errorx[-1]
finalRoll=errory[-1]
finalPitch=finalPitch[:7]
finalRoll=finalRoll[:7]
pyplot.figtext(0.5,0.8, "Pitch="+str(finalPitch)+"\nRoll="+str(finalRoll))
"""

pyplot.plot(time, errorx, label='errorx')
pyplot.plot(time, errory, label='errory')
pyplot.legend()

pyplot.title('error over Time')
pyplot.xlabel('Time/seconds')
pyplot.savefig('./quickcsv/error values over Time refeed7-1.png')
pyplot.show()
