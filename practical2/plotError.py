import matplotlib.pyplot as plt


data = open('logtxt.txt', 'r')
dataList = []

print('data open')
for i in data:
    values = map(float, i.split())
    dataList.append(values)

time=[]
actualAngleOne = []
actualAngleTwo = []
referenceAngleOne = []
referenceAngleTwo = []

"""
for j in range(0,1000):
    time.append(dataList[j][0])
    actualAngleOne.append(dataList[j][1])
    referenceAngleOne.append(dataList[j][3])
"""

for row in dataList:
    if(len(row) == 5):
        time.append(row[0])
        referenceAngleOne.append(row[1])
        actualAngleOne.append(row[2])
        referenceAngleTwo.append(row[3])
        actualAngleTwo.append(row[4])


"""
actual = plt.plot(time, actualAngleOne, label="actualAngle")
reference = plt.plot(time, referenceAngleOne, label="referenceAngle")
plt.legend([actual, reference], ['Actual', 'Reference'])
"""


f, ax = plt.subplots(2, sharex=True, sharey='col')
a = ax[0].plot(time, actualAngleOne, color='blue', label='Actual')
b = ax[0].plot(time, referenceAngleOne, color='green', label='Reference')
ax[0].set_title('Left')

ax[1].plot(time, actualAngleTwo, color='blue', label='Actual')
ax[1].plot(time, referenceAngleTwo, color='green', label='Reference')
ax[1].set_title('Right')

plt.xlabel('Time')
plt.ylabel('Position')

plt.legend(loc = 'lower right')
# plt.xlabel('Time')
# plt.ylabel('Error')

# plt.plot(time, actualAngleTwo, color='blue')
# plt.plot(time, referenceAngleTwo, color='green')

#plt.plot(time, referenceAngleOne)
plt.show()
