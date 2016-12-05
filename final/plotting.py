import matplotlib.pyplot as plt

print('test')
data = open('logtxt.txt','r')
dataList = []
print('data open')
for i in data:
  values = map(float,i.split())
  dataList.append(values)
print 'second test'
time=[]
actualAngleLeft = []
actualAngleRight = []
referenceAngleLeft = []
referenceAngleRight = []
print 'third test'
for row in dataList:
    if(len(row) != 1):
        time.append(row[0])
        referenceAngleLeft.append(row[1])
        actualAngleLeft.append(row[2])
        referenceAngleRight.append(row[3])
        actualAngleRight.append(row[4])
print 'fourth test'
plt.plot(time,actualAngleLeft,color='blue')
plt.plot(time,referenceAngleLeft,color='green')
print 'fifth test'