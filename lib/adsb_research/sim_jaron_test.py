from UAT_Model import UAT_Model
import mso.propagation_param as p
from random import randrange

model = UAT_Model()

for i in range(p.num_vehicles):
    model.xPos = randrange(0, 10)
    model.yPos = randrange(0, 10)
    model.zPos = 0#randrange(0, 1000)
    #model.timeIndex = i/100
    model.vehicleNum = i
    for j in range(p.num_seconds):
        #model.vehicleNum = j
        model.propagate()
model.printResults()
print("Done")
