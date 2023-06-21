from random import randrange
from random import seed
from datetime import datetime
import mso.propagation_param as p
from mso.MsoHandler import MsoHandler
from mso.MsoGrapher import MsoGrapher
from positionHandlers.PositionHandler import PositionHandler
from positionHandlers.PositionHandler_Zeros import PositionHandler_Zeros
from positionHandlers.PositionHandler_Random import PositionHandler_Random
from Vehicles.Vehicle import Vehicle
from Vehicles.Vehicle_UAT import Vehicle_UAT
import time as time


def initVehicles(vehicles, positionHandler):
    for i in range(p.num_vehicles):
        if p.sameFrame:
            uas = Vehicle_UAT(p.frameValue, 0)
        elif p.zeroStart:
            uas = Vehicle_UAT(randrange(0,3600), 0)
        # TODO: Needs to be fixed so that a random previous_random_number is not selected, but rather it is the 
        # necessary previous_random_number according to that frame and starting position
        else:
            uas = Vehicle_UAT(randrange(0,3600))
        positionHandler.initPosition(uas)
        vehicles.append(uas)
    
def initVehicle(vehicles, positionHandler, frame, previousRandomNumber):
    uas = Vehicle_UAT(frame, previousRandomNumber)
    positionHandler.initPosition(uas)
    vehicles.append(uas)

# Runs through as many simulations with all variables defined in propagation_param.py
def main():
    msoHandler = MsoHandler()
    msoGrapher = MsoGrapher()
    for t in range(p.num_simulations):
        #print(t)
        msoHandler.getSimulation(t)
        seed(datetime.now())
        vehicles = []
        positionHandler = PositionHandler_Zeros()
        frame = 0
        rmd1 = 0
        initVehicle(vehicles, positionHandler, frame, rmd1)
        for n in range(p.num_seconds):
            positionHandler.getNextPosition(vehicles[0], n)
            msoHandler.addMso(n, 0, vehicles[0].get_MSO(n))
        # Takes the previous random number from the other vehicle and starts on the next frame
        initVehicle(vehicles, positionHandler, 800, vehicles[0].msos[798] - 752)
        for n in range(p.num_seconds):
            positionHandler.getNextPosition(vehicles[1], n)
            msoHandler.addMso(n, 1, vehicles[1].get_MSO(n))
        
        # msoGrapher.addMsoList(vehicles)
        # msoGrapher.graphMsos()
        msoHandler.evaluateAllMSOs(vehicles)
    msoHandler.performCalculations()


if __name__ == "__main__":
    start = time.time()
    main()
    end = time.time()
    print("Total runtime:", round(end - start,2))
