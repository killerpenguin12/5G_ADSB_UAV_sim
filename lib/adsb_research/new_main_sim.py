from positionHandlers.PositionHandler_Straight import PositionHandler_Straight
from random import randrange
from random import seed
from datetime import datetime
import matplotlib.pyplot as plt
import mso.propagation_param as p
from mso.MsoCalculator import MsoCalculator
from mso.MsoHandler import MsoHandler
from mso.Grapher import Grapher
from positionHandlers.PositionHandler import PositionHandler
from positionHandlers.PositionHandler_Zeros import PositionHandler_Zeros
from positionHandlers.PositionHandler_Straight import PositionHandler_Straight
from positionHandlers.PositionHandler_Random import PositionHandler_Random
from Vehicles.Vehicle import Vehicle
from Vehicles.Vehicle_UAT import Vehicle_UAT
import time as time
from progress.bar import IncrementalBar

def initVehicles(vehicles, paths, msoHandler, t):
    bar = IncrementalBar("Initializing Vehicles...", max=p.num_vehicles)
    for i in range(p.num_vehicles):
        pathNum = i%len(paths)
        # startFrame = i%p.maxFlightTime
        startFrame = i*(t+1)
        if p.sameFrame:
            lat = paths[pathNum][0][startFrame:startFrame+p.num_seconds]
            lon = paths[pathNum][1][startFrame:startFrame+p.num_seconds]
            alt = paths[pathNum][2][startFrame:startFrame+p.num_seconds]
            msos = msoHandler.addMsos(lat, lon, alt)
        else:
            lat = paths[pathNum][0]
            lon = paths[pathNum][1]
            alt = paths[pathNum][2]
            msos = msoHandler.addMsos(lat, lon, alt)[startFrame:startFrame+p.num_seconds]
            lat = lat[startFrame:startFrame+p.num_seconds]
            lon = lon[startFrame:startFrame+p.num_seconds]
            alt = alt[startFrame:startFrame+p.num_seconds]
        uas = Vehicle_UAT(lat, lon, alt, msos)
        vehicles.append(uas)
        bar.next()
    bar.finish()

# Runs through as many simulations with all variables defined in propagation_param.py
def main():
    msoCalculator = MsoCalculator()
    msoHandler = MsoHandler()
    for t in range(p.num_simulations):
        start = time.time()
        seed(datetime.now())
        msoCalculator.getSimulation(t)
        vehicles = []
        positionHandler = PositionHandler_Zeros()
        paths = positionHandler.getPaths()
        initVehicles(vehicles, paths, msoHandler, t)
        msoCalculator.addMsos(vehicles)
        msoCalculator.evaluateAllMSOs(vehicles)
        end = time.time()
        print("Time to run simulation: ", end - start)   
        msoCalculator.performCalculations(t)


    

if __name__ == "__main__":
    start = time.time()
    main()
    end = time.time()
    print("Total runtime:", round(end - start,3))










        # # To check if the decodes were correct
        # while True:
        #     transmitVehicle = input("Transmit Vehicle: ")
        #     if transmitVehicle == "":
        #         break
        #     receivingVehicle = input("Receiving Vehicle: ")
        #     print(msoCalculator.potentialDecodes[9][int(transmitVehicle)][int(receivingVehicle)])
