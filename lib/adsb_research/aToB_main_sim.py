from positionHandlers.PositionHandler_Straight import PositionHandler_Straight
from random import randrange
from random import seed
import numpy as np
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
import math
from progress.bar import IncrementalBar

def initVehicles(vehicles, paths, msoHandler):
    bar = IncrementalBar("Initializing Vehicles...", max=p.num_vehicles)
    for i in range(p.num_vehicles):
        pathNum = randrange(0, len(paths))
        if p.sameFrame:
            startFrame = randrange(0, p.maxFlightTime - p.num_seconds - 1)
            lat = paths[pathNum][0][startFrame:startFrame+p.num_seconds]
            lon = paths[pathNum][1][startFrame:startFrame+p.num_seconds]
            alt = paths[pathNum][2][startFrame:startFrame+p.num_seconds]
            msos = msoHandler.addMsos(lat, lon, alt)
        else:
            startFrame = randrange(0, p.maxFlightTime - p.num_seconds - 1)
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

def get_distance(transmitCoord, receivingCoord):
        return np.int(np.ceil(np.sqrt((transmitCoord[0] - receivingCoord[0]) ** 2.0 +
                                  (transmitCoord[1] - receivingCoord[1]) ** 2.0 +
                                  (transmitCoord[2] - receivingCoord[2]) ** 2.0)))

# Runs through as many simulations with all variables defined in propagation_param.py
def main():
    seed(datetime.now())
    msoHandler = MsoHandler()
    vehicle = Vehicle_UAT()
    msoSet = []
    nextPointReached = True
    for i in range(800):
        msoSet.append(752 + i*4)
    print(msoSet)
    point_a_x_pos = 0
    point_a_y_pos = 0
    point_b_x_pos = randrange(10, 2000)
    point_b_y_pos = randrange(10, 2000)
    vehicle_x_pos = point_a_x_pos
    vehicle_y_pos = point_a_y_pos
    y_lim = 10
    x_lim = 10
    lim_count = 0
    second = 0
    vehicle.update_position(vehicle_y_pos, vehicle_x_pos, 0)
    vehicle.addMso(msoHandler.getMso(vehicle.getCoordinates(second)))
    second += 1
    while (vehicle_x_pos != point_b_x_pos and vehicle_y_pos != point_b_y_pos) and second != p.num_seconds:
        if not nextPointReached:
            y_lim += 10
            x_lim += 10
        nextPointReached = False
        m = (point_b_y_pos - point_a_y_pos)/(point_b_x_pos - point_a_x_pos)
        y = m*(vehicle_x_pos - point_a_x_pos) + point_a_y_pos
        x = (vehicle_x_pos - point_a_x_pos)
        if get_distance(vehicle.getCoordinates(second-1), (point_b_x_pos, point_b_y_pos, 0)) < get_distance(vehicle.getCoordinates(second-1), (x + x_lim, y + y_lim, 0)):
            vehicle_x_pos = point_b_x_pos
            vehicle_y_pos = point_b_y_pos
            break
        try:
            for i in range(1, y_lim):
                for j in range(1, x_lim):
                    lat = y+i*p.latMeterDist
                    lon = x+j*p.lonMeterDist
                    mso = msoHandler.getMso((lat, lon, 0))
                    if mso in msoSet:
                        vehicle.update_position(lat, lon, 0)
                        vehicle_x_pos = lon
                        vehicle_y_pos = lat
                        vehicle.addMso(mso)
                        second += 1
                        nextPointReached = True
                        if y_lim != 10 and x_lim != 10:
                            print("y_lim: ", y_lim)
                            print("x_lim: ", x_lim)
                            lim_count += 1
                            print("Count: ", lim_count)
                        y_lim = 10
                        x_lim = 10
                        raise 
        except:
            pass

    print("Vehicle Msos: \n", vehicle.getMsos())
    plt.plot(vehicle.getLongitudes(), vehicle.getLatitudes())
    plt.xlabel('Longitude')
    plt.ylabel("Latitude")
    plt.title('Vehicle position over time')
    plt.show(block=False)
    input("Press enter to end")

                


    # for t in range(p.num_simulations):
    #     start = time.time()
    #     seed(datetime.now())
    #     msoCalculator.getSimulation(t)
    #     vehicles = []
    #     positionHandler = PositionHandler_Random()
    #     paths = positionHandler.getPaths()
    #     initVehicles(vehicles, paths, msoHandler)
    #     msoCalculator.addMsos(vehicles)
    #     msoCalculator.evaluateAllMSOs(vehicles)
    #     end = time.time()
    #     print("Time to run simulation: ", end - start)   
    # msoCalculator.performCalculations()
    

if __name__ == "__main__":
    start = time.time()
    main()
    end = time.time()
    print("Total runtime:", round(end - start,3))
