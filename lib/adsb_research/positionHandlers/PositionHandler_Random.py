from positionHandlers.PositionHandler import PositionHandler
from mso.UAT import UAT
from random import randrange
from random import seed
from datetime import datetime
import mso.propagation_param as p
from progress.bar import IncrementalBar

# Lands in a random position somewhat close to its previous position every second
class PositionHandler_Random(PositionHandler):
    def __init__(self) -> None:
        self.msos = []
        pass

    def initPosition(self, vehicle):
        pass

    def getNextPosition(self, vehicle, n):
        pass

    # Calculates a random position for the vehicle
    def getRandomPosition(self, coord, second):
        return randrange(coord[second-1]-p.highest_speed, coord[second-1]+p.highest_speed)

    def getPaths(self):
        paths = []
        seed(datetime.now())
        for i in range(p.num_paths):
            lat = [randrange(0, 2*p.radius_of_area)]
            long = [randrange(0, 2*p.radius_of_area)]
            alt = [randrange(p.initial_altitude + p.lowest_altitude, p.initial_altitude + p.highest_altitude)]
            for n in range(p.maxFlightTime):
                lat.append(self.getRandomPosition(lat, n))
                long.append(self.getRandomPosition(long, n))
                alt.append(self.getRandomPosition(alt, n))
            paths.append((lat,long,alt))
        return paths