
from mso.UAT import UAT
from Vehicles.Vehicle_UAT import Vehicle_UAT
from Vehicles.Vehicle import Vehicle
from positionHandlers.PositionHandler import PositionHandler
import mso.propagation_param as p
from progress.bar import IncrementalBar
# Only lands in one spot for every second
class PositionHandler_Straight(PositionHandler):
    def __init__(self) -> None:
        super().__init__()
        self.msos = []

    def initPosition(self, vehicle):
        pass

    def getNextPosition(self, vehicle, n):
        pass

    def getZeroPosition(self, vehicle):
        pass

    def getPaths(self):
        print("Does this do anything?")
        paths = []
        for i in range(p.num_paths):
            lat = []
            long = []
            alt = []
            for n in range(p.maxFlightTime):
                lat.append(i*(2*p.radius_of_area//p.num_paths) + p.latMeterDist)
                long.append(4*p.lonMeterDist*n - p.lonMeterDist)
                alt.append(p.middle_altitude)
            paths.append((lat,long,alt))
        return paths