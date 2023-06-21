
from positionHandlers.PositionHandler import PositionHandler
from mso.UAT import UAT
import mso.propagation_param as p
from progress.bar import IncrementalBar
# Only lands in one spot for every second
class PositionHandler_Zeros(PositionHandler):
    def __init__(self) -> None:
        super().__init__()
        self.msos = []

    def initPosition(self, vehicle):
        self.getZeroPosition(vehicle)

    def getNextPosition(self, vehicle, n):
        self.getZeroPosition(vehicle)

    def getZeroPosition(self, vehicle):
        vehicle.update_position(0,0,0)

    def getPaths(self):
        paths = []
        bar = IncrementalBar("Getting Paths...", max=p.num_paths)
        for i in range(p.num_paths):
            lat = []
            long = []
            alt = []
            for n in range(p.maxFlightTime):
                lat.append(0)
                long.append(0)
                alt.append(0)
            paths.append((lat,long,alt))
            bar.next()
        bar.finish()
        return paths

