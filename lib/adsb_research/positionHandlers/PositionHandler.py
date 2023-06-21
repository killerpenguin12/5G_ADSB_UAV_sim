from abc import abstractmethod
import mso.propagation_param as p

# Interface/Abstract class for positions
class PositionHandler:
    def __init__(self) -> None:
        pass

    @abstractmethod
    def initPosition(self, vehicle):
        pass
    @abstractmethod
    def getNextPosition(self, vehicle, n):
        pass