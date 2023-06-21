'''
Contains dataclasses used like C++ structs in the code. 5/5/2020
'''
from dataclasses import dataclass

''' These classes are basically c-like structs that I use to group data '''

@dataclass
class MSO_collision:
    second: int
    MSO: int
    vehicles_involved: list

@dataclass
class vehicle_power:
    power: float
    vehicle_ID: int
    distance: float

@dataclass
class transmission:
    # second: int
    # MSO: int
    # transmitter_ID: int
    receiver_ID: int
    distance: float
    received_power: float
    # transmitter_frame: int
    successful_decode: bool
