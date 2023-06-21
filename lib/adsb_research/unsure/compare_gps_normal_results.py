# Feb 1, 2021
# I wrote this script to compare some the results between normal GPS coordinates

import pymap3d as pymap3d
import propagation_param as p
import numpy as np
from Vehicle import Vehicle
from Vehicle_gps import VehicleGPS

gps_coordinate1 = (40.233709, -111.675637, 1487.000)
gps_coordinate2 = (40.233799, -111.675520, 1487.000)

position1 = pymap3d.geodetic2ned(gps_coordinate1[0], gps_coordinate1[1], gps_coordinate1[2],
                                 p.initial_lat, p.initial_lon, p.initial_altitude,
                                 pymap3d.Ellipsoid("wgs84"))

position2 = pymap3d.geodetic2ned(gps_coordinate2[0], gps_coordinate2[1], gps_coordinate2[2],
                                 p.initial_lat, p.initial_lon, p.initial_altitude,
                                 pymap3d.Ellipsoid("wgs84"))

example_vehicle1 = Vehicle(position1, 1, 1)
example_vehicle2 = Vehicle(position2, 1, 2)


example_gps_vehicle1 = VehicleGPS(gps_coordinate1, 1, 1)
example_gps_vehicle2 = VehicleGPS(gps_coordinate2, 1, 2)

example_gps_vehicle1.update_position(position1)
gps_mso11 = example_gps_vehicle1.full_mso_range()
example_gps_vehicle1.frame += 1
gps_mso12 = example_gps_vehicle1.full_mso_range()

example_gps_vehicle1.update_position(position2)
gps_mso21 = example_gps_vehicle2.full_mso_range()
example_gps_vehicle1.frame += 1
gps_mso22 = example_gps_vehicle2.full_mso_range()

example_vehicle1.update_position(position1)
mso11 = example_vehicle1.full_mso_range()
example_vehicle1.frame += 1
mso12 = example_vehicle1.full_mso_range()

example_vehicle2.update_position(position2)
mso21 = example_vehicle2.full_mso_range()
example_vehicle2.frame += 1
mso22 = example_vehicle2.full_mso_range()

print("Position 1:", position1)
print("Position 2:", position2)
print("Distance:", np.sqrt((position1[0] - position2[0])**2 +
      (position1[1] - position2[1])**2 +
      (position1[2] - position2[2])**2))
print("Possible MSOs for 1 before NED conversion:", (gps_mso11, gps_mso12))
print("Possible MSOs for 2 before NED conversion:", (gps_mso21, gps_mso22))
print("Possible MSOs for 1 after NED conversion:", (mso11, mso12))
print("Possible MSOs for 2 after NED conversion:", (mso21, mso22))

# Also figure out what amount of accuracy is achieved with the decimal places that they show

# But am I evaluating this the wrong way? I need to imitate when I read in the data, perhaps.
# Or maybe I can try to use the sim file I normally use, but convert positions to NED first. Not sure
