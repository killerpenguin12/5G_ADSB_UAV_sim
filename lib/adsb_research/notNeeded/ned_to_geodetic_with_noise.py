# Feb 1, 2021
# Testing adding some noise to coordinates

import numpy as np
import pymap3d as pymap3d
import random as random
import Vehicle as Vehicle

random_param = 0.05

origin_lat = 40.2338
origin_lon = -111.6585
origin_alt = 1387.0

<<<<<<< HEAD
position = (0, 0, 1387)
=======
position = (-10, 1500, -100)
>>>>>>> master

noisy_position = (position[0] + (random_param * random.random() * (1 if random.random() > 0.5 else -1)),
                  position[1] + (random_param * random.random() * (1 if random.random() > 0.5 else -1)),
                  position[2] + (random_param * random.random() * (1 if random.random() > 0.5 else -1)))

clean_coordinate = pymap3d.ned2geodetic(position[0], position[1], position[2],
                                  origin_lat, origin_lon, origin_alt,
                                  pymap3d.Ellipsoid("wgs84"))

print(clean_coordinate)