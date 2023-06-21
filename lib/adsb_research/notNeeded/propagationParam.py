'''
Constants used for simulation. Also imports positional data. 5/5/2020
'''
import sys
import numpy as np

#import flightPaths.openLogFiles as olf
# from dec2bin import dec2bin

# Physical Constants
c_0 = 2.998 * 10.0**8.0 # m/s, propagation velocity
mu_0 = 4.0*np.pi*10.0**-7.0# H/m permeability of free space
epsilon_0 = 8.854*10.0**-12.0 # F/m, permittivity of free space
k_B = 1.38064852*10.0**-23.0 # J/K, Boltzmann Constant
eta_0 = np.sqrt(mu_0/epsilon_0) # Ohms, impedance of free space

# Frequency parameters for 978 MHz UAT
f = 978.0 * (10.0**6.0) # Hz, desired transmission frequency
wavelength = c_0/f # m, wavelength of 978 MHz
k = (2.0*np.pi)/wavelength # rad/m, wavenumber of 978 MHz

# UAT Parameters
ERP = 0.01 # W, power delivered to antenna
B = 1.3*10.0**6.0 # Hz, receiving bandwidth

# Simulation constants
power_difference_dB = 4.0
sample_rate = 1000 # Hz, in Jaron's simulation
receiver_min_trigger_level = -93.0 # dBm
lat0 = 40.233845 # degrees N, Provo
lon0 = -111.658531 # degrees W, Provo
h0 = 1387. # m, altitude above sea level Provo
LSB = 360./(2.**24.) # degrees, conversion factor from DO-282B

'''
Now we import positional data
'''
'''
The two variables below should be verified before simulation
'''
folderName = 'fifteenVehicles'
num_vehicles = 15

#sys.path.append(folderName)

stateType = np.dtype([('t',np.float64),
    ('p',np.float64,(3,1)),
    ('v',np.float64,(3,1)),
    ('lin_accel',np.float64,(3,1)),
    ('q',np.float64,(4,1)),
    ('omega',np.float64,(3,1)),
    ('ang_accel',np.float64,(3,1))])

eulerType = np.dtype([('t',np.float64),
    ('ang',np.float64,(3,1))])

vehicleStates = []
vehicleEulers = []
vehiclePositions = []

for i in range(num_vehicles):
    vehicleStates.append(np.fromfile('flightPaths/'+folderName+'/quad'+str(i)+'_true_state.log', dtype=stateType))
    vehicleEulers.append(np.fromfile('flightPaths/'+folderName+'/quad'+str(i)+'_euler_angles.log', dtype=eulerType))
    # vehicleStates.append(np.fromfile('/tmp/quad'+str(i)+'_true_state.log', dtype=stateType))
    # vehicleEulers.append(np.fromfile('/tmp/quad'+str(i)+'_euler_angles.log', dtype=eulerType))

num_seconds = int(np.floor(len(vehicleStates[0]['t'])/sample_rate))
one_percent = np.round_(num_seconds/100.)
used_samples = num_seconds * sample_rate
print(used_samples)
# MSO range from 752 to 3951 in ADS-B segment
# for each second each vehicle transmits, and each transmission should be
# decoded by all of the other vehicles
num_desired_decodes = num_seconds*num_vehicles*(num_vehicles - 1)

if(num_vehicles != len(vehicleStates)):
    txt = "The following don't match: \np.num_vehicles: {} \nVehicles in data: {}"
    print(txt.format(num_vehicles, len(vehicleStates)))
if(num_seconds != np.floor(len(vehicleStates[0]['t'])/sample_rate)):
    txt = "The following don't match: \np.num_seconds: {} \nSeconds in data: {}"
    print(txt.format(num_seconds, np.floor(len(vehicleStates[0]['t'])/sample_rate)))

print(len(vehicleStates[i]['p'][0 : used_samples : sample_rate]))
for i in range(num_vehicles):
    positions = vehicleStates[i]['p'][0 : used_samples : sample_rate].copy()
    vehiclePositions.append(positions)
