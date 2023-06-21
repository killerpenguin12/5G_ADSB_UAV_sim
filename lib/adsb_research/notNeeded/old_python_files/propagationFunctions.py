'''
This file contains some of the RF functions used in the simulation. 5/5/2020
'''
import numpy as np
import random
import propagation_param as p

'''
# Returns a power density in W/m^2 for a
# certain electric field strength
def getPowerDensityEField(e_field_strength):
    return (e_field_strength**2.0)/(2.0*p.eta_0)

# Returns the electric field strength (V/m)
# of a given power density
def getEFieldStrength(S):
    return np.sqrt(2.0*p.eta_0*S)
'''

def distanceBetweenVehicles(vehicle_t, vehicle_r):
    #distance = np.sqrt((vehicle_to_transmit.LAT[i] - vehicles_sorted[k % p.num_vehicles].LAT[i])**2 + \
    #            (vehicle_to_transmit.LON[i] - vehicles_sorted[k % p.num_vehicles].LON[i])**2)

    #return np.sqrt((vehicle_t.LAT[second] - vehicle_r.LAT[second])**2 + \
    #    (vehicle_t.LON[second] - vehicle_r.LON[second])**2)
    distance = np.sqrt((vehicle_t.position[0] - vehicle_r.position[0])**2.0 + \
        (vehicle_t.position[1] - vehicle_r.position[1])**2.0 + (vehicle_t.position[2] - vehicle_r.position[2])**2.0)

    #print("distance: ", distance)
    if distance == 0.0:
        return 1 # I'm not sure how to handle this case
    return distance

'''This function is not really used now'''
# def countUnsuccessfulDecodes(num_vehicles, sorted_powers):
#     if(len(sorted_powers) == 2):
#         if(sorted_powers[0] - sorted_powers[1] >= 3.0):
#             return len(sorted_powers) - 1
#         else:
#             return len(sorted_powers)
#     elif(len(sorted_powers) == 3):
#         if(sorted_powers[0] - sorted_powers[1] >= 3.0): # if 3 dB difference between first and second
#             return len(sorted_powers) - 1
#         elif(sorted_powers[0] - sorted_powers[2] >= 3.0): # what do we check for now? we need to do a few different cases with 5 vehicles
#             return len(sorted_powers) - 2
#         else:
#             return len(sorted_powers)



# Returns a power density of an
# isotropic radiatior (unit W/m^2) for
# a certain transmission power and distance
def powerDensityPt(ERP, distance):
    return ERP/(4.0*np.pi*(distance**2.0))

# Returns transmitted power for a given
# power density and wavelength
def powerReceived(S, lamb):
    return (S*(lamb**2.0))/(4.0*np.pi)

'''This function is not really used'''
# # Returns thermal noise from a
# # given temperature
# def thermalNoise(temp):
#     return p.k_B*temp*p.B

# Returns a value given as a natural number in dB
def todB(P):
    return 10.*np.log10(P)

# Returns a value given as a natural number in dBm
def todBm(P):
    return todB(P) + 30.

# Returns a value given in dB as a natural number
def toNaturalNumber(P_dB):
    return 10.0**(P_dB/10.0)

def powerReceiveddBm(distance):
    return todBm(powerReceived(powerDensityPt(p.ERP, distance), p.wavelength))

def combinedReceivedPower(vehicle_t, vehicle_r):
    return powerReceiveddBm(distanceBetweenVehicles(vehicle_t, vehicle_r))

'''This function is not really used now'''
# def determineUnsuccessfulDecodes(sorted_powers, index_to_check):
#     for i in range(index_to_check + 1, index_to_check + len(sorted_powers)):
#         if(sorted_powers[index_to_check].power - sorted_powers[i % len(sorted_powers)].power >= p.power_difference_dB) and \
#             (sorted_powers[index_to_check].power >= p.receiver_min_trigger_level):
#             return len(sorted_powers) - 1
#         else:
#             return len(sorted_powers)
