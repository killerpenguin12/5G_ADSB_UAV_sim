'''
This file is no longer in use. 5/5/2020
These functions are implemented fully in the vehicle class.
'''
import numpy as np

# Returns the 12 L.S.B.'s of the most recent valid "LATITUDE" or
# "LONGITUDE", based on the value of m
def leastSignificantBits(self, m, N_0, N_1):
    if m == 0:
        return N_0
    elif m == 1:
        return N_1
    else:
        print('ERROR: m is invalid in N(0) or N(1)')

# Returns a pseudo-random number
# In order to get R(m -1), this function cannot be used,
# since R(m - 1) is the previously chosen random number.
# If the vehicle latitude or longitude has changed, this
# function may return a different value than the previous
# random number
def pseudoRandomNumberR(self, m, R_m_d1):
    if m == 0:
        return (leastSignificantBits(0) % 3200)
    elif m >= 1:
        return ((4001*R_m_d1 + leastSignificantBits(m % 2)) % 3200)
    else:
        print('ERROR: m is invalid in R(m)')

# See MSO details on pdf page 115 of of DO-B282 (page 85)
# Returns the initial MSO when it is first being calculated
def fullMSORange(self, m, R_m_d1):
    return (752 + pseudoRandomNumberR(m, R_m_d1))

# Returns an MSO once the vehicle is no longer in the full MSO range mode
def restrictedMSORange(self, m, k, R_m_d1):
    # FIXME is R(k) constant? Or is k just constant and R(k) is calculated every time?
    # My interpretation is that k is constant because R(m-1) is defined as a number,
    # but for R(k), k is just defined as a frame--not R(k) being defined as a number
    R_star = pseudoRandomNumberR(k, R_m_d1) - (pseudoRandomNumberR(k, R_m_d1) % 800) # definition of R_star (R*)
    return (752 + R_star + (pseudoRandomNumberR(m, R_m_d1) % 800))
