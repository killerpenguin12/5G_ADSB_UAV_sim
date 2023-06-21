# adsb_research
Code for simulation of ADS-B with SUAS project. 


The new_main_sim.py is where all the code runs.

Important Folders:
- mso: Has all UAT, MSO, and propagation parameters for the simulation
- Vehicles: Has the vehicle parent class and child classes
- PositionHandlers: Has all classes that make vehicles move


How to run Jonathan's Code with Jaron's code:
- To get a better understanding of the interaction between C++ and Python, I used the Python/C API reference manual in the Python docs website
- In the sim.yaml and the propagation_param.py, make sure the number of vehicles and number of seconds are the same in both files
- The UAT_Model.py is the file called and run by uav_sim_node.cpp. To see how uav_sim_node is run, refer to the README in the master branch on Jarons uav_sim repository
- There are still a few things that need to be done with Jarons code to make it the most efficient possible in order for each vehicle to "see" each other without an extra for loop. To do this, the C/Python API reference manual may need to switch to C++ only or switch to PyBind