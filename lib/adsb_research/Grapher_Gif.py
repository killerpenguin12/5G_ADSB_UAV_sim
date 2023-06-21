import numpy as np
import pandas as pd
from scipy import stats
import matplotlib.pyplot as plt
from PIL import Image
import matplotlib.cm as cm
import glob # Need this to load photos (better than holding them in RAM forever)
import os

def plotSim(t,data,collision,crashes):
    # noiseFreePathColor = '#00FF00'
    # noisyPathColor = '#0000FF'
    # estimatedPathColor = '#FFBB00'

    # noiseFreeBearingColor = '#00FFFF'
    # observedBearingColor = '#FF0000'
        
    #=================================================
    # data *not* available to your filter, i.e., known
    # only by the simulator, useful for making error plots
    #=================================================
    # actual position (i.e., ground truth)
   
    point_size = 10
    radius = 2513
    #################################################
    # Graphics
    #################################################
    plt.clf() # clear the frame.
    # print("len data: ",len(data))
    #print("t: ",t)
    fig = plt.figure(1)
    ax = fig.add_subplot(111)
    circle = plt.Circle((0, 0), radius, edgecolor="k", facecolor="none")  # Create black circle
    plt.gca().add_artist(circle)
    cmap = plt.cm.get_cmap('viridis')
    #print("len col: ",len(collision))
    #colors = iter(cm.rainbow(np.linspace(0, 1, len(collision))))
    overlap = collision.loc[collision["type"] == "Overlap"]
    closeness = collision.loc[collision["type"] == "Closeness"]
    collisions = collision.loc[collision["type"] == "Collision"]
    lists = np.linspace(0, 1, len(collision))
    colors = cm.rainbow(np.linspace(0, 1, len(collision)))
    for i in range(len(data)):
        # print("data: ",data[i][0][t])
        # print("data 2: ",data[i][1][t])
        # print("data gen: ",data[i])
        # x = data[i][0][t]
        # y = data[i][1][t]
        #print("x: ",x," ,y: ",y)
        
        if(not collision.empty):
        #for i in range(len(collision)):
            c_x1 = collisions["Veh_1_X"].values
            c_y1 = collisions["Veh_1_Y"].values
            c_x2 = collisions["Veh_2_X"].values
            c_y2 = collisions["Veh_2_Y"].values

            o_x1 = overlap["Veh_1_X"].values
            o_y1 = overlap["Veh_1_Y"].values
            o_x2 = overlap["Veh_2_X"].values
            o_y2 = overlap["Veh_2_Y"].values

            cl_x1 = closeness["Veh_1_X"].values
            cl_y1 = closeness["Veh_1_Y"].values
            cl_x2 = closeness["Veh_2_X"].values
            cl_y2 = closeness["Veh_2_Y"].values

            # x1 = collisions["Veh_1_X"].values
            # y1 = collisions["Veh_1_Y"].values
            # x2 = collisions["Veh_2_X"].values
            # y2 = collisions["Veh_2_Y"].values
        
    #   for i in range(len(crashes)):
        if(not crashes.empty):
            x1c = crashes["Veh_1_X"].values
            y1c = crashes["Veh_1_Y"].values
            x2c = crashes["Veh_2_X"].values
            y2c = crashes["Veh_2_Y"].values
        
        
        if(not collision.empty):
            # print("i: ",i)
            # print("len(x1): ",len(x1))
            #for j in range(len(x1)):
            #color_now = next(colors)
            # ax.scatter(x1,y1,marker="D", s=point_size * 2,c = lists,cmap="viridis")
            # ax.scatter(x2,y2,marker="D", s=point_size * 2,c = lists,cmap="viridis")
            ax.scatter(cl_x1,cl_y1,marker="1", s=point_size * 7,color = 'b')
            ax.scatter(cl_x2,cl_y2,marker="1", s=point_size * 7,color = 'b')
            ax.scatter(o_x1,o_y1,marker="s", s=point_size * 1.5,color = 'g')
            ax.scatter(o_x2,o_y2,marker="s", s=point_size * 1.5,color = 'g')
            ax.scatter(c_x1,c_y1,marker="D", s=point_size * 1.5,color = 'r')
            ax.scatter(c_x2,c_y2,marker="D", s=point_size * 1.5,color = 'r')
        #ax.scatter(x,y,color = 'm',s=5)#color = cmap(int(i/len(data))),s=5)
        if(not crashes.empty):
            ax.scatter(x1c,y1c,marker="x", c='r', s=point_size * 20,linewidth=0.5)
            ax.scatter(x2c,y2c,marker="x", c='r', s=point_size * 20,linewidth=0.5)
    #print("Done loop")
    for i in range(len(data)):
        # print("data: ",data[i][0][t])
        # print("data 2: ",data[i][1][t])
        # print("data gen: ",data[i])
        x = data[i][0][t]
        y = data[i][1][t]
        #print("x: ",x," ,y: ",y)
        ax.scatter(x,y,color = 'm',s=5)#color = cmap(int(i/len(data))),s=5)
    ax.set_xlim([-radius,radius])
    ax.set_ylim([-radius,radius])
    #ax.set_xlim([-2000,-1000]) #Lower left point crash
    #ax.set_ylim([1000,2000]) 
    # ax.set_xlim([-1800,-1600])
    # ax.set_ylim([1700,1900])
    ax.set_autoscale_on(False)
    ax.set_title("UAV movements")
    #plt.pause(0.01)
    # plt.xticks(np.arange(-1000,1000,step=100))
    # plt.yticks(np.arange(-1000,1000,step=100))
    #ax.clear()
radius = 2513 
intereting_x = [-radius,radius]
interesting_y = [-radius,radius]
#intereting_x = [-2000,-1000]
#interesting_y = [1000,2000]
#firstread = pd.read_csv('Jonathan_Position_test.csv')
firstread = pd.read_csv('Positions.csv',header=None)
#firstread = pd.read_csv('Jonathan_Position_test.csv')
Path = os.path.isfile('Crashes.csv')
Coll1 = pd.read_csv('Collisions.csv',header=None)
if Path == True:
    Crashes = pd.read_csv('Crashes.csv',header=None)
    Crashes.columns = ["Veh_1", "Veh_2", "Veh_1_X", "Veh_1_Y","Veh_2_X","Veh_2_Y","t"]
else:
    Crashes = pd.DataFrame(columns = ["Veh_1", "Veh_2", "Veh_1_X", "Veh_1_Y","Veh_2_X","Veh_2_Y","t"])
Coll1.columns = ["Veh_1", "Veh_2", "Veh_1_X", "Veh_1_Y","Veh_2_X","Veh_2_Y","t","type"]

firstread.columns = ["id","x","y"]
vehicles = []
t = 60
dt = 0.01
num_vehicles = 100
num_vehicles_bot = 0
number_of_steps = t/dt
printNumLength = len(str(t))
for i in range(num_vehicles_bot,num_vehicles): # Number of vehicles in Positions
    vehicles.append(firstread.loc[firstread['id'] == i]) #seperate all vehicles by id number
data = []
for i in range(len(vehicles)):
    #vals = []
    vals = [vehicles[i]['x'].values,vehicles[i]['y'].values]
    data.append(vals)
plt.ion()
for i in range(t): #Time of simulation in sim.yaml
    #data = []
    #collisions = []
    Coll2 = Coll1.loc[(Coll1["Veh_1_X"] < intereting_x[1]) & (Coll1["Veh_1_X"] > intereting_x[0])]
    Coll_X = Coll2.loc[(Coll2["Veh_2_X"] < intereting_x[1]) & (Coll2["Veh_2_X"] > intereting_x[0])] #Only get the vehicles we want
    Coll_Y_1 = Coll_X.loc[(Coll_X["Veh_1_Y"] < interesting_y[1]) & (Coll_X["Veh_1_Y"] > interesting_y[0])]
    Coll = Coll_Y_1.loc[(Coll_Y_1["Veh_2_Y"] < interesting_y[1]) & (Coll_Y_1["Veh_2_Y"] > interesting_y[0])]
    collisions = Coll.loc[(Coll["t"] < i) & (Coll["t"] > (i - 2))] #Get all our collisions for this second
    #collisions = Coll.loc[(Coll["t"] < i)]
    #collisions = collisions1.loc[(Coll["t"] > (i - 1))]
    crashes = Crashes.loc[(Crashes["t"] < i) & (Crashes["t"] > (i - 2))] #get all the crashes for this second
    #crashes = []
    #for j in range(len(vehicles[i])):
    #    print("Vehicles: ",vehicles[i][j].values)
    #    print("Vehicles j: ",vehicles[j].values)
    #    data.append(vehicles[i][j]) #get all our positions for this second
    #print("data: ",data[i])
    plotSim(i,data,collisions,crashes)
    #plt.legend()
    #print("Here is i: ",i)
    plt.gcf().canvas.draw()
    imgData = np.frombuffer(plt.gcf().canvas.tostring_rgb(), dtype=np.uint8) # Extra image data from the plot
    w, h = plt.gcf().canvas.get_width_height() # Determine the dimensions
    mod = np.sqrt(imgData.shape[0]/(3*w*h)) # multi-sampling of pixels on high-res displays does weird things, account for it.
    im = imgData.reshape((int(h*mod), int(w*mod), -1)) # Create our image array in the right shape
    Image.fromarray(im).save("outputGif/sim_" + str(i).zfill(printNumLength) + ".png") # And pass it to PIL to save it.
    plt.gcf().canvas.flush_events()
plt.show()
plt.ioff()

# Create the frames
frames = [] # This holds each image
imgs = glob.glob("outputGif/sim_*.png") # load 'em in
imgs.sort() # Make sure they're in the right order
for i in imgs: # For each one, we'll open and append
    new_frame = Image.open(i)
    frames.append(new_frame)

# Save into a GIF file that loops forever
frames[0].save('outputGif/output.gif', format='GIF',
                append_images=frames[1:],
                save_all=True,
                duration=len(imgs)*4.00, loop=6)
plt.show()



    
def plotRobot(pose=[0, 0, 0], color="red", fillColor=None, r=13):
    """This will draw the robot on the field, all nice and pretty

    Parameters: 
    pose: Pose [x, y, theta] of the robot to plot, default [0, 0, 0] but why would you use the default?
    color: What color (use hex for #RGBA) to make the outline of the robot - default red, full opacity
    fillColor: What color to fill the robot, defaults to none
    
    Return: 
    None
    """
    plotCircle(pose[:2], r, color=color, fillColor = fillColor)
    plt.plot(np.array([pose[0], pose[0] + np.cos(pose[2])*r*1.5]), np.array([pose[1], pose[1] + np.sin(pose[2])*r*1.5]), color="black")

def plotCircle(center=[0, 0], radius=1, NOP=360, color="red", fillColor=None):
    """This will plot a circle - which I sincerely hope you can figure out.

    Parameters: 
    center: Center of the circle to be plotted, [x, y] default [0, 0]
    radius: The radius of the circle to be plotted, default 1
    NOP: The number of points the define the circle, default 360 which might be overkill
    color: What color (use hex for #RGBA) to make the border - default red, full opacity
    fillColor: what color to fill the circle, defaults to none.
    
    Return: 
    None
    """
    center = np.array(center)
    t = np.linspace(0, 2*np.pi, NOP)
    X = center[0] + radius*np.cos(t)
    Y = center[1] + radius*np.sin(t)
    plt.plot(X, Y, color=color)
    if (fillColor is not None):
        plt.fill(X, Y, color=fillColor)







