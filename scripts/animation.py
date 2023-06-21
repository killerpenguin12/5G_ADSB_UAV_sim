import sys
sys.path.append("/home/jonathan/Desktop/uav_sim/scripts")
import os

cwd = os.getcwd()  # Get the current working directory (cwd)
files = os.listdir(cwd)  # Get all the files in that directory
# print("Files in %r: %s" % (cwd, files))
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
from numpy.linalg import norm
import mpl_toolkits.mplot3d.axes3d as p3
from mpl_toolkits import mplot3d
import pdb
from stl import mesh


class Animation:
    '''
        Create pendulum animation
    '''
    def __init__(self, numVehicles):
        self.flagInit = True
        # self.fig, self.ax = plt.subplots()
        self.handle = []

        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        # self.ax = self.fig.add_subplot(111)
        # self.ax2 = self.fig.add_subplot(122)
        # self.fig.subplots_adjust(bottom=0.25)

        self.set_up_ax(self.ax)
        # self.set_up_ax(self.ax2, title='View Ellipse')

        self.plotObjects = []
        self.flagInit = True

        self.meshes = []
        for i in range(numVehicles):
            self.meshes.append(mesh.Mesh.from_file("models/quadrotor_2.stl"))
            self.meshes[i].points = self.meshes[i].points * 15
        # plt.axis([-2*V.length,2*V.length, -0.1, 2*V.length]) # Change the x,y axis limits
        # plt.plot([0,10],[0,0],'b--')    # Draw a base line
        # plt.plot([0,0],[0,10],'b--')    # Draw a base line
        # plt.xlabel('z(m)')
        # plt.ylabel('h(m)')

        self.image_num = 0

    def set_up_ax(self, ax, title='UVO'):
        maxAxis = 100

        ax.set_xlim3d([-maxAxis, maxAxis])
        # ax.set_xlim3d([-50, -95])
        ax.set_xlabel('X (m)')

        ax.set_ylim3d([-maxAxis, maxAxis])
        # ax.set_ylim3d([35, 65])
        ax.set_ylabel('Y (m)')

        ax.set_zlim3d([-maxAxis, maxAxis])
        # ax.set_zlim3d([-10, 10])
        ax.set_zlabel('Z (m)')

        ax.set_title(title)

        ax.invert_zaxis()
        ax.invert_yaxis()

        ax.view_init(90, 90)

    def drawAll(self, states, eulers, commandedStates, idx):
        # pdb.set_trace()
        for plot in self.plotObjects:
            plot.remove()
        self.plotObjects = []

        self.ax.clear()
        self.set_up_ax(self.ax)
        for i in range(len(self.meshes)):
            self.drawVehicle(self.meshes[i], states[i][idx], eulers[i][idx])
            # self.plotObjects.append(self.drawSphere(states[i][idx]['p']))
            # self.plotObjects.append(self.drawVelocity(states[i][idx], eulers[i][idx]))
            # self.plotObjects.append(self.drawCommandedVelocity(states[i][idx], eulers[i][idx], commandedStates[i][idx]))
        # self.fig.canvas.flush_events()


        ''' Save Images '''
        # fname = 'images/_tmp%03d.png'%self.image_num
        # self.fig.savefig(fname, dpi=300)
        # self.image_num += 1

        if self.flagInit:
            self.flagInit = False

    def drawVehicle(self, mesh, state, euler, color="limegreen"):
        # pdb.set_trace()
        mesh.rotate([0.5, 0, 0], np.pi)
        mesh.rotate([0.5, 0, 0], -euler['ang'][0])
        mesh.rotate([0, 0.5, 0], -euler['ang'][1])
        mesh.rotate([0, 0, 0.5], -euler['ang'][2])
        mesh.translate(state['p'])
        collection = mplot3d.art3d.Poly3DCollection(mesh.vectors)

        # xy = state['p']
        # self.handle.append(collection)
        self.ax.add_collection3d(collection)
        mesh.translate(-state['p'])
        mesh.rotate([0, 0, 0.5], euler['ang'][2])
        mesh.rotate([0, 0.5, 0], euler['ang'][1])
        mesh.rotate([0.5, 0, 0], euler['ang'][0])
        mesh.rotate([0.5, 0, 0], -np.pi)

        # if self.flagInit == True:
        #     self.handle.append(collection)
        #     self.ax.add_collection3d(self.handle[idx])
        #     # self.handle.append(mpatches.CirclePolygon(xy,
        #     #     radius = radius, resolution = 15,
        #     #     fc = color, ec = 'black'))
        #     # self.ax.add_patch(self.handle[idx])
        # else:
        #     # self.handle[idx]._xy=xy
        #     print(xy)
        #     self.handle[idx].set_3d_properties()

    def drawSphere(self, state, color='blue'):
        # Sphere
        x_pos, y_pos, z_pos = state
        radius = 5.0
        # print(x_pos,radius)
        u = np.linspace(0, 2 * np.pi, 50)
        v = np.linspace(0, np.pi, 50)

        x = radius * np.outer(np.cos(u), np.sin(v)) + x_pos
        y = radius * np.outer(np.sin(u), np.sin(v)) + y_pos
        z = radius * np.outer(np.ones(np.size(u)), np.cos(v)) + z_pos
        return self.ax.plot_surface(x, y, z, rstride=4, cstride=4, color=color, linewidth=0, alpha=0.5)


    def rotation(self, theta):
        c, s = np.cos(theta), np.sin(theta)
        R = np.matrix([[c, s, 0], [-s, c, 0], [0, 0, 1]])
        return R

    def drawVelocity(self, state, euler, color="red"):
        start = state['p']
        startRadius = 0.5
        end = start + state['v']
        endRadius = 0.0001
        if all(end == start):
            end = end + 0.1

        X,Y,Z = self.truncated_cone(start, end, startRadius, endRadius)
        return self.ax.plot_wireframe(X, Y, Z,
            color=color,
            linewidth=0.0001,
            antialiased=False)
        # X = [start[0,0],end[0,0]]
        # Y = [start[1,0],end[1,0]]
        # Z = [start[2,0],end[2,0]]
        #
        # return self.ax.plot(X,Y,Z,color='red')

    def drawCommandedVelocity(self, state, euler, commandedState, color="green"):
        start = state['p']
        startRadius = 0.5
        # end = self.rotation(euler['ang'][0,0]) * (start + commandedState['v'])
        end = start + commandedState['v']
        endRadius = 0.0001
        if all(end == start):
            end = end + 0.1

        X,Y,Z = self.truncated_cone(start, end, startRadius, endRadius)
        return self.ax.plot_wireframe(X, Y, Z,
            color=color,
            linewidth=0.0001,
            antialiased=False)
        # X = [start[0,0],end[0,0]]
        # Y = [start[1,0],end[1,0]]
        # Z = [start[2,0],end[2,0]]
        #
        # return self.ax.plot(X,Y,Z,color='red')


    def truncated_cone(self, p0, p1, R0, R1):
        """
        Based on https://stackoverflow.com/a/39823124/190597 (astrokeat)
        """
        if (p1 == p0).all():
            p1 = np.array([p1[0]+0.1, p1[1]+0.1, p1[2]+0.1])

        v = p1 - p0
        mag = norm(v)
        v = v / mag
        not_v = np.array([1, 1, 0])
        v = np.squeeze(np.asarray(v))

        if (v == not_v).all():
            not_v = np.array([0, 1, 0])

        n1 = np.cross(v, not_v)
        n1 /= norm(n1)
        n2 = np.cross(v, n1)

        n = 80
        t = np.linspace(0, mag, n)
        theta = np.linspace(0, 2 * np.pi, n)

        t, theta = np.meshgrid(t, theta)
        R = np.linspace(R0, R1, n)
        X, Y, Z = [p0[i] + v[i] * t + R *
                   np.sin(theta) * n1[i] + R * np.cos(theta) * n2[i] for i in [0, 1, 2]]

        return X,Y,Z


# Used see the animation from the command line
if __name__ == "__main__":

    simAnimation = ballbeamAnimation()    # Create Animate object
    z = 0.0                               # Position of cart, m
    theta = 0.0*np.pi/180                 # Angle of pendulum, rads
    simAnimation.drawVtol([z, theta, 0, 0])  # Draw the pendulum

    print('Press key to close')
    plt.waitforbuttonpress()
    plt.close()
