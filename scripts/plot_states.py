import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from mpl_toolkits.mplot3d import Axes3D
from IPython.core.debugger import set_trace
from importlib import reload

import animation
reload(animation)
from animation import Animation

import matplotlib.transforms as mtransforms


stateType = np.dtype([('t',np.float64),
    ('p',np.float64,(3,1)),
    ('v',np.float64,(3,1)),
    ('lin_accel',np.float64,(3,1)),
    ('q',np.float64,(4,1)),
    ('omega',np.float64,(3,1)),
    ('ang_accel',np.float64,(3,1))])

eulerType = np.dtype([('t',np.float64),
    ('ang',np.float64,(3,1))])

commandType = np.dtype([('t',np.float64),
    ('Al',np.float64),
    ('El',np.float64),
    ('Th',np.float64),
    ('Ru',np.float64)])

quadCommandType = np.dtype([('t',np.float64),
    ('Th',np.float64),
    ('TauX',np.float64),
    ('TauY',np.float64),
    ('TauZ',np.float64)])

quadKalmanType = np.dtype([('t',np.float64),
    ('x',np.float64,(8,1)),
    ('Cov',np.float64,(8,8))])

numVehicles = 3
quadStates = []
quadCommandedStates = []
quadEulers = []

for i in range(numVehicles):

    quadStates.append(np.fromfile('/tmp/quad'+str(i)+'_true_state.log', dtype=stateType))
    quadCommandedStates.append(np.fromfile('/tmp/quad'+str(i)+'_commanded_state.log', dtype=stateType))
    quadEulers.append(np.fromfile('/tmp/quad'+str(i)+'_euler_angles.log', dtype=eulerType))

if False:
    quad1Kalman = np.fromfile('/tmp/quad1_0_kalman_state.log', dtype=quadKalmanType)
    quad2Kalman = np.fromfile('/tmp/quad2_0_kalman_state.log', dtype=quadKalmanType)
    quad3Kalman = np.fromfile('/tmp/quad3_0_kalman_state.log', dtype=quadKalmanType)


    plt.rcParams.update({'font.size': 15})

    fig = plt.figure()
    plt.gcf().subplots_adjust(left=0.15)
    # fig.subplots_adjust(hspace=0.5)
    ax = fig.add_subplot(211)
    plt.plot(quadStates[0]['t'], quadStates[0]['p'][:,0], label='Truth', color='orange')
    plt.plot(quad1Kalman['t'], quad1Kalman['x'][:,0], label='Estimated', linestyle='-')
    # plt.plot(quad2Kalman['t'], quad2Kalman['x'][:,0], label='Estimated 2')
    # plt.plot(quad3Kalman['t'], quad3Kalman['x'][:,0], label='Estimated 3')
    s = np.sqrt(quad1Kalman['Cov'][:,0,0])
    plt.plot(quad1Kalman['t'], quad1Kalman['x'][:,0,0]+2*s, color="green", label="2 $\sigma$", linestyle='-')
    plt.plot(quad1Kalman['t'], quad1Kalman['x'][:,0,0]-2*s, color="green", linestyle='-')
    ax.set_ylabel('X (m)')
    # ax.fill_between(quad1Kalman['t'], 500*np.ones(quad1Kalman['t'].shape[0]), -500*np.ones(quad1Kalman['t'].shape[0]), facecolor='gray', alpha=0.25, label='Collision Range')
    # plt.title('t vs Positions')
    ax.legend()

    ax = fig.add_subplot(212, sharex=ax)
    plt.plot(quadStates[0]['t'], quadStates[0]['p'][:,1], label='Truth', color='orange')
    plt.plot(quad1Kalman['t'], quad1Kalman['x'][:,1], label='Estimated', linestyle='-')
    # plt.plot(quad2Kalman['t'], quad2Kalman['x'][:,1], label='Estimated 2')
    # plt.plot(quad3Kalman['t'], quad3Kalman['x'][:,1], label='Estimated 3')
    s = np.sqrt(quad1Kalman['Cov'][:,1,1])
    plt.plot(quad1Kalman['t'], quad1Kalman['x'][:,1,0]+2*s, color="green", label="2 $\sigma$", linestyle='-')
    plt.plot(quad1Kalman['t'], quad1Kalman['x'][:,1,0]-2*s, color="green", linestyle='-')
    # plt.xlabel('t (sec)')
    ax.set_xlabel('t (sec)')
    ax.set_ylabel('Y (m)')
    # ax.fill_between(quad1Kalman['t'], 500*np.ones(quad1Kalman['t'].shape[0]), -500*np.ones(quad1Kalman['t'].shape[0]), facecolor='gray', alpha=0.25, label='Collision Range')
    # ax.legend()


    fig = plt.figure()
    plt.gcf().subplots_adjust(left=0.15)
    ax = fig.add_subplot(211)
    plt.plot(quadStates[0]['t'], quadStates[0]['v'][:,0], label='Truth', color='orange')
    plt.plot(quad1Kalman['t'], quad1Kalman['x'][:,2], label='Estimated', linestyle='-')
    # plt.plot(quad2Kalman['t'], quad2Kalman['x'][:,2], label='Estimated 2')
    # plt.plot(quad3Kalman['t'], quad3Kalman['x'][:,2], label='Estimated 3')
    s = np.sqrt(quad1Kalman['Cov'][:,2,2])
    plt.plot(quad1Kalman['t'], quad1Kalman['x'][:,2,0]+2*s, color="green", label="2 $\sigma$", linestyle='-')
    plt.plot(quad1Kalman['t'], quad1Kalman['x'][:,2,0]-2*s, color="green", linestyle='-')
    ax.set_ylabel('X (m/s)')
    # ax.fill_between(quad1Kalman['t'], 50*np.ones(quad1Kalman['t'].shape[0]), -50*np.ones(quad1Kalman['t'].shape[0]), facecolor='gray', alpha=0.25, label='Collision Range')
    # plt.title('t vs Velocities')
    ax.legend()

    ax = fig.add_subplot(212, sharex=ax)
    plt.plot(quadStates[0]['t'], quadStates[0]['v'][:,1], label='Truth', color='orange')
    plt.plot(quad1Kalman['t'], quad1Kalman['x'][:,3], label='Estimated', linestyle='-')
    # plt.plot(quad2Kalman['t'], quad2Kalman['x'][:,3], label='Estimated 2')
    # plt.plot(quad3Kalman['t'], quad3Kalman['x'][:,3], label='Estimated 3')
    s = np.sqrt(quad1Kalman['Cov'][:,3,3])
    plt.plot(quad1Kalman['t'], quad1Kalman['x'][:,3,0]+2*s, color="green", label="2 $\sigma$", linestyle='-')
    plt.plot(quad1Kalman['t'], quad1Kalman['x'][:,3,0]-2*s, color="green", linestyle='-')
    # plt.xlabel('t (sec)')
    ax.set_xlabel('t (sec)')
    ax.set_ylabel('Y (m/s)')
    # ax.fill_between(quad1Kalman['t'], 50*np.ones(quad1Kalman['t'].shape[0]), -50*np.ones(quad1Kalman['t'].shape[0]), facecolor='gray', alpha=0.25, label='Collision Range')
    # ax.legend()

    # pdb.set_trace()
    plt.show()

###------------------------------------------------###
elif True:
    # numVehicles = 1
    # quadStates.append(np.fromfile('/tmp/wing1'+'_true_state.log', dtype=stateType))
    # quadCommandedStates.append(np.fromfile('/tmp/wing1'+'_commanded_state.log', dtype=stateType))
    # quadEulers.append(np.fromfile('/tmp/wing1'+'_euler_angles.log', dtype=eulerType))
    #
    # print(len(quadStates[0]))

    # quad1Kalman = np.fromfile('/tmp/quad1_0_kalman_state.log', dtype=quadKalmanType)
    print(numVehicles)
    animation = Animation(numVehicles)


    # for i in range(len(quadStates[0])):
    #     print(quadStates[0]['t'][i])
    #     if quadStates[0]['t'][i] > 9:
    #         animation.drawAll(quadStates, quadEulers, quadCommandedStates, i)
    #         plt.waitforbuttonpress()


    jump = 1000
    plt.waitforbuttonpress()
    # set_trace()
    for i in range(len(quadStates[0])//jump):
        # print(quadStates[0]['t'][i*jump])
        animation.drawAll(quadStates, quadEulers, quadCommandedStates, i*jump)
        # set_trace()
        # state = quad1Kalman['x'][i*jump,:][0:2]
        # animation.drawSphere(np.squeeze(np.vstack((state,0))), color='red')

        # plt.waitforbuttonpress()
        plt.pause(0.001)

    print('Press key to close')
    plt.waitforbuttonpress()

    plt.close()

###------------------------------------------------###
elif True:

    # numVehicles = 1
    # quadStates.append(np.fromfile('/tmp/wing1'+'_true_state.log', dtype=stateType))
    # quadCommandedStates.append(np.fromfile('/tmp/wing1'+'_commanded_state.log', dtype=stateType))
    # quadEulers.append(np.fromfile('/tmp/wing1'+'_euler_angles.log', dtype=eulerType))
    #
    # print(len(quadStates[0]))
    plt.rcParams.update({'font.size': 15})
    fig = plt.figure()
    plt.gcf().subplots_adjust(left=0.15)
    ax = fig.add_subplot(111)

    # set_trace()
    stop = quadStates[i]['p'][:,0].shape[0]
    # plt.waitforbuttonpress()
    for i in range(numVehicles):
        # ax.scatter(quadStates[i]['p'][:,0][0:stop:100],quadStates[i]['p'][:,1][0:stop:100],quadStates[i]['p'][:,2][0:stop:100], s=5)
            ax.scatter(quadStates[i]['p'][:,0][0:stop:100],quadStates[i]['p'][:,1][0:stop:100], s=5)

    # ax.view_init(90, 90)
    circle1 = plt.Circle((0, 0), 100, color='grey', fill=False, linewidth=5)
    circle2 = plt.Circle((0, 0), 105, color='white', fill=False, linewidth=10)
    # ax.add_artist(circle1)
    # ax.add_artist(circle2)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.invert_yaxis()
    # ax.set_ylabel('Y (m)')
    plt.show()

elif True:

    ## Help plotting Wing

    wingStates = np.fromfile('/tmp/wing1'+'_true_state.log', dtype=stateType)
    wingCommands = np.fromfile('/tmp/wing1'+'_command.log', dtype=commandType)
    # quadCommandedStates.append(np.from
