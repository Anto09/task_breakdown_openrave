#!/usr/bin/env python
# -*- coding: utf-8 -*-
#HW3 for RBE 595/CS 525 Motion Planning
import time
import openravepy

#### YOUR IMPORTS GO HERE ####
import sys
#### END OF YOUR IMPORTS ####

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)

def tuckarms(env,robot):
    with env:
        jointnames = ['left_s0','left_s1','left_e0','left_e1','left_w0','left_w1','left_w2',
                      'right_s0','right_s1','right_e0','right_e1','right_w0','right_w1','right_w2']
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
        robot.SetActiveDOFValues([-0.08, -1.0, -1.19, 1.94,  0.67, 1.03, -0.50,
                                   0.08, -1.0,  1.19, 1.94, -0.67, 1.03,  0.50]);         
        robot.GetController().SetDesired(robot.GetDOFValues());
    waitrobot(robot)

def checkPoses(env, robot):
    with env:
        jointnames = ['right_s0','right_s1','right_e0','right_e1','right_w0','right_w1','right_w2']
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])


    targets = [[ 0.08, -1, 1.19, 1.94, -0.67, 1.03, 0.5],
               [ 0.07111111, -0.92957163, 1.07963398, 1.6971865, -0.59487401, 0.91233845, 0.44449249],
               [ 0.06222221, -0.83171655, 0.95065146, 1.4750886, -0.51998324, 0.79520219, 0.38896183],
               [ 0.05333332, -0.73385912, 0.82166749, 1.25299221, -0.44509259, 0.67806614, 0.33343115],
               [ 0.04444442, -0.63490283, 0.69211165, 1.03152617, -0.3702606, 0.56107408, 0.27788985],
               [ 0.03555553, -0.52865509, 0.55960938, 0.81400675, -0.29572177, 0.44520978, 0.22230857],
               [ 0.02666663, -0.39649132, 0.41970703, 0.61050506, -0.22179133, 0.33390733, 0.16673143],
               [ 0.01777776, -0.26432755, 0.27980469, 0.40700337, -0.14786089, 0.22260489, 0.11115429],
               [ 0.00888888, -0.13216377, 0.13990234, 0.20350169, -0.07393044, 0.11130244, 0.05557714],
               [ 0, 0, 0, 0, 0, 0, 0]]

    targets2 =  [[0.08, -1, 1.19,  1.94, -0.67,  1.03,  0.5],
                 [0.07111111, -0.88888889, 1.05777778, 1.72444444, -0.59555556, 0.91555556, 0.44444444],
                 [0.06222222, -0.77777778, 0.92555556, 1.50888889, -0.52111111, 0.80111111, 0.38888889],
                 [0.05333333, -0.66666667, 0.79333333, 1.29333333, -0.44666667, 0.68666667, 0.33333333],
                 [0.04444444, -0.55555556, 0.66111111, 1.07777778, -0.37222222, 0.57222222, 0.27777778],
                 [0.03555556, -0.44444444, 0.52888889, 0.86222222, -0.29777778, 0.45777778, 0.22222222],
                 [0.02666667, -0.33333333, 0.39666667, 0.64666667, -0.22333333, 0.34333333, 0.16666667],
                 [0.01777778, -0.22222222, 0.26444444, 0.43111112, -0.14888889, 0.22888889, 0.11111111],
                 [0.00888889, -0.11111111, 0.13222222, 0.21555587, -0.07444444, 0.11444444, 0.05555556],
                 [0, 0, 0, 0, 0, 0, 0]]

    targets3 = [[ 0.08, -1,    1.19,  1.94, -0.67,  1.03,  0.5 ],
[ 0.17225388, -0.99998926,  1.2214498,   1.79763884, -0.55374121,  0.93665151,
  0.47368421],
[ 0.23908771, -0.97453809,  1.23103586,  1.64132948, -0.43969934,  0.84330333,
  0.44736842],
[ 0.30592155, -0.94908693,  1.24062193,  1.37223734, -0.32565746,  0.74996164,
  0.42105263],
[-0.03701482, -0.58833367,  0.86360219,  1.27013207, -0.31213378,  0.64682369,
  0.39473684],
[-0.03364224, -0.54910461,  0.80607932,  1.18546394, -0.29143325,  0.60380712,
  0.36842105],
[-0.03131069, -0.50981219,  0.74843743,  1.10085947, -0.27055815,  0.5606192,
  0.34210526],
[-0.02892607, -0.47057189,  0.69084355,  1.01620187, -0.24972579,  0.5174745,
  0.31578947],
[-0.02652943, -0.43134366,  0.63326051,  0.93153225, -0.22890312,  0.47433942,
  0.28947368],
[-0.02412721, -0.39212102,  0.57568255,  0.84685705, -0.20808567,  0.43120955,
  0.26315789],
[-0.02172075, -0.35290263,  0.51810848,  0.76217761, -0.18727176,  0.38808324,
  0.23684211],
[-0.01931134, -0.31368721,  0.46053714,  0.67749521, -0.16646036,  0.34495944,
  0.21052632],
[-0.01689989, -0.27447383,  0.4029677,   0.59281078, -0.14565068,  0.30183737,
  0.18421053],
[-0.01448709, -0.23526181,  0.3453995,   0.508125,   -0.12484216,  0.25871647,
  0.15789474],
[-0.01207341 -0.19605067,  0.28783213,  0.42343833, -0.1040344,  0.21559632,
  0.13157895],
[-0.00965918, -0.15684009,  0.23026528,  0.33875111, -0.08322712,  0.17247665,
  0.10526316],
[-0.0072446, -0.11762985,  0.17269875,  0.25406355, -0.06242014,  0.12935729,
  0.07894737],
[-0.00482982, -0.07841981,  0.11513241,  0.16937579, -0.04161335,  0.08623811,
  0.05263158],
[-0.00241494, -0.03920988,  0.05756618,  0.08468792, -0.02080665,  0.04311903,
  0.02631579],
[0,  0,  0,  0, 0, 0, 0]]


    for i in range (0, len(targets3)):     
        arr = targets3[i]
        print arr
        with env:
            robot.SetActiveDOFValues(arr)
            robot.GetController().SetDesired(robot.GetDOFValues());
        waitrobot(robot)
        raw_input("Press enter to continue...")


if __name__ == "__main__":

    env = Environment()
    # RaveInitialize()
    # module = RaveCreateModule(env, 'urdf')
    #RaveLoadPlugin('myplugin/build/myplugin')
    env.SetViewer('qtcoin')
    env.Reset()        
    # load a scene from ProjectRoom environment XML file
    env.Load('empty.env.xml')
    #env.Load('empty.env.xml')
    time.sleep(0.1)

    # 1) get the 1st robot that is inside the loaded scene
    # 2) assign it to the variable named 'robot'
    robot = env.GetRobots()[0]
    #print robot.GetJoints()
    #print robot.GetActiveDOF()

    
    ### INITIALIZE YOUR PLUGIN HERE ###

    #!/usr/bin/env python
    print sys.path
    RaveInitialize()
    #RaveLoadPlugin('myplugin/build/myplugin')

    ### END INITIALIZING YOUR PLUGIN ###
   

    # tuck in the PR2's arms for driving
    raw_input("Press enter to tuck...")
    tuckarms(env, robot)
    checkPoses(env, robot)

    #set active joints
    #jointnames =['l_shoulder_pan_joint','l_shoulder_lift_joint','l_elbow_flex_joint','l_upper_arm_roll_joint','l_forearm_roll_joint','l_wrist_flex_joint','l_wrist_roll_joint']
    #robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])        

    # set start config
    #startconfig = [-0.15,0.075,-1.008,0,0,0,0]
    #robot.SetActiveDOFValues(startconfig);        
    #robot.GetController().SetDesired(robot.GetDOFValues());
    #waitrobot(robot)

    #with env:
        #goalconfig = [0.449,-0.201,0,0,0,0,0]

        ### YOUR CODE HERE ###
        ###call your plugin to plan, draw, and execute a path from the current configuration of the left arm to the goalconfig
        
        #robot_weights = [0.491099, 0.08705757,  0.06498835,  0.04085778,  0.03327996,  0.05081965, 0.01457205];
        #robot joint weights generated by
        #'''
        #lmodel = databases.linkstatistics.LinkStatisticsModel(robot)
        #if not lmodel.load():
        #    lmodel.autogenerate()
        #lmodel.setRobotWeights()
        #print _probot.GetActiveJointWeights()
        #'''

        #RRT = RaveCreateModule(env,'RRT')
        #goal_string = str(goalconfig).strip('[').strip(']')
        #weight_string = str(robot_weights).strip('[').strip(']')

        #bidirect = "0"
        #goal_bias = "0.16"

        #RRT.SendCommand("Set_Joint_Weights " + weight_string.replace(',', ''))
        #RRT.SendCommand("Set_Bidirectional " + bidirect)
        #RRT.SendCommand("Set_Goal_Bias "+ goal_bias)
        #RRT.SendCommand("Start_RRT " + goal_string.replace(',', ''))

        ### END OF YOUR CODE ###
    #waitrobot(robot)
    #RaveDestroy()

    raw_input("Press enter to exit...")