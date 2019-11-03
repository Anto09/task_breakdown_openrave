import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--interactive", action="store_true")
args = parser.parse_args()

import openravepy
import trajoptpy
import json
import time
import numpy as np
import trajoptpy.kin_utils as ku
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

if __name__ == "__main__":

    #env = Environment()
    #env.SetViewer('qtcoin')
    #env.Reset()        
    #env.Load('empty.env.xml')

    #robot = env.GetRobots()[0]
    
    env = Environment()
    env.SetViewer('qtcoin')
    env.Load("kinbodies_robots_envs/empty.env.xml")
    robot = env.GetRobots()[0]
    #env.StopSimulation()

    tuckarms(env, robot)

    print robot
    print robot.GetManipulators()

    print '!!!!!!!!!!!!!!!!!!!!!!!!!!!'
    print type(args.interactive)
    trajoptpy.SetInteractive(args.interactive) # pause every iteration, until you press 'p'. Press escape to disable further plotting
    time.sleep(0.1)


    joint_start = [0.08, -1.0,  1.19, 1.94, -0.67, 1.03,  0.50]

    robot.SetDOFValues(joint_start, robot.GetManipulator('rightarm').GetArmIndices())

    joint_target = [0.0, 0.0, 0, 0, 0, 0, 0]

    #xyz_target 
    start xyz = [-0.075 -0.825  0.725]
    start rot = [0.70710678118654746, 0.0, 0.70710678118654746, 0.0]

    end xyz = [-0.17322091 -0.825       0.18611469]
    end rot = [0.70710678118654746, 0.0, 0.70710678118654746, 0.0] <type 'list'>

    # manip = robot.GetManipulator("rightarm")
    # init_joint_target = ku.ik_for_link(hmat_target, manip, "r_wrist", filter_options = openravepy.IkFilterOptions.CheckEnvCollisions)
    request = {
      "basic_info" : {
        "n_steps" : 100,
        "manip" : "rightarm", # see below for valid values
        "start_fixed" : True # i.e., DOF values at first timestep are fixed based on current robot state
      },
      "costs" : [
      {
        "type" : "joint_vel", # joint-space velocity cost
        "params": {"coeffs" : [1]} #change this
        # a list of length one is automatically expanded to a list of length n_dofs
        # also valid: [1.9, 2, 3, 4, 5, 5, 4, 3, 2, 1]
      },
      {
        "type" : "collision",
        "name" :"cont_coll",
        "params" : {
          "continuous" : True,
          "coeffs" : [1000], # penalty coefficients. list of length one is automatically expanded to a list of length n_timesteps
          "dist_pen" : [0.025] # robot-obstacle distance that penalty kicks in. expands to length n_timesteps
        },    
      }
      ],
      "constraints" : [
      {
        "type" : "joint", # joint-space target
        "params" : {"vals" : joint_target } # length of vals = # dofs of manip
        # "type" : "pose", 
        # "params" : {"xyz" : xyz_target, 
        #             "wxyz" : quat_target, 
        #             "link": "r_gripper_tool_frame",
        #             "timestep" : 9
        #             }
                 
      }
      ],
      "init_info" : {
          "type" : "straight_line", # straight line in joint space.
          "endpoint" : joint_target
      }
    }
    s = json.dumps(request) # convert dictionary into json-formatted string
    prob = trajoptpy.ConstructProblem(s, env) # create object that stores optimization problem
    t_start = time.time()

    result = trajoptpy.OptimizeProblem(prob) # do optimization
    t_elapsed = time.time() - t_start
    print result
    print "optimization took %.3f seconds"%t_elapsed

    raw_input("Press enter to continue...")

    from trajoptpy.check_traj import traj_is_safe
    prob.SetRobotActiveDOFs() # set robot DOFs to DOFs in optimization problem
    #assert traj_is_safe(result.GetTraj(), robot) # Check that trajectory is collision free
    print "Is traj safe:",traj_is_safe(result.GetTraj(), robot)

    with env:
      traj = RaveCreateTrajectory(env,'')
      spec = IkParameterization.GetConfigurationSpecificationFromType(IkParameterizationType.Transform6D,'linear')

      print robot.GetActiveConfigurationSpecification()

      traj.Init(robot.GetActiveConfigurationSpecification());

      for traj_pts in result.GetTraj():
        print traj_pts
        traj.Insert(traj.GetNumWaypoints(), traj_pts)

      planningutils.RetimeActiveDOFTrajectory(traj,robot,hastimestamps=False,maxvelmult=0.01,maxaccelmult=0.01,plannername='LinearTrajectoryRetimer')
    
      robot.GetController().SetPath(traj)
    robot.WaitForController(0)

    raw_input("Press enter to exit...")
