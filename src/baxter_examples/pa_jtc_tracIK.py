#!/usr/bin/env python

# Copyright (c) 2013-2015, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Baxter RSDK Joint Trajectory Action Client Example
"""
#------------------------------------ Imports ------------------------------
import pdb

import argparse
import sys

import tf
import rospy
import actionlib

from copy import copy

# Action modules: gripper and trajectory
from control_msgs.msg import (
    GripperCommandAction,
    GripperCommandGoal,
)

# Open and Close Grippers 
from baxter_examples import pa_closeHand
from baxter_examples import pa_openHand

# Moving Arms to Home position
from birl_recorded_motions import GoHome as gh

# Pose Stamped and Transformation
from geometry_msgs.msg import PoseStamped

from rbx1_nav.transform_utils import quat_to_angle

# Kinematics
from baxter_pykdl import baxter_kinematics
import PyKDL
from math import pi

# Baxter Messages
from baxter_core_msgs.msg import (
    EndpointState,
    JointCommand,
)

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
) 

# Open and close hand
from baxter_examples import pa_openHand
from baxter_examples import pa_closeHand

# Flags
malePickup=0
assembly  =1

# Baxter Stuff
import baxter_interface
from baxter_interface import CHECK_VERSION

#------------------------------------ Design Parameters ------------------------------
reference_pose_flag = 1
approach_flag=2
_joints = JointCommand()
#------------------------------------ Class ___________ ------------------------------
class Trajectory(object):
    def __init__(self, limb):
        ns = 'robot/limb/' + limb + '/'
        self._client = actionlib.SimpleActionClient( # create simple client w/ topic and msg type
            ns + "follow_joint_trajectory",
            FollowJointTrajectoryAction,
        )
        self._goal = FollowJointTrajectoryGoal() 		# trajectory(header/points);
								# path_tolerance
                                                                # goal_tolerance 
                                                                # goal_time_tolerance
        self._goal_time_tolerance = rospy.Time(0.1) 		# Reach goal within this tolerance of final time.
        self._goal.goal_time_tolerance = self._goal_time_tolerance # Assign to goal object. 
        server_up = self._client.wait_for_server(timeout=rospy.Duration(10.0)) # Connect to server within this time. 
        if not server_up:
            rospy.logerr("Timed out waiting for Joint Trajectory"
                         " Action Server to connect. Start the action server"
                         " before running example.")
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)
        self.clear(limb)

    def add_point(self, positions, time):
        point = JointTrajectoryPoint()
        point.positions = copy(positions)
        point.time_from_start = rospy.Duration(time)
        self._goal.trajectory.points.append(point)

    def start(self):
        self._goal.trajectory.header.stamp = rospy.Time.now()
        self._client.send_goal(self._goal)

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=15.0):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))

    def result(self):
        return self._client.get_result()

    def clear(self, limb):
        self._goal = FollowJointTrajectoryGoal()
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        self._goal.trajectory.joint_names = [limb + '_' + joint for joint in \
            ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']]

#------------------------------------ Global Methods ------------------------------
def callback(joints):
    joints=JointCommand()
    _joints.mode=joints.mode
    _joints.names=copy(joints.names)
    _joints.command=copy(joints.command)
    

def calcInvKin(posePub,currPose,limb):
    """ This function converts an EndpointState to a PoseStamped and publishes the data, which is used by the track_IK node to produce a 7 dof joint angle solution of type JointState's."""

    # Convert EndpointState to PoseStamped
    tempPose = PoseStamped()
    tempPose.header.stamp=rospy.Time.now()
    tempPose.header.frame_id=limb

    # Copy the position from a dictionary to a xyz
    tempPose.pose.position.x=currPose['position'][0]
    tempPose.pose.position.y=currPose['position'][1]
    tempPose.pose.position.z=currPose['position'][2]

    # Copy the orientation from the dictionary to xyzw
    tempPose.pose.orientation.x=currPose['orientation'][0]
    tempPose.pose.orientation.y=currPose['orientation'][1]
    tempPose.pose.orientation.z=currPose['orientation'][2]
    tempPose.pose.orientation.w=currPose['orientation'][3]

    # Publish
    posePub.publish(tempPose)
    posePub.publish(tempPose)

    # Get a single message of type baxter_core_msgs/JointCommand
    _joints = rospy.wait_for_message("joints", JointCommand)#timeout, default=none

    # return JointCommand
    return _joints

def main():
    """RSDK Joint Trajectory Example: Simple Action Client

    Creates a client of the Joint Trajectory Action Server
    to send commands of standard action type,
    control_msgs/FollowJointTrajectoryAction.

    Make sure to start the joint_trajectory_action_server.py
    first. Then run this example on a specified limb to
    command a short series of trajectory points for the arm
    to follow.
    """
    global reference_pose_flag # Will use this global variable
    global _joints
    arg_fmt = argparse.RawDescriptionHelpFormatter # create ArgumentParser object
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    required = parser.add_argument_group('required arguments') # set required strings
    required.add_argument(
        '-l', '--limb', required=True, choices=['left', 'right'],
        help='send joint trajectory to which limb'
    )
    args = parser.parse_args(rospy.myargv()[1:]) # return objects
    
    # Set limb and gripper side
    limb = args.limb

    #gripper = args.limb

    print("Initializing node... ")
    rospy.init_node("pa_jtc_tracIK")
    posePub=rospy.Publisher("pose",PoseStamped,queue_size=2)
    rospy.Subscriber("joints",JointCommand,callback)

    # Create Kinematic Objects
    kin = baxter_kinematics(limb)

    # Get robot State
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()
    print("Running. Ctrl-c to quit")
    
    # Create Joint Names List
    jNamesl=['right_s0','right_s1','right_e0','right_e1','right_w0','right_w1','right_w2']
      
    # Get Current Joint Positions first and command them
    arm = baxter_interface.limb.Limb(limb)    
    current_angles = [arm.joint_angle(joint) for joint in arm.joint_names()]

    # Create open and close gripper object
    pc=pa_closeHand.GripperClient(limb)
    po=pa_openHand.GripperClient(limb)

    ################################# State Machine############################
    #--------------- Get Starting and Reference positions -----------------------
    rospy.loginfo('Starting pivot approach demo....')

    # Go to the Home Position
    rospy.loginfo('I will first move to the home position, as a standard starting point. There you will be able to let the robot grip the male part.')
    gh.GoHome()


    rospy.loginfo('I will now open the gripper so that you can place the male camera mold....')
    po.open()

    key=raw_input('When you are ready to close, press a key: \n')
    pc.close()

    # Capture Starting Position
    rospy.loginfo('Lets capture the starting pose  of the robot') 
    startPose=arm.endpoint_pose()
    startJoints=kin.inverse_kinematics(startPose['position'],startPose['orientation']).tolist()
    startJoints_=dict(zip(jNamesl,startJoints))

    # Record reference position

    if not reference_pose_flag:
        rospy.loginfo('Now please move to the reference location.\n Open a new terminal and use keyboard teleoperation: roslaunch baxter_end_effector_control end_effector_controarm.launch keyboard:=true')
        key=raw_input('When you have finished, pres any key. Tthen I will record this as the reference location, after that the assembly should run automatically: \n')
        referencePose=arm.endpoint_pose()
        reference_pose_flag=1

    else:
        x=    0.55686099076
        y=   -0.296882237511
        z=    -0.177124326921
        qx=-0.00928787935556
        qy= 0.989519808056
        qz=-0.00726430566722
        qw=-0.143914956223
        ref_p=baxter_interface.limb.Limb.Point(x,y,z)
        ref_q=baxter_interface.limb.Limb.Quaternion(qx,qy,qz,qw)
        referencePose={'position': ref_p, 'orientation': ref_q}

    referenceJoints=calcInvKin(posePub,referencePose,limb)    
    #referenceJoints=kin.inverse_kinematics(referencePose['position'],referencePose['orientation']).tolist()
    referenceJoints_=dict( zip( list(referenceJoints.names),list(referenceJoints.command) ) )
    
    #------------------ Compute Goal Position ---------------------------------------
    # Convert ortientation to RPY using PyKDL
    qref=referencePose['orientation']
    rot_mat=PyKDL.Rotation.Quaternion(qref.x,qref.y,qref.z,qref.w)
    #rot=rot_mat.GetRPY()
    # The idea is to set the roll to -pi and the pitch to 0. Keep the yaw
    #rot[0]=-pi, rot[1]=0
    rot_mat[0,0]=-1; rot_mat[1,0]=0; rot_mat[2,0]=0
    rot_mat[0,1]=0;  rot_mat[1,1]=1; rot_mat[2,1]=0
    q_goal=rot_mat.GetQuaternion()
    
    # Create the dictionary
    goalPose=referencePose
    # Change the orientation
    goalPose['orientation']=q_goal

    goalJoints=calcInvKin(posePub,goalPose,limb)    
    #goalJoints=kin.inverse_kinematics(goalPose['position'],goalPose['orientation']).tolist()
    goalJoints_=dict( zip( list(goalJoints.names),list(goalJoints.command) ) )

    #----------------------------------- Approach # 1
    # Add 5 cm to referene pose
    approach1Pose=referencePose
    _x=referencePose['position'][0];
    _y=referencePose['position'][1];
    _z=referencePose['position'][2]+0.05
    del_z=baxter_interface.limb.Limb.Point(_x,_y,_z)
    
    approach1Pose['position']=del_z
    approach1Joints=calcInvKin(posePub,approach1Pose,limb)    
    #approach1Joints=kin.inverse_kinematics(approach1Pose['position'],approach1Pose['orientation']).tolist()
    approach1Joints_=dict( zip( list(approach1Joints.names),list(approach1Joints.command) ) )

    #----------------------------------- Approach # 2
    # Add 1 cm to referene pose
    approach2Pose=referencePose

    _z=referencePose['position'][2]+0.01
    del_z=baxter_interface.limb.Limb.Point(_x,_y,_z)
    
    approach2Pose['position']=del_z
    approach2Joints=calcInvKin(posePub,approach2Pose,limb)    
    #approach2Joints=kin.inverse_kinematics(approach2Pose['position'],approach2Pose['orientation']).tolist()
    approach2Joints_=dict( zip( list(approach2Joints.names),list(approach2Joints.command) ) )


    ############################## Moving to Points ####################################
    #1. Approach 0
    rospy.loginfo('001 Moving to starting position 1/7.')
    arm.move_to_joint_positions(startJoints_)
    rospy.sleep(1)

    #2. Appraoch 1
    rospy.loginfo('002 Moving to approach 1 2/7.')
    arm.move_to_joint_positions(approach1Joints_)
    rospy.sleep(1)

    #3. Appraoch 2
    rospy.loginfo('003 Moving to approach 2 3/7.')
    arm.move_to_joint_positions(approach2Joints_)
    rospy.sleep(1)

    #4. Reference 
    rospy.loginfo('004 Moving to reference position 4/7.')
    arm.move_to_joint_positions(referenceJoints_)
    rospy.sleep(3)

    #5. Goal
    rospy.loginfo('005 Moving to goal position 5/7.')
    # Can move to the goal with two approaches: move_to_joint_positions or joint_trajectory_action_server
    if approach_flag == 1:
        # (i) move_to_joint_positions
        arm.move_to_joint_positions()

        #6. Open the Gripper
        rospy.loginfo('006 Opening the gripper 6/7.')
        po.open()
        rospy.sleep(1)

        # Move Back
        rospy.loginfo('007 Moving back to start position 7/7.')
        arm.move_to_joint_positions(startJoints_)
        rospy.sleep(1)
        print("Exiting: Planner")
    
    # (ii) joint_trajectory_action_server
    else:
        pdb.set_trace()
        duration=2.0					# Set duration of motion
        traj=Trajectory(limb) 				# Create a trajectory class object
        traj.add_point( list(referenceJoints.command), 0.0)
        traj.add_point( list(goalJoints.command),duration )	# Add list of joint angles
        traj.start()					# Call the server
        traj.wait(duration+1.0)					# If motion does not complete before timeout, throw exception.
        rospy.sleep(3)

        #6. Open the Gripper
        rospy.loginfo('006 Opening the gripper 6/7.')
        po.open()
        rospy.sleep(1)

        # Move Back
        rospy.loginfo('007 Moving back to start position 7/7.')
        traj.stop()
        traj.add_point( list(goalJoints.command),0.0 )	# Add list of joint angles        
        traj.add_point( startJoints, duration)
        traj.start()
        traj.wait(duration+1.0)
        print("Exiting: Planner")



if __name__ == "__main__":
    try:
        main()
    except:
        rospy.loginfo('pa_jtc_tracIK terminated.')
