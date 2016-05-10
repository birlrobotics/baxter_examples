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
import argparse
import sys

import rospy
import actionlib

import pdb
from copy import copy

# Action modules: gripper and trajectory
from control_msgs.msg import (
    GripperCommandAction,
    GripperCommandGoal,
)

# Open and Close grippers
from baxter_examples import pa_closeHand
from baxter_examples import pa_openHand

# Pose Stamped and Transformation
from geometry_msgs.msg import PoseStamped
from rbx1_nav.transform_utils import quat_to_angle

# Kinematics
from baxter_pykdl import baxter_kinematics
import PyKDL
from math import py

# Baxter Messages
from baxter_core_msgs.msg import EndpointState

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
    rospy.init_node("PivotApproach_trajectory_client_%s" % (limb,))

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
    l = baxter_interface.limb.Limb(limb)    
    current_angles = [l.joint_angle(joint) for joint in l.joint_names()]

    # Create open and close gripper object
    close=pa_closeHand.GripperClient('limb')
    open=pa_openHand.GripperClient('limb')

    ################################# State Machine############################
    #--------------- Get Starting and Reference positions -----------------------
    rospy.loginfo('Starting state machine....')
    rospy.loginfo('Let me capture the current pose of the robot')
    startPose=l.endpoint_pose()
    startJoints=kin.inverse_kinematics(startPose['position'],startPose['orientation']).tolist()
    startPose_=dict(zip(jNamesl,startJoints))

    # Record reference position
    rospy.loginfo('Now please move to the reference location, using keyboard teleoperation')
    key=raw_input('When you have finished, pres any key. Tthen I will record this as the reference location, after that the assembly should run automatically: \n')

    referencePose=l.endpoint_pose()
    referenceJoints=kin.inverse_kinematics(referencePose['position'],referencePose['orientation']).tolist()
    referencePose_=dict(zip(jNamesl,referenceJoints))
    
    # Save to file
    # file_=open('pa_refPos','w')
    # foreach(var i in referencePose.ToArray())
    #     file_.write(referencePose[i.position]=i.Value.Trim())
    # file_.close()

    #------------------ Compute Goal Position ---------------------------------------
    # Convert ortientation to RPY using PyKDL
    q=referencePose['orientation']
    rot_mat=PyKDL.Rotation.Quaternion(q.x,q.y,q.z,q.w)
    #rot=rot_mat.GetRPY()
    # The idea is to set the roll to -pi and the pitch to 0. Keep the yaw
    #rot[0]=-pi, rot[1]=0
    rot_mat[0,0]=-1, rot_mat[1,0]=0,rot_mat[2,0]=0,
    rot_mat[0,1]=0,  rot_mat[1,1]=1,rot_mat[2,1]=0,
    q_goal=rot_mat.GetQuaternion()
    
    # Create the dictionary
    goalPose=referenePose
    # Change the orientation
    goalPose['orientation']=q_goal

    goalJoints=kin.inverse_kinematics(goalPose['position'],goalPose['orientation']).tolist()
    goalPose_=dict(zip(jNamesl,goalJoints))

    #----------------------------------- Approach # 1
    # Add 10 cm to referene pose
    approach1Pose=referencePose
    del_z=referencePose['position'][2]+0.05
    approach1Pose['position'][2]=del_z

    approach1Joints=kin.inverse_kinematics(approach1Pose['position'],approach1Pose['orientation']).tolist()
    approach1Pose_=dict(zip(jNamesl,approach1Joints))


    #----------------------------------- Approach # 2
    # Add 5 cm to referene pose
    approach2Pose=referencePose
    del_z=referencePose['position'][2]+0.01
    approach2Pose['position'][2]=del_z

    # approach2Joints=kin.inverse_kinematics(approach2Pose['position'],approach2Pose['orientation']).tolist()
    # approach2Pose_=dict(zip(jNamesl,approach2Joints))


    # ############################## Moving to Points ####################################
    # #1. Approach 0
    # l.move_to_joint_positions(startPose_)
    # rospy.sleep(8)

    # #2. Appraoch 1
    # l.move_to_joint_positions(approach1Pose_)
    # rospy.sleep(1)

    # #3. Appraoch 2
    # l.move_to_joint_positions(approach2Pose_)
    # rospy.sleep(5)

    # #4. Reference 
    # l.move_to_joint_positions(referencePose_)
    # rospy.sleep(5)

    # #5. Appraoch 4
    # l.move_to_joint_positions(goalPose_)
    # rospy.sleep(5)

    # # Move Back
    # l.move_to_joint_positions(startPose_)
    # rospy.sleep(8)



    # ## Reference: manually input from teleoperation
    # _pose=EndpointState()
    # _pose.pose.position.x=    0.55686099076
    # _pose.pose.position.y=   -0.296882237511
    # _pose.pose.poition.z=    -0.177124326921
    # _pose.pose.orientation.x=-0.00928787935556
    # _pose.pose.orientation.y= 0.989519808056
    # _pose.pose.orientation.z=-0.00726430566722
    _pose.pose.orientation.w=-0.143914956223

    # Also need to comput the joint angle positions to pass the the action server
    iKin=kin.inverse_kinematics()

    
    # 1. Midpoint
    pos = [0.19136410307006838, -0.662679699609375, 0.8352525380493164, 1.6306215756591798, -0.7830971913208008, 0.9376457555236817, 0.43373306727905275]
    ref=dict(zip(jNamesl,pos))
    l.move_to_joint_positions(ref)    
    rospy.sleep(1)

    if malePickup:
        # 2. Closing in to Male Part
        pos = [0.3631699511169434, -0.21015536770019533, 0.6718835843261719, 1.194971032397461, -0.9027476926391602, 0.8467573939453126, 0.3367087825561524]
        ref=dict(zip(jNamesl,pos))
        l.move_to_joint_positions(ref) #set_joint_positions(ref)    
        rospy.sleep(1)
    
    
        # 3. PreGrab
        pos=[0.3861796629089356, -0.09012137118530274, 0.6576942620544434, 1.0952622812988282, -0.9223059476623536, 0.8114758358642579, 0.31178159478149414]
        ref=dict(zip(jNamesl,pos))
        l.move_to_joint_positions(ref) 
        rospy.sleep(1)
    #    
        # 4. Grab Male Part
        pos = [0.3900146148742676, -0.06864564017944337, 0.6507913485168457, 1.0711020839172365, -0.9253739092346192, 0.8107088454711915, 0.31293208037109377]
        ref=dict(zip(jNamesl,pos))
        l.move_to_joint_positions(ref) 
        rospy.sleep(8)

        # Close Gripper 
        close.close()        
        rospy.sleep(1)
        
        # 5. Come Up
        pos = [0.41072335548706057, -0.1833107039428711, 0.6879903825805664, 1.0151117852233886, -0.7677573834594728, 1.1750292821777344, 0.2297136227233887]
        ref=dict(zip(jNamesl,pos))
        l.move_to_joint_positions(ref) 
        rospy.sleep(1)

    # 6. Tilt
    pos = [0.33594179216308595, -0.27228158953857423, 0.6231796943664552, 1.4220001887451172, -0.8329515668701173, 0.9150195389282227, 0.40880587950439456]
    ref=dict(zip(jNamesl,pos))
    l.move_to_joint_positions(ref) 
    rospy.sleep(1)

    # 6. Precontact Point
    #pos = [0.34783014325561523, -0.1852281799255371, 0.6201117327941895, 1.3406992070800783, -0.8709175913269044, 0.9085001205871582, 0.4003689851806641]
    pos = [0.34706315286254885, -0.18100973276367188, 0.6197282375976563, 1.338398235900879, -0.8666991441650391, 0.9096506061767579, 0.4007524803771973]
    ref=dict(zip(jNamesl,pos))
    l.move_to_joint_positions(ref) 
    rospy.sleep(1)
    
    if assembly:
        # 10. Contact
        pos = [0.3501311144348145, -0.17679128560180665, 0.6158932856323243, 1.334563283935547, -0.8655486585754395, 0.9050486638183595, 0.3984515091979981]
        ref=dict(zip(jNamesl,pos))
        l.move_to_joint_positions(ref) 
        rospy.sleep(1)
        
        # 10. Insertion A
        pos=[0.34744664805908204, -0.1825437135498047, 0.6128253240600586, 1.3548885293518067, -0.8820389520263673, 0.865903028881836, 0.4145583074523926]
        ref=dict(zip(jNamesl,pos))
        l.move_to_joint_positions(ref) 
        rospy.sleep(5)   
        
        # 11. Insertion B
        pos=[0.34783014325561523, -0.18651167512207033, 0.6051554201293946, 1.368694356427002, -0.8874078847778321, 0.8498913747314453, 0.4237621921691895]
        ref=dict(zip(jNamesl,pos))
        l.move_to_joint_positions(ref) 
        rospy.sleep(5)  
        
        # Open Gripper 
        #gc = gc.(gripper)
        #gc.command(position=100.0, effort=0.0)
        #rospy.sleep(1)
    
    # 12. Up a bit
    pos=[0.2956747965270996, -0.3271214026428223, 0.5648884244934083, 1.617966234173584, -0.9756117799804688, 0.5905826026611328, 0.600169982574463]
    ref=dict(zip(jNamesl,pos))
    l.move_to_joint_positions(ref) 
    rospy.sleep(1)          
    
    # 13. Back Home
#    pos=[0.3390097537353516, -0.26192721923217777, 0.611674838470459, 1.4285196070861816, -0.8605632210205079, 0.8624806970031739, 0.42798063933105474]
#    ref=dict(zip(jNamesl,pos))
#    l.move_to_joint_positions(ref) 
#    rospy.sleep(1)
    
    # 11. Go Home
    l.move_to_joint_positions(home)  
    print("Exiting: Planner")

if __name__ == "__main__":
    main()
