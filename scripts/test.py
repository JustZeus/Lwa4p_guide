#!/usr/bin/env python
############################################
# @file		test.py	 		   #
# @brief SCHUNK Lwa4p Move it demo program #
# @author   Ricardo Zamudio		   #
# @date		2022/01/02		   #
############################################

import moveit_commander
import rospy
import geometry_msgs.msg
import time

def main():
	rospy.init_node("moveit_command_sender")

	robot = moveit_commander.RobotCommander()
    	print '''


 _____ _____  _   _ _   _ _   _  _   __  ___  ___ _______   _______ _____ _____  
/  ___/  __ \| | | | | | | \ | || | / /  |  \/  ||  ___\ \ / /_   _/  __ \  _  | 
\ `--.| /  \/| |_| | | | |  \| || |/ /   | .  . || |__  \ V /  | | | /  \/ | | | 
 `--. \ |    |  _  | | | | . ` ||    \   | |\/| ||  __| /   \  | | | |   | | | | 
/\__/ / \__/\| | | | |_| | |\  || |\  \  | |  | || |___/ /^\ \_| |_| \__/\ \_/ / 
\____/ \____/\_| |_/\___/\_| \_/\_| \_/  \_|  |_/\____/\/   \/\___/ \____/\___/  
                                                                                 
                                                                                 
______                     ______       _ _    _                    ___          
| ___ \                    | ___ \     | | |  | |                  /   |         
| |_/ /____      _____ _ __| |_/ / __ _| | |  | |  __      ____ _ / /| |_ __     
|  __/ _ \ \ /\ / / _ \ '__| ___ \/ _` | | |  | |  \ \ /\ / / _` / /_| | '_ \    
| | | (_) \ V  V /  __/ |  | |_/ / (_| | | |  | |___\ V  V / (_| \___  | |_) |   
\_|  \___/ \_/\_/ \___|_|  \____/ \__,_|_|_|  \_____/\_/\_/ \__,_|   |_/ .__/    
                                                                       | |       
                                                                       |_|       
           
                                                                                 
This test Script was created by Ricardo Zamudio
Applications engineer at SCHUNK MEXICO
Ricardo.Zamudio@mx.schunk.com
 '''
	print "=" * 10, " Robot Groups:"
	print robot.get_group_names()

	#print "=" * 10, " Printing robot state"
	#print robot.get_current_state()
	#print "=" * 10 

	arm = moveit_commander.MoveGroupCommander("manipulator")
	arm.set_max_velocity_scaling_factor(.20)
	print "=" * 10, " Printing robot joint states"
    print arm.get_current_joint_values()
	
	print "=" * 10, " Intialize in home position"
    arm.go([0,0,0,0,0,0], wait=True)
 	arm.stop()
	time.sleep(1)

	print "=" * 10, " Moving Robot using joint state Goal", "=" * 10
	print "=" * 10, " Moving to position 1"
	arm.go([0.0,-0.7462504282752155, -0.5695882013883494, -1.0179807395182126, 0.03600614246864302, 0.49300315381083826], wait=True)
 	arm.stop()
	time.sleep(1)

	print "=" * 10, " Returning to home position"
	arm.go([0,0,0,0,0,0], wait=True)
 	arm.stop()
	time.sleep(1)

	print "=" * 10, " Moving to position 2"
    arm.go([0.0,0.4741062629069983, -0.7912476227793528, 0.0041526706870680385, -2.4662076334003302e-05, 2.4489075676648042e-05],wait=True)
 	arm.stop()
	time.sleep(1)

	print "=" * 10, " Returning to home position"
    arm.go([0,0,0,0,0,0], wait=True)
 	arm.stop()
	time.sleep(1)

	print "=" * 10, " Getting current pose"
	print arm.get_current_pose().pose
	
	# A representation of pose in free space, is composed of position and orientation. 
	print "=" * 10, " Move to pose in free space", "=" * 10
        ## BEGIN_SUB_TUTORIAL plan_to_pose
        ##
        ## Planning to a Pose Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.4
    pose_goal.position.y = 0.1
	pose_goal.position.z = 0.4

    arm.set_pose_target(pose_goal)

        ## Now, we call the planner to compute the plan and execute it.
        # `go()` returns a boolean indicating whether the planning and execution was successful.
    success = arm.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
    arm.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
	arm.clear_pose_targets()

	time.sleep(1)

	print "=" * 10, " Returning to home position"
    arm.go([0,0,0,0,0,0], wait=True)
 	arm.stop()
	time.sleep(1)


	print "=" * 10, " END OF PROGRAM","=" * 10

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
