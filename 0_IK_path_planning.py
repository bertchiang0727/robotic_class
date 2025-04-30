#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import atan2, acos, asin, sqrt, sin, cos, pi
from moveit_commander.conversions import pose_to_list
import math

def all_close(goal, actual, tolerance):

  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True


class MoveGroupPythonIntefaceTutorial(object):
  
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()
    
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group_name = "ldsc_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    planning_frame = move_group.get_planning_frame()
    # print "============ Planning frame: %s" % planning_frame

    # move_group.set_workspace([-0.2984,-0.2984,0.0,0.2984,0.2984,0.4404])

    group_names = robot.get_group_names()


    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    
    self.group_names = group_names

    joint_angles = move_group.get_current_joint_values()
    self.joint_angles = joint_angles

  def go_to_joint_state(self):
    
    move_group = self.move_group
    joint_angles = self.joint_angles

    joint_goal = move_group.get_current_joint_values()

    joint_goal[0] = joint_angles[0]
    joint_goal[1] = joint_angles[1]
    joint_goal[2] = joint_angles[2]
    joint_goal[3] = joint_angles[3]

    move_group.go(joint_goal, wait=True)

    move_group.stop()

    current_joints = move_group.get_current_joint_values()
    current_pose = self.move_group.get_current_pose('link5').pose
    print ("current pose:")
    # print (current_pose.position) 
    print ("x: %.5f" %current_pose.position.x)
    print ("y: %.5f" %current_pose.position.y)
    print ("z: %.5f" %current_pose.position.z)

    current_rpy = self.move_group.get_current_rpy('link5')
    print ("rol: %.5f" %current_rpy[0])
    print ("pit: %.5f" %current_rpy[1])
    print ("yaw: %.5f" %current_rpy[2])
    print ("")
    return all_close(joint_goal, current_joints, 0.01)


def Your_IK(x, y, z, pitch):
    # === Link lengths (unit: meter) ===
    l0 = 0.0600
    l1 = 0.0820
    l2 = 0.1450
    l3 = 0.1800
    l4 = 0.082
    l5 = 0.0040
    l_offset = l5  # tip length affects final pitch

    # === Step 1: j1 ===
    j1 = math.atan2(y, x)
    print("j1 = %.4f rad (%.2f°)" % (j1, math.degrees(j1)))

    # === Step 2: wrist center ===s
    r = math.sqrt(x**2 + y**2)
    r_wrist = r -l4
    z_wrist = z - l0 - l1 
    if(y == 0):
      z_wrist = z - l0 - l1 - 0.03 
    print("r_wrist = %.4f, z_wrist = %.4f" % (r_wrist, z_wrist))

    # === Step 3: distance D ===
    D = math.sqrt(r_wrist**2 + z_wrist**2)
    print("D = %.4f" % D)

    # === Step 4: j3 (elbow) ===
    cos_j3 = (l2**2 + l3**2 - D**2) / (2 * l2 * l3)
    sin_j3_a = sqrt(1- cos_j3**2)
    sin_j3_b = -sqrt(1- cos_j3**2) 
    j3_a = math.atan2(sin_j3_a,cos_j3)  # elbow-down
    j3_b = math.atan2(sin_j3_b,cos_j3)  # elbow-up
    
    print("j3_a = %.4f rad (%.2f°)" % (j3_a, math.degrees(j3_a)))
    print("j3_b = %.4f rad (%.2f°)" % (j3_b, math.degrees(j3_b)))


    # === Step 5: j2 (shoulder) ===
    theta1 = math.atan2(r_wrist, z_wrist)
    cos_theta2 = (l2**2 + D**2 - l3**2) / (2 * l2 * D)
    sin_j2 = sqrt(1- cos_theta2**2)
    j2_a = theta1 - math.atan2(sin_j2,cos_theta2)  # elbow-down
    j2_b = theta1 + math.atan2(sin_j2,cos_theta2)  # elbow-up

    print("j2_a = %.4f rad (%.2f°)" % (j2_a, math.degrees(j2_a)))
    print("j2_b = %.4f rad (%.2f°)" % (j2_b, math.degrees(j2_b)))

    # === Step 6: j4 (wrist) ===

    j4_a = pitch - j2_a - j3_a
    j4_b = pitch - j2_b - j3_b

    print("j4_a = %.4f rad (%.2f°)" % (j4_a, math.degrees(j4_a)))
    print("j4_b = %.4f rad (%.2f°)" % (j4_b, math.degrees(j4_b)))


    # # === Step 7: Choose better branch ===
    if (abs(j2_a) + abs(j3_a) + abs(j4_a)) < abs(abs(j2_b) + abs(j3_b) + abs(j4_b)) :
        print("✔️ Using elbow-down solution")
        j2, j3, j4 = j2_a, j3_a, j4_a
    else:
        print("✔️ Using elbow-up fallback")
        j2, j3, j4 = j2_b, j3_b, j4_b
    # === Final debug print ===
    print("j2 = %.4f rad (%.2f°)" % (j2, math.degrees(j2)))
    print("j3 = %.4f rad (%.2f°)" % (j3, math.degrees(j3)))
    print("j4 = %.4f rad (%.2f°)" % (j4, math.degrees(j4)))

    return [j1, j2, j3, j4]


def main():
    try:
        path_object = MoveGroupPythonIntefaceTutorial()
        print("請輸入目標位置（格式：x y z pitch），例如：0.25 0.15 0.05 1.57")
        print("按 Ctrl+C 結束")
        while not rospy.is_shutdown():
            try:
                user_input = input("輸入目標點：")
                values = list(map(float, user_input.strip().split()))

                if len(values) != 4:
                    raise ValueError("請輸入 4 個數值：x y z pitch")

                x_input, y_input, z_input, q_input = values

                path_object.joint_angles = Your_IK(x_input, y_input, z_input, q_input)
                print("✅ IK 解算結果:", path_object.joint_angles)
                path_object.go_to_joint_state()

            except Exception as e:
                print("❌ 輸入錯誤，返回 home，錯誤原因：", e)
                path_object.joint_angles = [0, -pi/2, pi/2, 0]
                path_object.go_to_joint_state()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':
  main()

