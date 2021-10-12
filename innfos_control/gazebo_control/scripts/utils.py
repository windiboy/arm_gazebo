#!/usr/bin/env python2
# coding=utf-8
import numpy as np
import geometry_msgs
from tf_conversions import transformations

def GetCrossAngle(line1, line2):
    # x = [x1, x2, x3, x4] 按顺序分别是第一条线 起点 终点 第二条起点终点
    x = [line1.start[0], line1.end[0], line2.start[0], line2.end[0]]
    y = [line1.start[1], line1.end[1], line2.start[1], line2.end[1]]
    if len(x) != 4 or len(y) != 4:
        return 0.0
    arr_0 = np.array([(x[1] - x[0]), (y[1] - y[0])])
    arr_1 = np.array([(x[3] - x[2]), (y[3] - y[2])])
    cos_value = (float(arr_0.dot(arr_1)) / (np.sqrt(arr_0.dot(arr_0))
                                            * np.sqrt(arr_1.dot(arr_1))))   # 注意转成浮点数运算
    return np.arccos(cos_value) * (180/np.pi)


def ListToPose(list):
    pose_goal = geometry_msgs.msg.Pose()
    q = transformations.quaternion_from_euler(
        list[3], list[4], list[5])
    pose_goal.position.x = list[0]
    pose_goal.position.y = list[1]
    pose_goal.position.z = list[2]
    pose_goal.orientation.x = q[0]
    pose_goal.orientation.y = q[1]
    pose_goal.orientation.z = q[2]
    pose_goal.orientation.w = q[3]
    return pose_goal


if __name__ == "__main__":
    x = [1.3794386386871338, 1.3873677253723145,
         0.6680925488471985, 1.2863837480545044]
    y = [0.610981285572052, 1.7962329387664795, -
         0.8605140447616577, -0.43797817826271057]
    angle = GetCrossAngle(x, y)
    print(angle)
