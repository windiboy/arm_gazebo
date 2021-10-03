inet 192.168.1.230  netmask 255.255.255.0  broadcast 192.168.1.1
sh -c "$(curl -fsSL <https://raw.github.com/robbyrussell/oh-my-zsh/master/tools/install.sh>)"
git clone git://github.com/zsh-users/zsh-autosuggestions $ZSH_CUSTOM/plugins/zsh-autosuggestions
git clone <https://github.com/zsh-users/zsh-syntax-highlighting.git> ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-syntax-highlighting

sudo apt install ros-melodic-ar-track-alvar

## 2021年4月12日

- group manipulator not found
默认的规划组的名字是manipulator 而我们的是gluon 需要再handeye_server_robot.py中指定
- easy_handeye start pose faile
需要注释handeye_robot.py _check_target_poses 的两行

```python
# if len(plan.joint_trajectory.points) == 0 or CalibrationMovements._is_crazy_plan(plan, joint_limits):
#     return False
```

- 标定到某个点规划不出路径，导致程序退出

## 2021年4月22日

1. 完成目标识别，并转化到base_link坐标系
    - 相机和基座的精确坐标关系没有，还是得标定？而且如果相机仰角的话，tf关系就更复杂了
    - 相机不仰角得话，走进了看不见门把手
    - TODO：可以开发算法，在机器人靠近门的过程中，记录机器人里程计和相机识别到的tf变换，循环迭代，减小误差
2. 底盘运动待开发
3. SLAM相关待开发

## 2021年9月27日

1. 原来机器人被拆了，准备做在gazebo中仿真开门
2. rm机械臂有gazebo的相关配置，realsense也有，多线激光雷达也有，只不过不是镭神科技的
3. 问题：openvino在gazebo中怎么做识别？
4. 问题：底盘的模型得重新建，怎么和机械臂连起来？