# agx 机械臂

[EN](README(EN).md)

![ubuntu](https://img.shields.io/badge/Ubuntu-20.04-orange.svg)

|ROS |STATE|
|---|---|
|![ros](https://img.shields.io/badge/ROS-noetic-blue.svg)|![Pass](https://img.shields.io/badge/Pass-blue.svg)|

## 安装方法

### 安装依赖

```shell
pip3 install python-can
```

```shell
pip3 install piper_sdk
```

## 快速使用

### 使能can模块

首先需要设置好shell脚本参数

#### 单条机械臂

##### pc只插入一个usb转can模块

- **此处使用`can_activate.sh`脚本**

直接执行

```bash
bash can_activate.sh can0 1000000
```

##### pc插入多个usb转can模块

- **此处使用`can_activate.sh`脚本**

拔掉所有can模块

只将连接到机械臂的can模块插入PC，执行

```shell
sudo ethtool -i can0 | grep bus
```

并记录下`bus-info`的数值例如`1-2:1.0`

ps:**一般第一个插入的can模块会默认是can0，如果没有查询到can可以使用`bash find_all_can_port.sh`来查看刚才usb地址对应的can名称**

假设上面的操作记录的`bus-info`数值为`1-2:1.0`

然后执行,查看can设备是否激活成功

```bash
bash can_activate.sh can_piper 1000000 "1-2:1.0"
```

ps:**此处的意思是，1-2:1.0硬件编码的usb端口插入的can设备，名字被重命名为can_piper，波特率为1000000，并激活**

然后执行`ifconfig`查看是否有`can_piper`，如果有则can模块设置成功

### 运行节点

#### 单个机械臂

节点名`piper_ctrl_single_node.py`

param

```shell
can_port:要打开的can路由名字
auto_enable:是否自动使能，True则开启程序就自动使能
# 注意这个设置为False，中断程序后再启动节点，机械臂会保持上次启动程序的状态
# 若上次启动程序机械臂状态为使能，则中断程序后再启动节点，机械臂仍为使能
# 若上次启动程序机械臂状态为失能，则中断程序后再启动节点，机械臂仍为失能
girpper_exist:是否有末端夹爪，True则说明有末端夹爪，会开启夹爪控制
rviz_ctrl_flag:是否使用rviz来发送关节角消息，True则接收rviz发送的关节角消息
#由于rviz中的joint7范围是[0,0.04]，而真实夹爪行程为0.08m，打开rviz控制后会将rviz发出的joint7乘2倍
```

`start_single_piper_rviz.launch`如下:

```xml
<launch>
  <arg name="can_port" default="can0" />
  <arg name="auto_enable" default="true" />
  <include file="$(find piper_description)/launch/display_xacro.launch"/>
  <!-- 启动机械臂节点 -->
  <node name="piper_ctrl_single_node" pkg="piper" type="piper_ctrl_single_node.py" output="screen">
    <param name="can_port" value="$(arg can_port)" />
    <param name="auto_enable" value="$(arg auto_enable)" />
    <param name="rviz_ctrl_flag" value="true" />
    <param name="girpper_exist" value="true" />
    <remap from="joint_ctrl_single" to="/joint_states" />
  </node>
</launch>
```

`start_single_piper.launch`如下:

```xml
<launch>
  <arg name="can_port" default="can0" />
  <arg name="auto_enable" default="true" />
  <!-- <include file="$(find piper_description)/launch/display_xacro.launch"/> -->
  <!-- 启动机械臂节点 -->
  <node name="piper_ctrl_single_node" pkg="piper" type="piper_ctrl_single_node.py" output="screen">
    <param name="can_port" value="$(arg can_port)" />
    <param name="auto_enable" value="$(arg auto_enable)" />
    <param name="rviz_ctrl_flag" value="true" />
    <param name="girpper_exist" value="true" />
    <remap from="joint_ctrl_single" to="/joint_states" />
  </node>
</launch>
```

启动控制节点

```shell
# 启动节点
roscore
rosrun piper piper_ctrl_single_node.py _can_port:=can0 _mode:=0
# 启动launch
roslaunch piper start_single_piper.launch can_port:=can0 auto_enable:=true
# 或，会以默认参数运行
roslaunch piper start_single_piper.launch
# 也可以用rviz开启控制,需要更改的参数如上
roslaunch piper start_single_piper_rviz.launch
```

## 注意事项

- 需要先激活can设备，并且设置正确的波特率，才可以读取机械臂消息或者控制机械臂

### piper自定义消息

ros功能包`piper_msgs`

机械臂自身状态反馈消息，对应can协议中`id=0x2A1`的反馈消息
说明

`PiperStatusMsg.msg`

```c
uint8 ctrl_mode
/*
0x00 待机模式  
0x01 CAN指令控制模式
0x02 示教模式
0x03 以太网控制模式
0x04 wifi控制模式
0x05 遥控器控制模式
0x06 联动示教输入模式
0x07 离线轨迹模式*/
uint8 arm_status
/*
0x00 正常
0x01 急停
0x02 无解
0x03 奇异点
0x04 目标角度超过限
0x05 关节通信异常
0x06 关节抱闸未打开 
0x07 机械臂发生碰撞
0x08 拖动示教时超速
0x09 关节状态异常
0x0A 其它异常  
0x0B 示教记录
0x0C 示教执行
0x0D 示教暂停
0x0E 主控NTC过温
0x0F 释放电阻NTC过温*/
uint8 mode_feedback
/*
0x00 MOVE P
0x01 MOVE J
0x02 MOVE L
0x03 MOVE C*/
uint8 teach_status
/*
0x00 关闭
0x01 开始示教记录（进入拖动示教模式）
0x02 结束示教记录（退出拖动示教模式）
0x03 执行示教轨迹（拖动示教轨迹复现）
0x04 暂停执行
0x05 继续执行（轨迹复现继续）
0x06 终止执行
0x07 运动到轨迹起点*/
uint8 motion_status
/*
0x00 到达指定点位
0x01 未到达指定点位*/
uint8 trajectory_num
/*0~255 (离线轨迹模式下反馈)*/
int64 err_code//故障码
bool joint_1_angle_limit//1号关节通信异常（0：正常 1：异常）
bool joint_2_angle_limit//2号关节通信异常（0：正常 1：异常）
bool joint_3_angle_limit//3号关节通信异常（0：正常 1：异常）
bool joint_4_angle_limit//4号关节通信异常（0：正常 1：异常）
bool joint_5_angle_limit//5号关节通信异常（0：正常 1：异常）
bool joint_6_angle_limit//6号关节通信异常（0：正常 1：异常）
bool communication_status_joint_1//1号关节角度超限位（0：正常 1：异常）
bool communication_status_joint_2//2号关节角度超限位（0：正常 1：异常）
bool communication_status_joint_3//3号关节角度超限位（0：正常 1：异常）
bool communication_status_joint_4//4号关节角度超限位（0：正常 1：异常）
bool communication_status_joint_5//5号关节角度超限位（0：正常 1：异常）
bool communication_status_joint_6//6号关节角度超限位（0：正常 1：异常）
```

机械臂末端位姿控制，注意:有些奇异点无法到达

`PosCmd.msg`

```c
float64 x
float64 y
float64 z
float64 roll
float64 pitch
float64 yaw
float64 gripper
int32 mode1//暂时无用
int32 mode2//暂时无用
```

## 使用仿真

### rviz

`display_xacro.launch` 打开rviz

执行

```shell
cd Piper_ros
source devel/setup.bash
roslaunch piper_description display_xacro.launch
```

运行后会发布`/joint_states`,可以通过`rostopic echo /joint_states` 查看

![ ](./asserts/pictures/tostopic_list.jpg)

同时会弹出两个页面如下，滑动条数值对应`/joint_states`数值，拖动滑动条可以改变其数值，rviz中的模型也会随动

有时候滑动条窗口没看到，看下dock栏是否有窗口，可能是被其它页面覆盖了

![ ](./asserts/pictures/piper_rviz.jpg)

### gazebo

执行

```shell
cd Piper_ros
source devel/setup.bash
roslaunch piper_description gazebo_xacro.launch
```

![ ](./asserts/pictures/piper_gazebo.jpg)

运行后会发布

```shell
/piper_description/joint_states
/piper_description/joint1_position_controller/command
/piper_description/joint2_position_controller/command
/piper_description/joint3_position_controller/command
/piper_description/joint4_position_controller/command
/piper_description/joint5_position_controller/command
/piper_description/joint6_position_controller/command
/piper_description/joint7_position_controller/command
/piper_description/joint8_position_controller/command
```

`/piper_description/joint_states` 是gazebo的机械臂关节状态的虚拟反馈话题，其余的为gazebo的订阅节点

这几个command话题用来控制gazebo中的机械臂运动，关节在xacro中的限位如下，其中，1～6关节单位为弧度，7、8关节单位为m，因为7、8关节对应夹爪

|joint_name|     limit     |
|----------|  ---------    |
|joint1    | [-2.618,2.618]|
|joint2    | [0,3.14]|
|joint3    | [-2.697,0]|
|joint4    | [-1.832,1.832]|
|joint5    | [-1.22,1.22]|
|joint6    | [-3.14,3.14]|
|joint7    | [0,0.038]|
|joint8    | [-0.038,0]|

假如想要令关节1运动90度，发送

```shell
rostopic pub /piper_description/joint1_position_controller/command std_msgs/Float64 "data: 1.57"
```

可以看到机械臂link1旋转了90度

![ ](./asserts/pictures/piper_gazebo_90.jpg)
