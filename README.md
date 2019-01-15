# RoboMaster 机甲大师赛 机器人统一框架
## 上海交通大学交龙战队

### 电机配置

使用者在MotorTask.c中配置电机相应信息，具有三种已规范初始化方法：
> 1. AngleBasedMotorInfo(用于控制位置速度双环电机，控制函数选择ControlNM)
> 1. GimbalMotorInfo(用于控制云台电机，控制函数选择ControlGMY或者ControlGMP)
> 1. SpeedBasedMotorInfo(用于控制单速度环电机，控制函数选择ControlCM)

初始化完成后将其挂在在对应的MotorINFO canx[8] 数组上，其中x可取1、2，分别代表两条can线，在数组中的位置表示电机id号码，0-7号位一一对应0x201-0x208。

务必要根据实际挂载电机的情况在“includes.h”里#define CAN11-CAN22。

### 业务代码书写
FunctionTask.c中书写所有的控制代码，如：从遥控器/键鼠接收到xxx消息，执行xxx动作。

动作控制通过更改MotorINFO结构体中的Target来实现（底盘电机除外）。

底盘电机控制有一套相互影响的逻辑，通过更改结构体ChassisSpeed_Ref_t的对象ChassisSpeedRef中的三个子参量进行平移旋转控制。

### 已完成模块启用
框架内封装了多个实用模块，包括但不限于功率限制，射速限制，上位机数据交换，自动瞄准等。
这些模块不对所有机器人有效，可以选择性开启，可以在includes.h文件中，选择是否开启其功能。

|宏定义名称|功能说明|
|-|-|
| DEBUG_MODE | 启用云台调参模式，配合PCInterFace.exe可以观察云台响应曲线，调整pid各项参数|
|TEST_MODE  |用于测试各项未完成功能|
|USE_AUTOAIM |启用自瞄功能|
|USE_IMU     |启用板载陀螺仪（与USE_GYRO冲突，同时启用优先）|
|USE_GYRO    |启用外置陀螺仪（与USE_IMU冲突）|
|USE_CHASSIS_FOLLOW |启用底盘跟随云台功能，不开启时底盘独立运动 |
|USE_SUPER_CAP|启用超级电容功能|
|USE_POWER_LIMIT|启用功率限制|
