# Robocup_Localization
#交叉定位组技术文档(PFL Document)

@陆恺, 2019.07.12

![](/Users/lukai/Documents/localization_src/1.png)

PFL(Particle Filter Localization，基于粒子滤波的定位算法），是一种广泛应用于**机器人定位**的重要方法。粒子滤波，是使用蒙特卡罗方法(Monte Carlo method)的递归**滤波**器，透过一组具有权重的随机样本(称为粒子)来表示随机事件的后验概率，从观测序列估计出动态系统的状态，**粒子滤波**器可以运用在任何状态空间的模型上。

Tsinghua Hephaestus Robotics的交叉定位系统以PFL为核心算法，使用里程计积分和视觉感知，实现定位。从2014年起，由Sotirios Stasinopoulos主要开发完成后，定位组改进完善，在Robocup2014-2019的Adult Size比赛中应用。

## 目录

- [特性](#一、特性)
- [安装](#二、安装)
- [文档](#三、文档)
- [教程](#四、文档)
- [版本历史](#五、版本历史)
- [谁使用PFL?](#六、谁使用PFL？)
- [贡献者](#七、贡献者)

## 一、特性

- 基于ROS和C++，使用粒子滤波（particle filter）算法和视觉感知定位，适用于大部分机器人。
- 支持具有感知功能和里程积分功能的机器人。
- 感知接口目前为视觉感知，可以使用其他能够提供感知信息的接口（需改部分代码）。
- 里程积分接口目前为UDP，可以使用Serial或其他接口（需自行编写transform）。
- 具有数据输入输出流控制节点
- 具有完善的可视化功能，可以查看机器人位置和方向、障碍物、足球和标记点。
- 具有完善的地图模型，可以导入Adultsize18、Adultsize19、Teensize等多种地图。
- 适用于linux系统（基于ROS）。

## 二、安装

#### 环境

- ROS kinetic
- OpenCV3
- CMake
- Linux（推荐Ubuntu 16.04以上）

#### 开始安装

建议熟悉ROS的相关操作，如果不熟悉，请参阅ROS指南[此处](http://wiki.ros.org/cn/ROS/Tutorials)。

需要重点关注的是：

- ROS的基本框架：包括节点node，消息message，主题topic；
- ROS基于C++的代码编写：包括接受发送msg，回调callback，循环rate，调试debug（info等）。
- ROS的基本命令：roslauch、rosrun、catkin_make、rostopic、rosbag等。

如果已经熟悉了ROS的架构和使用，开始编译流程：

```
cd robot_workspace/src/localization && ls
```

这里假定已经拥有全部的pkg，包括决策、定位、视觉和UDP。那么会显示该文件夹下的文件，重点检查msg、src和CMakeLists.txt。

开始编译，编译全部

```
cd robot_workspace
catkin_make
```

如果出现了缺少包，一般可以这样处理:

```
sudo apt-get install [pkg-not-found]
```

如果出现编译失败，缺少头文件或消息，则按顺序编译：

```
catkin_make --pkg [pkg]
```

顺序一般是vision、decision、localization。

最后，(也可以将此写入`.bashrc`)

```
source ~/robot_workspace/devel/setup.bash
```

#### 开始运行

(默认已经source到`.bashrc`中)

```
roslaunch localization_testing.launch
```

应呈现：

![](/Users/lukai/Documents/localization_src/2.png)



1. 如果是拥有全部包且在真实机器人上，不用单独运行此节点，应运行：

```
sh start_all.sh
```

2. 如果只有bag，请运行：

```
rosbag play xxx.bag
```

请检查：

```
rostopic list
```

应看到：

```
/behavior/initial_pose
/vision/ball
/vision/opponents
/vision/landmarks
/vision/goalpost
/ros_udp_vision/msg_from_gait
```

那么可以运行（任一行）：

```
roslaunch localization_testing.launch
roslaunch localization_19.launch
roslaunch localization_module.launch && roslaunch localization_visualization.launch
```



## 三、文档

### 文档目录

- **架构**：PFL-ROS-C++项目架构
- 数据输入输出流
- 粒子滤波
- PFL交叉定位
- 可视化方法
- 参数管理

### 架构

PFL粒子滤波定位的整体架构是在ROS平台上以C++语言开发的，基本的框架如下：

![](/Users/lukai/Documents/localization_src/3.png)

其中，核心的部分为3个模块：数据输入输出流管理、粒子滤波定位核心和可视化程序。

### 数据输入输出流管理

- 数据流首先输入视觉信息和里程计的信息，通过data_input_output的节点进行处理：

  ```c++
  sub_vision_landmarks = ldio_nh.subscribe("vision/landmarks",10,&LocalizationDataInputOutput::visionLandmarksCallback,this);
  sub_vision_goal = ldio_nh.subscribe("vision/goalpost",10,&LocalizationDataInputOutput::visionGoalCallback,this);
  sub_vision_ball = ldio_nh.subscribe("vision/ball",10,&LocalizationDataInputOutput::visionBallCallback,this);
  sub_vision_opponents = ldio_nh.subscribe("vision/opponents",10,&LocalizationDataInputOutput::visionOpponentCallback,this);
  subscriber_initial_pose = ldio_nh.subscribe("/behavior/initial_pose", 10, &LocalizationDataInputOutput::handleInitPose,this);
  sub_odom = ldio_nh.subscribe("ros_udp_vision/msg_from_gait", 10, &LocalizationDataInputOutput::odomCb, this);
  sub_head = ldio_nh.subscribe("arm_head_motion/head_deg", 10, &LocalizationDataInputOutput::headCb, this);
  sub_goal_x = ldio_nh.subscribe("/behavior/initial_pose", 10, &LocalizationDataInputOutput::goalpostCb, this);
  ```

  输入流的数据经过转换后发出的消息为world_objects_detected

  ```c++
  pub_localization_world_objects = ldio_nh.advertise<localization::WorldObjects>("/localization/world_objects_detected",10);
  ```

  这里需要涉及坐标系的转换:

  ```c++
  msg_current_pose.robotPose.x = initial_x + gait_input_msg.gait_data[0] * cos(initial_theta) - gait_input_msg.gait_data[1] * sin(initial_theta);
  msg_current_pose.robotPose.y = initial_y + gait_input_msg.gait_data[0] * sin(initial_theta) + gait_input_msg.gait_data[1] * cos(initial_theta);
  msg_current_pose.robotPose.theta = initial_theta + gait_input_msg.gait_data[2];
  ```

  ![](/Users/lukai/Documents/localization_src/4.png)

  

  值得注意的是，这里用于particle filter的msg只是world_objects_detected，这实际上是视觉信息中对于landmark点的一个整合，vision给予到定位组的msg中，是landmarks相对于机器人的位置，而机器人的坐标系是时刻在变化的，但是全局坐标系是不变的。这里需要对坐标系进行一个解释：

  全局坐标系和机器人坐标系：

  ![](/Users/lukai/Documents/localization_src/5.png)

  当然，data节点做的不只是这一件事情，但这是进行粒子滤波的第一步，如果发现结果出现严重偏差，首先考虑是否是坐标系方向有误。

### 粒子滤波

- 粒子滤波器(particle filter)是一种使用蒙特卡罗方法(Monte Carlo method)的递归滤波器，透过一组具有权重的随机样本(称为粒子)来表示随机事件的后验几率，从含有噪声或不完整的观测序列，估计出动态系统的状态，粒子滤波器可以运用在任何状态空间的模型上。

  粒子滤波器是卡尔曼滤波器(Kalman filter)的一般化方法，卡尔曼滤波器建立在线性的状态空间和高斯分布的噪声上；而粒子滤波器的状态空间模型可以是非线性，且噪声分布可以是任何型式。

- 原理：粒子滤波器能够从一系列含有噪声或不完整的观测值中，估计出动态系统的内部状态。在动态系统的分析中，需要两个模型，一个用来描述状态随时间的变化(系统模型)，另一个用来描述每个状态下观测到的噪声(观测模型)，将这两个模型都用几率来表示。

  在许多情况下，每得到一个新的观测值时，都必须对系统做出一次估计，利用递回滤波器，能够有效地达到这样的目的。递回滤波器会对得到的资料做连续处理，而非分批处理，因此不需要将完整的资料储存起来，也不需要在得到新的观测值时，将现有的资料重新做处理。递回滤波器包含两个步骤：

  预测：利用系统模型，由前一个状态的资讯预测下一个状态的几率密度函数。

  更新：利用最新的观测值，修改预测出的几率密度函数。

  借由贝叶斯推论(Baysian inference)，我们可以描述出状态空间的几率，并在得到新的观测值时，对系统做出更新，因而达成上述目的。

- 预测

  利用查普曼-科尔莫戈罗夫等式(Chapman–Kolmogorov equation)，可以由**状态转移函数**与时间![k-1](https://wikimedia.org/api/rest_v1/media/math/render/svg/21363ebd7038c93aae93127e7d910fc1b2e2c745)的几率密度函数![{\displaystyle p(x_{k-1}\mid y_{1:k-1})}](https://wikimedia.org/api/rest_v1/media/math/render/svg/284baf76e0b15fbe6a2cf4a9c20ed537dc772b4b)，计算出时间![k](https://wikimedia.org/api/rest_v1/media/math/render/svg/c3c9a2c7b599b37105512c5d570edc034056dd40)的先验几率![{\displaystyle p(x_{k}\mid y_{1:k-1})}](https://wikimedia.org/api/rest_v1/media/math/render/svg/9dd6a9defc7db795b510ce07950fa2b409261980)

  ${\displaystyle {\begin{aligned}p(x_{k}\mid y_{1:k-1})&=\int p(x_{k},x_{k-1}\mid y_{1:k-1})d{x_{k-1}}\\&=\int p(x_{k}\mid x_{k-1},y_{1:k-1})p(x_{k-1}\mid y_{1:k-1})d{x_{k-1}}\\&=\int p(x_{k}\mid x_{k-1})p(x_{k-1}\mid y_{1:k-1})d{x_{k-1}}\\\end{aligned}}}$

  其中，由于状态转移模型被假设为一阶马可夫过程，时间![k-1](https://wikimedia.org/api/rest_v1/media/math/render/svg/21363ebd7038c93aae93127e7d910fc1b2e2c745)的状态只由时间决定，因此${\displaystyle p(x_{k}\mid x_{k-1},y_{1:k-1})=p(x_{k}\mid x_{k-1})}$。几率模型![{\displaystyle p(x_{k}\mid x_{k-1})}](https://wikimedia.org/api/rest_v1/media/math/render/svg/12f7cacc0eef7208ba7a5b491b74db8feb254c37)由**状态转移函数**![{\displaystyle x_{k}=f_{k}(x_{k-1},v_{k-1})}](https://wikimedia.org/api/rest_v1/media/math/render/svg/8637d4afbb5d9105ff9f2ad756779370806b29be)和${\displaystyle v_{k-1}}$的统计值得到。

- 更新

  在时间{\displaystyle k}![k](https://wikimedia.org/api/rest_v1/media/math/render/svg/c3c9a2c7b599b37105512c5d570edc034056dd40)，我们得到观测值![y_{k}](https://wikimedia.org/api/rest_v1/media/math/render/svg/4b2ab0248723a410cc2c67ce06ad5c043dcbb933)，因此可以利用贝叶斯定理，由先验几率![{\displaystyle p(x_{k}\mid y_{1:k-1})}](https://wikimedia.org/api/rest_v1/media/math/render/svg/9dd6a9defc7db795b510ce07950fa2b409261980)得到后验几率![{\displaystyle p(x_{k}\mid y_{1:k})}](https://wikimedia.org/api/rest_v1/media/math/render/svg/2e3fe04b481fc3d08eb8589688e261f056f49fea)，也就是考虑观测值后得到的几率。

  ![{\displaystyle p(x_{k}\mid y_{1:k})={p(y_{k}\mid x_{k})p(x_{k}\mid y_{1:k-1}) \over p(y_{k}\mid y_{1:k-1})}}](https://wikimedia.org/api/rest_v1/media/math/render/svg/4a8bc4c80dfccd034e704f1541c24de110689e14)

  其中的归一化常数为

  ![{\displaystyle p(y_{k}\mid y_{1:k-1})=\int p(y_{k}\mid x_{k})p(x_{k}\mid y_{1:k-1})dx_{k}}](https://wikimedia.org/api/rest_v1/media/math/render/svg/869c7885ada9296a0097a57fe9f210cea8d58936)

  其中的似然函数![{\displaystyle p(y_{k}\mid x_{k})}](https://wikimedia.org/api/rest_v1/media/math/render/svg/a7fc61ab323adc4288f765e257620371760e2330)由**观测函数**![{\displaystyle y_{k}=h_{k}(x_{k},n_{k})}](https://wikimedia.org/api/rest_v1/media/math/render/svg/ab4dbec42c715dd4d2ccb239a0e23a7ed4f0e3b7)和![n_{k}](https://wikimedia.org/api/rest_v1/media/math/render/svg/4b18f35b25d18a03f414bb9510599938e94c4768)的统计值得到。

  上述**“预测”**与**“更新”**的递回关系，是贝叶斯最佳解的基本概念，然而公式中运用到的积分，对于一般非线性、非高斯的系统，难以得到解析解，因此需要利用蒙地卡罗方法来近似贝叶斯最佳解。

- 对于粒子滤波的通俗理解：

- 粒子滤波定位机器人的过程分为三步，需要三个信息：

  1.机器人上一时刻位置

  2.机器人的运动是什么

  3.测量结果，比如视觉测量信息

  对应的是：

  1.Prediction Step

  2.Innovation Step

  3.Re-sampling Step

  **第一步，Prediction Step（预测）**

  用到上一步的结果，和运动。请看下图

  ![](/Users/lukai/Documents/localization_src/6.png)

  我们用“粒子”点来表示车的位置，也就是上图的×。上一步的位置，我们用红色的x来表示，他们都是机器人的可能的位置。比如上图的(1,5),(3,5),(3,7),(5,5)。我们可以给每个位置定个权重，然后加权平均一下。在预测这步权重都是一样的，我们有四个粒子，那么每个权重就是0.25。

  也就是我们有四个粒子x1，x2，x3，x4，x[(横坐标x，纵坐标y)，权重w]

  x1[(1,5),0.25]

  x2[(3,5),0.25]

  x3[(3,7),0.25]

  x4[(5,5),0.25]

  然后机器人有个运动，这个运动是里程计传感器得来的。假设我们现在知道了运动是(5,4)。但是还有个误差。

  预测这步就是把所有权重不变，在状态（运动）加上这个运动的结果。

  x1[(6,9),0.25]

  x2[(8,9),0.25]

  x3[(8,11),0.25]

  x4[(10,9),0.25]

  如果知道噪声的大小，可以在结果上加上一个随机数。结果如下：

  x1[(6.1,9.2),0.25]

  x2[(7.9,9),0.25]

  x3[(8.3,10.8),0.25]

  x4[(10.1,8.9),0.25]

  

  **第二步，Innovation Step**

  这步的作用是，利用测量或图像定位进行定位权重的分配。

  ![](/Users/lukai/Documents/localization_src/7.png)

  测量值是下图那个绿色的点，但是测量是不准的嘛，假设其遵循高斯分布。我们通过权重把这个测量和那一堆现在有的，经过prediction的粒子的信息融合在一起。按照测量的概率分布来重新分配权重，并归一化。

  x1[(6.1,9.2),0.26]

  x2[(7.9,9),0.33]

  x3[(8.3,10.8),0.23]

  x4[(10.1,8.9),0.16]


  **第三步，Resampling step**

  将之前那些粒子舍弃，重新根据概率生成粒子。一般粒子数量不变。

  x1[(6.1,9.2),0.26]

  x2[(7.9,9),0.33]

  x3[(8.3,10.8),0.23]

  x4[(10.1,8.9),0.16]

  例如x2的概率是0.33，那么新生成的粒子可能有两个x2，因其它概率最大，而x4的概率较小可能没有。

  然后认为摇出来的新的粒子权重相等。

  x1[(6.1,9.2),0.25]

  x2[(7.9,9),0.25]

  x3[(8.3,10.8),0.25]

  x4[(7.9,9),0.25]

  最终的估计值可以是四个粒子的平均。

  

  粒子滤波核心步骤即是循环上述过程。



### PFL交叉定位

PFL交叉定位首先构建了粒子滤波类：

```c++
LocalizationPF::LocalizationPF(ros::NodeHandle &nh)
    : ParticleFilter<Vec3f, Vec3f, LandmarkObservation, Mat22f>(
          new PoseParticleInitializer(		Vec3f(m_mean.x(), m_mean.y(), m_mean.z()),
                                            Vec3f(0.2, 0.2, 10.0 * M_PI / 180.0) ),
          //new PoseParticleInitializer(Vec3f(0.0,0.0,0.0),Vec3f(1.0,1.0,1.0)),
          new OdometryModel(),
          new ObservationModel(this),
          80, // minimum number of particles
          150, // maximum number of particles
          0.1, // slow average likelihood learning rate
          0.5 // fast average likelihood learning rate
          )
    , m_field(field_model::FieldModel::getInstance())
{
    // store time of localization code initialization
    m_lastTime = ros::Time::now();
    // subscribe to topic for detected objects
    m_sub_vision = nh.subscribe("/localization/world_objects_detected", 10, &LocalizationPF::handleDetections, this);
    // subscribe to topic for the orientation heading from the IMU
    //m_sub_imu = nh.subscribe("/decision/serial_receiver", 10, &LocalizationPF::handleImu, this);
    m_sub_udp = nh.subscribe("/ros_udp_vision/msg_from_gait", 10, &LocalizationPF::handleUDP, this);
    m_sub_pose = nh.subscribe("/localization/current_pose", 10, &LocalizationPF::handlePose, this);
    // subscribe to topic for initial pose given by behavior module
    m_sub_initial_pose = nh.subscribe("/behavior/initial_pose", 10, &LocalizationPF::handleInitPose, this);
    // advertise full particle set with poses and weights
    m_pub_particle_set = nh.advertise<localization::ParticleSet>("/localization/particle_set", 10);
    // advertise mean possition with confidence
    m_pub_mean_pose_conf_stamped = nh.advertise<localization::MeanPoseConfStamped>("/localization/mean_pose_conf_stamped", 10);
      
    m_odomPose.setZero();
    m_mean.setZero();
    m_confidence = 1.0;
    m_lastConfidence = 1.0;

    m_minEffectiveSampleSizeRatio = 0.5f; //0.5f;
    m_uniformReplacementProbability = 0.05f;
    m_actualUniformReplacementProbability = m_uniformReplacementProbability;
    m_usingActualReplacement = false;
    m_poseReplacementProbability = 0.05f;

    m_odomTimestamp = ros::Time(0);

    is_vision_detection = ros::Time::now();

    waist_odom_yaw = 0;
    is_lifted_before = false;
    is_lifted_now = false;
    is_kidnapped = false;
}
```



然后以一定速率进行主循环，这里设置的2Hz，这是因为udp是以0.5秒为周期进行更新的（由于机器人每走一步才会发出新的数据，根据步态组的定义，新数据是指预计会达到的方位，注：相对初始位置的方位），虽然粒子滤波本身的速率可以达到20Hz，但是不能够使用。

```c++
int main(int argc, char** argv)
{
    ros::init(argc, argv, "particle_filter_localization");
    ros::NodeHandle nh("~");
    LocalizationPF pfl(nh);
    ros::Rate rate(2); 
    while(ros::ok())
    {
        ros::spinOnce();
        pfl.newStep();
        rate.sleep();
    }
    return 0;
}
```

不能够使用20Hz的原因是：

​	里程计信息的发送频率是100Hz，但是实际上，这个msg更新的速率是2Hz，而视觉信息的实际刷新频率是10~15Hz，而粒子滤波PF的刷新频率在20Hz。因此，过高的刷新频率会导致，针对t时刻的里程计，接收到了晚于t时刻的视觉观测信息，这将会导致粒子的权重分配错误（更加靠前）。那么在下一个里程计信息到达时，粒子将同样前移这个距离，就会超过了真实位置，此时，又接收到了此刻的视觉信息，那么定位又会后退，因此会出现震荡，这种震荡如果过于严重，就会导致粒子滤波的混乱。

因此，需要在此处根据里程计频率设置合适的刷新频率。

![](/Users/lukai/Documents/localization_src/8.png)



### 可视化

- 实现了里程计定位和粒子滤波定位的结果显示，能够显示机器人的方向和位置。

- 实现了landmark点的标记，vision组发送的消息中，包含的是landmark相对于机器人的方位，因此，根据里程计定位位置和粒子滤波定位位置分别绘制了landmark点，便于比较两种方法的准确度。

- 标记了障碍物（对方机器人、handler）和球的位置。

- 需要注意的是，可视化节点使用的是OpenCV3进行绘图，所以坐标系需要进行y方向的转换（相反的）。例如：

  ```c++
  // Drawing ball with orange
  if (viz_output_data.bBallWasSeen) //it could have been seen but the position may not have been transformed so it can be using the initial (0,0)
  {
    cv::circle(cv_ptr->image, 
               cv::Point((viz_output_data.ballCenterOnField.x+1+(field->length()/2))*100,
                         (-viz_output_data.ballCenterOnField.y+1+(field->width()/2))*100),
               11, cv::Scalar(0,140,255),-1);
  }
  ```

  

### 参数管理

- 地图模型，根据不同的规格，导入不同模型，例如：

  ```c++
  field_type = "adultsize18";
  if(field_type == "tsinghua")
  {
    m_length = 6.0;
    m_width = 4.0;
    m_centerCircleDiameter = 1.2;
    m_goalWidth = 2.7;
    m_goalAreaWidth = 3.0;
    m_goalAreaDepth = 0.6;
    m_penaltyMarkerDist = 1.8;
  }
  ```

- 消息类型，值得注意的是，这里需要对于ROS系统的msg比较熟悉，如果需要更改消息类型，需要检查使用了它的地方。

- topic控制，如果出现了缺少topic或者需要转换的情况，可以自行编写node进行转换。

- launch文件管理。最新的版本已经对于launch文件做了极大的简化，最为简明的版本为`localization_testing.launch`。

## 四、教程

定位模块的使用教程，将在后续更新。@[DeerKK](https://deerkk.github.io/)

## 五、版本历史

2014.04       第一版本开发完成，初步使用。

2018.09       版本情况未知，为历年定位组使用。

2019.06       增加了转换节点，将/odom广播转换到/udp

2019.07       修复了粒子初始化失败的bug，修复了视觉时间与里程计信息不同步的bug，增加了第二帧复用的功能，改进了可视化程序的显示效果。

## 六、谁使用PFL?

在Robocup2019的比赛中，Tsinghua Hephaestus代表队获得Adult Size比赛的季军，其中的定位模块使用了PFL。从2014年起，PFL定位模块一直配置在Tsinghua Hephaestus中。

## 七、贡献者

本文档主要参考源代码、Wikipedia，感谢Tsinghua Hephaestus Robotics指导老师@清华大学自动化系赵明国教授，交叉定位组指导老师@Rosy，源代码主要开发者@Sotirios Stasinopoulos，以及往届和今年的定位组同学。
