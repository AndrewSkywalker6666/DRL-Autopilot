## sep26

* 在做了在做了（新建文件夹

## sep27

* 正在读dev_notes里面的相关paper

## oct1

* 读了哪几篇
  * 0 Control of a Quadrotor with Reinforcement Learning
  * 1 Flying through a narrow gap using neural network: an end-to-end planning and control approach
  * 2 Flying Through a Narrow Gap Using End-to-end Deep Reinforcement Learning Augmented with Curriculu Learning and Sim2Real

* 明白了
  * 实现的功能
  * 性能参数
  * 整体设计实现方案
* 忽略了
  * RL问题的设计
  * 奖惩函数
  * 网络的训练细节

## oct8

* 第一次汇报/讨论
* 第一部分任务明细：在gazebo实现用DRL控制器的验证

## oct12

* 把px4和ros空间设置好了，用最新的稳定版

* 分步骤走

  1. 建立通讯

      * FC->CC：需要px4的状态估计结果，已在mavros中原生有

      * CC->FC：需要actuator cmd override，需要包括一个flag和四个值

  2. 电机输出的MUX机制

  3. frequencies

  4. 思考怎么用gazebo

* 建立CC到FC的通讯

  1. 最终还是跳入了ros2的坑，https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html，教程见https://docs.ros.org/en/foxy/Tutorials.html

## oct14

* 调通了ros2-px4的pipeline
* 这样子自定义消息就不用管mavlink了，只需要uorb以及`msg/tools/`下的`yaml`就好了，修改了`yaml`文件之后需要拷贝到ros2那边并重新编译，见脚本。每次第一次编译总会出现`stderr`第二次就没问题，很迷
* `micrortps_agent -t UDP`在ros那边重新编译了之后才会刷新
* 接下来：
  * 仿照example写sub和pub
  * 把相关的教程读完，特别是offboard的例子，思考是否真的需要新建一个topic

## oct30

* 在master的stable release上重新调通了PX4-ROS2的通信，并且创建了一个controller的包，等待训练好的网络放进去

## oct31

* 和学长聊了一下，明确了用的算法和训练步骤

  > 需要的数据是融合后的数据，使用的也是policy和value网络架构，但是用的是更新的RL算法，SAC，TD3，PPO，NN用的是tensorflow
  >
  > 目前是在gym框架下，以gazebo为仿真器做训练，每一个回合大致包括{以任意姿态把飞机丢进去，等待一段时间结束，更新参数}，回合间要重置模型位置，以及飞控系统的重置，因为需要ekf的融合
  
* 接下来做的东西

  * 先把相关算法和gym看一遍儿
  * 学一下范例代码
  * 整合到自己的架构中（19年的那篇drl synthesis的phd学位论文可以看看有没有灵感）
  * 想好训练的pipeline，开训
  * 把tensorflow的网络放到drl-controller中