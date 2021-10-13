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