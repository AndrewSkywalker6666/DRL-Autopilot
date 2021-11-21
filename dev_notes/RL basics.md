# RL basics from Morvan

## 方法归类

1. 对环境是否理解
   1. 不理解环境 model-free
   2. 理解环境 model-based
2. 是否基于概率
   1. 是：决策时选择动作都有一定概率（能够处理连续决策）
      1. policy gradients
   2. 否：决策时选量化最高的动作（基于价值）
      1. Q
      2. Sarsa
   3. 混合：actor-critic
      1. 基于概率做出动作
      2. 并给出动作的价值
3. 更新事件
   1. 回合：monte-carlo update（要等待每一个游戏回结束）
   2. 单步：每一步都更新（现在基本都用这个）
4. 是否在线
   1. 在线：边玩边学
      1. Sarsa
   2. 离线：储存再处理
      1. Q

## 有用的资料

1. numpy pandas for data analysis
2. matplotlib for data visualization
3. tkinter/gym for UI and simulation
4. tf for integration with NN
5. https://spinningup.openai.com/en/latest/algorithms/sac.html
6. https://www.youtube.com/playlist?list=PLySQw_vQ73PyDY68KF0HdCzcILBoHVTvD
7. https://www.youtube.com/playlist?list=PLvOO0btloRnsiqM72G4Uid0UWljikENlU

# From AI textbook

## basics

1. learn from trying and reward

   * sparse means reward is only associated with the final outcome
   * for simulated envs, as long as the reward engineering is decent, rl is a general way to build AI/robots

2. agent are not given the MDP as a problem but actuatlly in the MDP

3. model-based and model free

   * model-based means there is a transition model to help with interpreting reward signals and making decisions; outcome is usually the utility function U, defined in terms of the sum of rewards from one state onwards

   * **model-free** means transition model is DONT-CARE

     * action-utility: e.g. Q learning, **quality function Q(s,a)=sum of rewards onwards if action a is taken**, choose the highest q action

       > deep q learning uses a NN to represent quality function

     * **policy search: learn a policy pi(s) that maps states to action**

4. pasive and active

   * passive: policy is fixed and learn the utilities of states or environment
   * **active: policy should be learned, exploration is required**

5. supporting methods

   * inductive learning, deep learning to speed up learning process
   * pseudorewards to guide the learner
   * apprenticeship learning, use demonstrations rather than reward

## policy search

1. idea: twiddle policy parameter as long as performance improves
2. if policy func is discontinuous, people often use a stochastic policy representation, e.g. softmax
3. optimizing policy
   * closed form: policy gradient
   * empirical gradient, hill climbing

# From DRL-Autopilot study list

## 网上博客的对比

1. DDPG源于DQN，是DQN解决连续控制问题的一种方法。然而**DQN存在过估计问题，DDPG也是如此**。为了解决过估计的问题，Hasselt提出了Double Q Learning方法，将此方法应用到DQN中，就是Double DQN，即**DDQN**。TD3则对DDPG做了改进，解决了DDPG的过估计问题，顺便增加了稳定性
2. PPO算法是目前最主流的DRL算法，但是PPO是一种on-policy算法，存在sample inefficiency的缺点，需要巨量的采样才能学习。DDPG及其拓展是面向连续控制的off-policy的算法，相对于PPO来说更sample efficient，但是它存在对其超参数敏感，收敛效果差的问题。SAC算法是面向最大熵强化学习开发的一种off-policy算法。与DDPG相比，SAC使用的是随机策略，相比确定性策略具有一定的优势。
3. SAC会同时学习策略（一个策略网络）+两个Q函数（两个Q网络）

> q-learning（加上网络后变成）deep q network，DQN（适配连续控制变成）deep deterministic policy gradient，DDPG（解决过估计问题变成）twin delayed ddpg，TD3
>
> 或者DDPG（采用随机策略后）soft actor-critic，SAC

# RL in 3h by Nicholas Renotte

## conda setup

* 可以自定义安装目录，主要用于把不同项目的环境分割开，防止需要的软件版本冲突
* 安装的时候会在bashrc里面加东西
* 添加环境`conda create -n 环境名称 --clone 环境名`
* 激活环境`conda activate 环境`
* 打开notebook`ipython3 notebook`

## gym concept

* gym env are represented by spaces, which model the observation and action

  * box
  * discrete
  * tuple
  * dict
  * multibinary
  * multidiscrete

* document about different algorithms

  * spinningup.openai.com
  * stable-baseline3.readthedocs
  * discrete single process dqn
  * discrete multi process ppo a2c
  * continuous single process sac td3
  * continuous multi process ppo a2c
  
* install cuda

  * https://developer.nvidia.com/cuda-toolkit-archive
  * use runfile
  * do not select driver
  * https://docs.nvidia.com/cuda/cuda-toolkit-release-notes/index.html <- driver cuda compatibility table
  * export path to bashrc

  ```
  ===========
  = Summary =
  ===========
  
  Driver:   Not Selected
  Toolkit:  Installed in /usr/local/cuda-11.3/
  Samples:  Installed in /home/yueqian/
  
  Please make sure that
   -   PATH includes /usr/local/cuda-11.3/bin
   -   LD_LIBRARY_PATH includes /usr/local/cuda-11.3/lib64, or, add /usr/local/cuda-11.3/lib64 to /etc/ld.so.conf and run ldconfig as root
  
  To uninstall the CUDA Toolkit, run cuda-uninstaller in /usr/local/cuda-11.3/bin
  ***WARNING: Incomplete installation! This installation did not install the CUDA Driver. A driver of version at least 465.00 is required for CUDA 11.3 functionality to work.
  To install the driver using this installer, run the following command, replacing <CudaInstaller> with the name of this run file:
      sudo <CudaInstaller>.run --silent --driver
  
  Logfile is /var/log/cuda-installer.log
  ```

  
