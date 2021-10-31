# RL basics

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
