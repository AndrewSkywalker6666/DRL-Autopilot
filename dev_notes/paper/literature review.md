# LR

​      Traditional flight controllers often use PID controllers to control UAVs due to the simple implementation and overall good performance [1].

​      With the advance of perception and planning technologies [2, 3, 4], the demand of being able to track a given trajectory is not trivial. Many researchers propose different types of the model predictive controller (MPC) to tackle this problem and they work well [5, 6, 7].

​      Some researchers also come to realize that the optimization-based nature of MPC can be combined with planning [8, 9] or perception [10]. They are essentially formulating "intelligence" into a single optimization problem, which is a type of unification. Another type of unification is unified planning and perception [11].

​      To take the idea of the "unified approach" a step further, Loquercio et al trained a direct sensor-to-motor network to complete perception, planning, and control in a simulator trained network [12]. This work shows better performance of the unified network in terms of the high success rate of navigation and less I/O latency. While the success rate of navigation is amazing, the trajectory is not optimal as it tends to wobble even if it can just fly straight. Though this network already outperformed many state-of-the-art approaches, notice the performance drop at high speeds. It is still challenging to push the limits, especially in a more predictable way in terms of trajectory shape. Also, the training process is done by privileged learning, which requires an expert model to imitate.

​      Neural networks seen in most works are essentially imitations of the traditional pipeline [12, 13]. Xiao et al were trained a planner without privileged information through curriculum learning and reinforcement learning [14]. Their results didn't show outstanding performance in terms of accuracy and agility, partly due to the lack of simulation fidelity and the poor tracking performance of the PID controller, yet this work shows the potential of training an end-to-end network without privileged information.

​      In recent years, ROS2 is gaining popularity because of its faster inter-process data transfer [15], with ROS2 the process latency and compound error can be further reduced.

​      [1] Control Algorithms for UAVs: A Comprehensive Survey, EAI 2020

​      [2] Autonomous Drone Racing with Deep Reinforcement Learning, IROS 2021

​      [3] Time-Optimal Planning for Quadrotor Waypoint Flight, SR 2019

​      [4] Whole-Body Real-Time Motion Planning for Multicopters, Arxiv 2021

​      [5] Learning-Based Model Predictive Control on a Quadrotor: Onboard Implementation and Experimental Results, ICRA 2012

​      [6] Data-Driven MPC for Quadrotors, RAL 2021

​      [7] Performance, Precision, and Payloads: Adaptive Nonlinear MPC for Quadrotors, Arxiv 2021

​      [8] Fast Nonlinear Model Predictive Control for Unified Trajectory Optimization and Tracking, ICRA 2016

​      [9] Model Predictive Contouring Control for Near-Time-Optimal Quadrotor Flight, Arxiv 2021

​      [10] PAMPC: Perception-Aware Model Predictive Control for Quadrotors, IROS 2018

​      [11] RAPTOR: Robust and Perception-aware Trajectory Replanning for Quadrotor Fast Flight, TRO 2021

​      [12] Learning High-Speed Flight in the Wild, SR 2021

​      [13] Flying through a narrow gap using neural network: an end-to-end planning and control approach, IROS 2019

​      [14] Flying Through a Narrow Gap Using End-to-end Deep Reinforcement Learning Augmented with Curriculum Learning and Sim2Real, TNNLS 2021

​      [15] Exploring the performance of ROS2, ICES 2016
