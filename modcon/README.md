<p align="center">
<img src="./assets/images/dtlogo.png" alt="Duckietown Logo" width="50%">
</p>

# **Activity: Modeling and Control (ModCon)**

# About these activities

This activity focuses on modeling and control of a differential drive robot. Provided in this activity are multiple notebooks each focusing on essential aspects of how such 
robots. Staring with:

## 1. Representations:

This task involves understanding how to define and represent the state of a robot, with a particular focus on the concept of "pose," (description of the robot's position 
and orientation in space).

## 2. Wheel-Calibration: 

Wheel- calibration focuses on calibrating the motors and wheels of Duckiebots to ensure they move straight when commanded and reduce wheel slippage. 
The calibration involves determining two key parameters: gain and trim

## 3. Wheel-Encoders-Tutorial:

It's important to understand how wheel encoders work in Duckiebots; these are sensors that convert the angular position or motion of a wheel's shaft into digital signals 
called "ticks"; which represent incremental changes in the wheel's angular position.

## 4. Odemetry:

This text explains the concept of odometry in robotics, which involves measuring the robot's path and estimating its pose (position and orientation) over time. The process 
uses wheel encoder data and a "dead-reckoning" model, following an iterative procedure to update the robot's pose.

## 5. PID Control:

The PID (Proportional-Integral-Derivative) controller, is the decision-making part of the robot, determining the commands sent to the robot's actuators to achieve specific goals.
For a Duckiebot, the controller's outputs are the linear and angular velocities, which difines the robot's speed and steering.


## Challenges of this activity:

1. Lot of the concept in this activity was new and hard to comprehend
2. Simultion values differed from those of the duckiebot
3. I experienced constant difficulties connecting and moving the duckiebot
4. Each change in one of the parameters during testing required a new build and run on both the simulation environment and duckiebot
   



# Instructions

### 1. Making sure the system is up-to-date

- 💻 Always make sure your Duckietown Shell is updated to the latest version. See [installation instructions](https://github.com/duckietown/duckietown-shell)

- 💻 Update the shell commands: `dts update`

- 💻 Update your laptop/desktop: `dts desktop update`

- 🚙 Update your Duckiebot: `dts duckiebot update duckiebot005` 



### 2. Launch the code editor

Open the code editor by running the following command,

```
dts code editor
```

Wait for a URL to appear on the terminal, then click on it or copy-paste it in the address bar
of your browser to access the code editor. The first thing you will see in the code editor is
this same document, you can continue there.



### 3.  💻 Testing in simulation

To test in simulation, use the command

    $ dts code workbench --sim

There will be two URLs popping up to open in your browser: one is the direct view of the
simulated environment. The other is VNC and only useful for some exercises, follow the instructions
in the notebooks to see if you need to access VNC.



### 4.  🚙 Testing on a physical robot

You can test your agent on the robot using the command,

    dts code workbench --duckiebot duckiebot005

In this mode "everything runs on the robot".

