<p align="center">
<img src="./assets/images/dtlogo.png" alt="Duckietown Logo" width="50%">
</p>

# **Learning Experience (LX): State Estimation**

# About these activities

In this learning experience, we will implement a few state estimators.
The first two notebooks have simple pedagogical examples of the `Kalman` and `particle filters`.
The third notebook contains an implementation of the `histogram filter` in duckietown with some parts in
the code to be completed as part of the exercise.

This learning experience is provided by the Duckietown team and can be run on Duckiebots. Visit us at the 
[Duckietown Website](https://www.duckietown.com) for more learning materials, documentation, and demos.

For guided setup instructions, lecture content, and more related to this LX, see the [Self Driving Cars with 
Duckietown course on EdX](https://learning.edx.org/course/course-v1:ETHx+DT-01x+3T2022/home).


## Grading challenge

Your submissions will be sent to the [`lx22-state-estimation`][challenge] challenge.

You can verify the scores of your submissions on the [Challenge Leaderboard][leaderboard] after your submission is evaluated.

[challenge]: https://challenges.duckietown.org/v4/humans/challenges/lx22-state-estimation
[leaderboard]: https://challenges.duckietown.org/v4/humans/challenges/lx22-state-estimation/leaderboard



# Instructions

**NOTE:** All commands below are intended to be executed from the root directory of this exercise (i.e., the directory containing this README).


## 1. Make sure your exercise is up-to-date

Update your exercise definition and instructions,

    git pull upstream mooc2022

**NOTE:** to pull from upstream, you need to have completed the instructions in the [duckietown-lx repository README](https://github.com/duckietown/duckietown-lx/blob/mooc2022/README.md) to *fork* this repository.


## 2. Make sure your system is up-to-date

- 💻 Always make sure your Duckietown Shell is updated to the latest version. See [installation instructions](https://github.com/duckietown/duckietown-shell)

- 💻 Update the shell commands: `dts update`

- 💻 Update your laptop/desktop: `dts desktop update`

- 🚙 Update your Duckiebot: `dts duckiebot update ROBOTNAME` (where `ROBOTNAME` is the name of your Duckiebot chosen during the initialization procedure.)


## 3. Work on the exercise

### Launch the code editor

Open the code editor by running the following command,

```
dts code editor
```

Wait for a URL to appear on the terminal, then click on it or copy-paste it in the address bar
of your browser to access the code editor. The first thing you will see in the code editor is
this same document, you can continue there.


### Walkthrough of notebooks

**NOTE**: You should be reading this from inside the code editor in your browser.

Inside the code editor, use the navigator sidebar on the left-hand side to navigate to the
`notebooks` directory and open the first notebook.

Follow the instructions on the notebook and work through the notebooks in sequence.


### 💻 Testing in simulation

To test in simulation, use the command

    $ dts code workbench --sim

In this case you should use the link for the VNC environment. It should look something like 

```commandline
================================================================
|                                                              |
|    VNC running at http://127.0.0.1:32768                     |
|                                                              |
================================================================
```

Click on that link (note that the port number 32768 will probably be different but that's ok).

You will see four icons on the desktop. Two are particularly useful at this point. One is the joystick (you can 
double-click and open one of those). When the robot is in "NORMAL_JOYSTICK_MODE," you
can drive it around by making the joystick window become active (you will see a blue circle
with a white arrow when the window is active) and then using your keyboard arrow keys.
**Important:** In order to test your code, you will need to put your robot into "LANE_FOLLOWING" mode, by pressing "A"
on the joystick. You can return to "NORMAL_JOYSTICK_MODE" by pressing "S". 

The other is RQT Image View, which is useful for debugging.
For example, with RQT Image View you can see the robot camera feed. Do so by opening the tool by double-clicking
the icon, and then once it opens use the drop-down bar in the top right to select `/agent/camera_node/image/compressed`.
You may find some of the options interesting also. For example:

 - `/agent/line_detector_node/debug/segments/compressed` shows you an image with all of the line detections,
 - `/agent/ground_projection_node/debug/ground_projection_image/compressed` shows you a top down view of those line 
detections projected onto the ground plane,
 - `/agent/histogram_lane_filter_node/belief_img` is maybe the most informative as you shows you a matrix
that represents the histogram filter that you are implementing.

Also note that this simulation test is just that, a test. Don't trust it fully. If you want a more accurate
metric of performance, continue reading to the `Perform local evaluation` section below.


### ℹ️️ Check Robot Compatibility

While we try our best to support running these exercises on all versions of the Duckiebot, some activities require special hardware and
are only supported on specific robots. Please use this section to ensure the compatibility of the exercise and your
robot.

The support matrix of this exercise is as follows:

| Duckiebot Type   	                                                                                | Configuration 	 | Support Level   	    |
|---------------------------------------------------------------------------------------------------|-----------------|----------------------|
| [DB21-J4](https://get.duckietown.com/products/duckiebot-db21?variant=41543707099311)            	 | Jetson 4GB    	 | ✔️ Full Support    	 |
| [DB21-J2](https://get.duckietown.com/products/duckiebot-db21?variant=40700056830127)            	 | Jetson 2GB    	 | ✔️ Full Support 	    |


### 🚙 Testing on a physical robot

You can test your agent on the robot using the command,

    dts code workbench --duckiebot YOUR_DUCKIEBOT

This is the modality "everything runs on the robot".

You can also test using

    dts code workbench --duckiebot YOUR_DUCKIEBOT --local 

This is the modality "drivers running on the robot, agent runs on the laptop."


### 📽 Perform local evaluation

We suggest you evaluate your work locally before submitting your solution.
You can do so by running the following command,

    dts code evaluate

This should take a few minutes.
This is not supposed to be an interactive process: just let it run, and when you return,
you will find the output in a folder, including videos, and trajectories, and all the statistics
you would usually find on the website.


### 📬 Submit your solution

When you are ready to submit your homework, use the following command,

    dts code submit

This will package all your code and send it to the Duckietown servers for evaluation.


## Troubleshooting


If an error of this form occurs

```bash
Traceback (most recent call last):
  File "/usr/local/lib/python3.8/dist-packages/duckietown_challenges_cli/cli.py", line 76, in dt_challenges_cli_main
    dt_challenges_cli_main_(args=args, sections=sections, main_cmd="challenges")
  File "/usr/local/lib/python3.8/dist-packages/duckietown_challenges_cli/cli.py", line 203, in dt_challenges_cli_main_
    f(rest, environment)
  File "/usr/local/lib/python3.8/dist-packages/duckietown_challenges_cli/cli_submit.py", line 165, in dt_challenges_cli_submit
    br = submission_build(
  File "/usr/local/lib/python3.8/dist-packages/duckietown_challenges_cli/cmd_submit_build.py", line 41, in submission_build
    raise ZException(msg, available=list(credentials))
zuper_commons.types.exceptions.ZException: Credentials for registry docker.io not available
available:
```

you need to log into docker using `dts`. Use this command:

```
dts challenges config --docker-username <USERNAME> --docker-password <PASSWORD>
```


## Retire obsolete submissions

Note that you can "retire" submissions that you know are wrong.
You can do this through [the Duckietown Challenges website](https://challenges.duckietown.org/).

To do so, login using your token, then find the submission you want to retire from the list of submission
in your user profile page. Use the button "retire" to the right of the submission record line.
