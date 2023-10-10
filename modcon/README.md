<p align="center">
<img src="./assets/images/dtlogo.png" alt="Duckietown Logo" width="50%">
</p>

# **Learning Experience (LX): Modeling and Control (ModCon)**

# About these activities

In this learning experience, you will progress through a set of activities on the topic of the modeling and control 
(`ModCon`) of a differential drive robot. This includes calculating odometry, visualizing the estimation of your 
agent's pose in the world, and tuning your own PID controller.

This learning experience is provided by the Duckietown team and can be run on Duckiebots. Visit us at the 
[Duckietown Website](https://www.duckietown.com) for more learning materials, documentation, and demos.

For guided setup instructions, lecture content, and more related to this LX, see the [Self Driving Cars with 
Duckietown course on EdX](https://learning.edx.org/course/course-v1:ETHx+DT-01x+3T2022/home).

# 💻 🚙 About this learning experience

We include `activities` and one `exercise`.

- Activies are designed as guided tutorials and solutions are publicly available. 

- Exercises are designed as "do it yourself", and solutions are not publicly available. 

  - Exercises have associated "challenges", to which solutions (agents) are submitted for evaluation. Duckietown challenges are available on our [challenges server][challenges-server].

[challenges-server]: https://challenges.duckietown.org/v4/humans/challenges

## Evaluation

Submissions to the exercise of this learning experience will be sent to the [`lx22-modcon`][challenge] challenge.

You can verify the performance metrics of your submitted agents on the [Challenge Leaderboard][leaderboard]. 

Evaluations are computationally intensive and performed on a best-effort basis. We deprioritize evaluating submissions from users that have already submitted in favor of those who have not. 

To ensure priority evaluation services, please reach out to Duckietown at info@duckietown.com. 

[challenge]: https://challenges.duckietown.org/v4/humans/challenges/lx22-modcon
[leaderboard]: https://challenges.duckietown.org/v4/humans/challenges/lx22-modcon/leaderboard


# Instructions

**NOTE:** All commands below are intended to be executed from the root directory of this exercise (i.e., the directory containing this `README`).


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

There will be two URLs popping up to open in your browser: one is the direct view of the
simulated environment. The other is VNC and only useful for some exercises, follow the instructions
in the notebooks to see if you need to access VNC.

This simulation test is just that, a test. Don't trust it fully. If you want a more accurate
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

In this mode "everything runs on the robot".

You can also test using

    dts code workbench --duckiebot YOUR_DUCKIEBOT --local 

In this mode instead "drivers run on the robot, agent runs on the laptop.". When testing like this you should expect reduced latency from computation (your computer is likely more powerful than the Duckiebot's onboard computer) but increased latency from network (as data and commands need to be shipped back and forth). Generally speaking the performance of this mode should be better than runnning the agent on the robot for computationally intensive agents, but this mode suffers from environmental factors (network quality and use).


### 📽 Perform local evaluation

We suggest you evaluate your work locally before submitting your solution.
You can do so by running the following command,

    dts code evaluate

This will take a few minutes and it is not supposed to be an interactive process: just let it run, and when you return you will find the output in a folder, including videos, and trajectories, and all the statistics you would usually find on the website.


### 📬 Submit your solution

When you are ready to submit your agent solution, use the following command:

    dts code submit

This will package all your code and send it to the Duckietown [challenges server][challenges-server] for evaluation.


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

You can do so by logging in on the [challenges server][challenges-server] using your token, finding the submission you want to retire from the list of submission in your user profile page, and using the "retire" button to the right of the submission record line.
