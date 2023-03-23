# **Learning Experience (LX): Linux and Networking**

# About these activities

In this learning experience you will learn the basics of Linux, the operating system running on Duckiebots and Duckiedrones, and get an introduction to networking.


It is comprised of two parts: 

1. Introduction to Linux

1. Networking

Please complete all parts of this learning experience.
You can do this learning experience in the ssh shell of your Duckiebot or Duckiedrone, or on
any unix machine with Python.  

The networking component of this learning experience will help you understand
how to communicate with your drone. Fundamentally, robots are
computers that are linked through networks. In robotics, accounting
for networking allows both more robust and more efficient design. The
networking part of this learning experience describes how to use basic
networking with a focus on concepts most useful to robotics.

Networking may not seem like a topic in robotics, but it is one of the
most common reasons robots fail to work.  If you cannot connect your
base station to the robot, you cannot see the robot's status; you
cannot see sensor output; you cannot send actuation commands. Moreover
networks in the wild can be set up in a variety of diverse ways that
may or may not allow your base station to connect to your robot. For
example, some guest networks do not allow peer-to-peer
connections, so even if you get your base station and the robot
connected on that network, you still cannot talk to the robot.

As a result, it is essential to be familiar with basic networking
concepts in order to make your drone, or any robot, work.  This unit
asks you to think and learn about some networking concepts. We also
cover helpful Linux commands.

This learning experience is provided by the Duckietown team. Visit us at the 
[Duckietown Website](https://www.duckietown.com) for more learning materials, documentation, and demos.

For guided setup instructions, lecture content, and more related to this LX, watch [these videos](https://learning.edge.edx.org/course/course-v1:BrownX+CS195R+2018_T1/block-v1:BrownX+CS195R+2018_T1+type@sequential+block@56017f6d3048461b90466ad229ac8df6) on EdX.

## Grading

When you are done, use [this link](https://classroom.github.com/a/i0KBPRjv) to create your assignment Github Repo.

Repo should include the following files that you will create during the activity:

- `hello1.py`, `hello2.py`, `hello.sh`, `my_cat.py`, `my_reverse_sort.py`, `my_sort_status.py`
- `shell.txt`, `networking.txt` 

# Instructions

**NOTE:** All commands below are intended to be executed from the root directory of this exercise (i.e., the directory containing this README).


## 1. Make sure your exercise is up-to-date

Update your exercise definition and instructions,

    git pull upstream duckiedrone-lxs

**NOTE:** Example instructions to fork a repository and configure to pull from upstream can be found in the [duckietown-lx repository README](https://github.com/duckietown/duckietown-lx/blob/mooc2022/README.md).


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

**NOTE**: the instructions below **do not apply** for these learning experience.

### 💻 Testing in simulation

To test in simulation, use the command

    $ dts code workbench --sim

There will be two URLs popping up to open in your browser: one is the direct view of the
simulated environment. The other is VNC and only useful for some exercises, follow the instructions
in the notebooks to see if you need to access VNC.

This simulation test is just that, a test. Don't trust it fully. If you want a more accurate
metric of performance, continue reading to the `Perform local evaluation` section below.


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
