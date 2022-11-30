# **Duckietown LX - MOOC Exercises**

<img src="./assets/images/dtlogo.png" alt="Duckietown" height="60em" style="margin-top:-75px; display: block; float: right">

# About this repository

This is an exercises repository to support the Duckietown MOOC.

# Setup

In order to use these exercise, you should complete the following steps:

## Step 0 - Requirements

We assume here that you have already set up your Duckietown development environment following the steps in [Unit C-1 
and C-2](https://docs.duckietown.org/daffy/opmanual_duckiebot/out/laptop_setup.html) of the Duckietown operation manual.

## Step 1 - Installation

Start by installing a new dependency,

    sudo apt install libnss3-tools

Then update your Duckietown shell and shell commands,

    pip3 install -U duckietown-shell

    dts update

## Step 2 - SSL certificate

Next, set up your local SSL certificate needed to run the exercises' editor,

    dts setup mkcert

## Step 3 - Fork this repository

In order to store your own code, while also keeping the ability to pull updates from our version of this repo, you 
need to create your own fork.

Start by pressing "Fork" in the top right corner of this repository page on GitHub. You will be able to create a new 
fork: `<your_username>/duckietown-lx`

Then clone your new repository, replacing your GitHub username in the address below,

    git clone -b mooc2022 git@github.com:<your_username>/duckietown-lx

## Step 4 - Set up the remote

Now we will configure the Duckietown version of this repository as the upstream repository to sync with your fork.

List the current remote repository for your fork,

    git remote -v

Specify a new remote upstream repository,

    git remote add upstream git@github.com:duckietown/duckietown-lx

Confirm that the new upstream repository was added to the list,

    git remote -v

You can now push your work to your own repository using the standard GitHub workflow, and the beginning of every 
exercise will prompt you to pull from the upstream repository - updating your exercises to the latest Duckietown 
version,

    git pull upstream mooc2022

## What next?

You will find the following set of instructions in every individual exercise directory in this repository. This 
workflow will allow you to build and test your code, run solutions in simulation and on the robot, and submit your 
work to the challenges server to evaluate it against other developers.

# Instructions

**NOTE:** All commands below are intended to be executed from the root directory of a single exercise (i.e., the 
`braitenberg` directory).

## 1. Make sure your exercise is up-to-date

Update your exercise definition and instructions,

    git pull upstream mooc2022


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

    dts code workbench --sim

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
