# **Exercise: Braitenberg**
<img src="./assets/images/dtlogo.png" alt="Duckietown" height="60em" style="margin-top:-75px; display: block; float: right">


# Preliminaries

**NOTE:** All commands below are intended to be executed from the root directory of this exercise (i.e., the directory containing this README).

## 1. Make sure your exercise is up to date

Update your exercise definition and instructions,

    git pull upstream mooc2022


## 2. Make sure your system is up to date

- 💻 Always make sure your Duckietown Shell is updated to the latest version. See [installation instructions](https://github.com/duckietown/duckietown-shell)

- 💻 Update the shell commands: `dts update`

- 💻 Update your laptop/desktop: `dts desktop update`

- 🚙 Update your Duckiebot: `dts duckiebot update ROBOTNAME` (where `ROBOTNAME` is the name of your Duckiebot chosen during the initialization procedure.)


# Work on the exercise

### Launch the code editor

Open the code editor by running the following command from the root of the `braitenberg` exercise.

```
dts code editor
```

Wait for a URL to appear on the terminal, then click on it or copy-paste it in the address bar 
of your browser to access the code editor. The first thing you will see in the code editor is 
this same document, you can continue there.


### Walkthrough of notebooks

Inside the code editor, use the navigator sidebar on the left-hand side to navigate to the 
`notebooks` directory and open the first notebook.

Follow the instructions on the notebook and work through the notebooks in sequence.


### 💻 Testing in simulation

To test in simulation, use the command

    $ dts code workbench --sim

There will be two URLs popping up to open in your browser: one is the direct view of the experiment
(probably `http://localhost:8090`).
The other is VNC and not useful for this exercise. Ignore it.

This simulation test will be very slow! We suggest opening the simulation viewer and enjoying a 
cup of tea/coffee while your agent does its job. Monitor how it acts! 
You might get some ideas on how to fix your matrices or the agent.py.

This simulation test is just that, a test. Don't trust it fully. If you want a more accurate 
metric of performance, continue reading to the `Do local evaluations` section below.


### 🚙 Testing on the robot

You can test your agent on the robot using:

    dts code workbench --duckiebot YOUR_DUCKIEBOT

This is the modality "everything runs on the robot".

You can also test using

    dts code workbench --duckiebot YOUR_DUCKIEBOT --local 

This is the modality "drivers running on the robot, agent runs on the laptop."


## Phase 3: Make a submission

At the end, to submit your homework, you should submit your agent using

    dts code submit

The robot should travel at least an average of 2 meters from the starting point.


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

### Happy life all together in harmony

We run the cloud evaluation service based on limited resources from our universities 
and sponsorships. To avoid a denial-of-service-by-many-exercises-submissions attack, 
we implemented a priority queue: the more submissions you send, the lower priority you have 
and it will take more time to do them. 
This is fair in the sense that, if you have never submitted, you are on top of the queue. 
However, we try to evaluate all submissions.

We hope this works and we don't need more stringent policies.

You can help us in the following ways.


### Retire obsolete submissions

Note that you can "retire" submissions that you know are wrong.
You can do this through [the Duckietown Challenges website](https://challenges.duckietown.org/).


### Do local evaluations

# TODO: this is not supported right now as the `submission.yaml` file is stored in the recipe.

We suggest you evaluate your work using the command

    dts challenges evaluate

This should take a few minutes. This is not supposed to be an interactive process: just let it run,
and when you return, you will find the output in a folder, including videos, and trajectories,
and all the statistics you find on the website.
