# **Learning Experience (LX): DD21 SLAM**

![Duckietown logo](./assets/_images/dtlogo.png)

## About this learning experience

In this learning experience you will learn how to build a map of an unknown environment and simultaneously localize your Duckiedrone in it.

This learning experience is provided by the Duckietown team and can be run on Duckiedrones.
Visit us at the [Duckietown Website](https://www.duckietown.com) for more learning materials, documentation, and demos.

For guided setup instructions, lecture content, and more related to this LX, see:

* [SLAM](https://learning.edge.edx.org/course/course-v1:BrownX+CS195R+2018_T1/block-v1:BrownX+CS195R+2018_T1+type@sequential+block@ace5da8044c24272bed236e84530ebc6/block-v1:BrownX+CS195R+2018_T1+type@vertical+block@8774ab97d62c437ab9e2052dae76bd92)

# Activity theory part

Before you start working through this activity, you should read through the following sections:

- [Background](./notebooks/00-theory/00-background.md)
- [Introduction](./notebooks/00-theory/01-implementation.md)

**Important:** when you first open these documents a security pop-up will appear in the top right corner; click on it and select `Disable` in the menu that appears. This will allow you to visualize the theory content.

# Instructions

**NOTE:** All commands below are intended to be executed from the root directory of this exercise (i.e., the directory containing this `README`).

## 1. Make sure your exercise is up-to-date

Update your exercise definition and instructions,

    git pull upstream duckiedrone-lxs

**NOTE:** Example instructions to fork a repository and configure to pull from upstream can be found in the [duckietown-lx repository README](https://github.com/duckietown/duckietown-lx/blob/mooc2022/README.md).

## 2. Make sure your system is up-to-date

- 💻 Always make sure your Duckietown Shell is updated to the latest version. See [installation instructions](https://github.com/duckietown/duckietown-shell)

- 💻 Update the shell commands: `dts update`

- 💻 Update your laptop/desktop: `dts desktop update`


## 3. Work on the exercise

Use this [link](https://classroom.github.com/a/HF1mpBKW) to generate a Github repo for this project.

To clone your repository follow these steps:

1. *From the terminal* of your base station go inside the root of this learning experience and move in the directory `packages`:

    cd lx/packages/

1. Clone your assignment's repository by running the following command and following the instructions that pop up (remember to change `<yourGithubUsername>`):

  ```bash
  git clone https://github.com/h2r/project-localization-slam-2019-<yourGithubUsername>.git
  ```

You can now access your project in the sidebar by going in the folder `packages`.

When you need to modify the code of an exercise and test it on the drone, do so by working on your base station inside this editor, committing the modified file and then pushing it to the remote repo.

Make sure to commit the changes and push them to GitHub each time you modify some file.

You can do so by using the Version Control tab in VSCode:

![VSCode version control](./assets/_images/vscode_source_control.png)

### Launch the code editor

Open the code editor by running the following command on your base station:,

    dts code editor

Wait for a URL to appear on the terminal, then click on it or copy-paste it in the address bar
of your browser to access the code editor. The first thing you will see in the code editor is
this same document, you can continue there.

### Walkthrough of notebooks

**NOTE**: You should be reading this from inside the code editor in your browser.

Inside the code editor, use the navigator sidebar on the left-hand side to navigate to the
`notebooks` directory and open the first notebook.

Follow the instructions on the notebook and work through the notebooks in sequence.

## Working on the drone

When you need to modify the code of an exercise and test it on the drone, do so by working on your base station inside this editor, committing the modified file and then pushing it to the remote repo.

To clone your repository on your drone follow these steps:

1. *From the terminal* of your base station connect through `ssh` to your drone and, inside screen, go inside the directory `~/catkin_ws/src/`.

1. Clone your assignment's repository by running the following command and following the instructions that pop up (remember to change `<yourGithubUsername>`):

    git clone https://github.com/h2r/project-localization-slam-2019-<yourGithubUsername>.git

### How to get your changes on the Duckiedrone

To get the changes from your repo to the Duckiedrone, pull the updates on the drone from inside the container, by running:

    cd ~/catkin_ws/src/project-localization-slam-2019-<yourGithubName>
    git pull

### 💻 Testing in simulation

To test in simulation, use the command

    dts code workbench --sim

There will be a URL popping up to open in your browser: it is a VNC environment which you can access from your browser.

## Deliverables

Please be sure to push your finished project directory to GitHub classroom to hand-in. For your final hand-in, you should have edited all the following files:

* `student_slam_helper.py`
* `student_localization_helper.py`
* `student_compute_displacement.py`
* `student_particle_filter.py`

## Hand-in

Commit and push your changes before the assignment is due. This will allow us to access the files you pushed to GitHub and grade them accordingly. If you commit and push after the assignment deadline, we will use your latest commit as your final submission, and you will be marked late.

Note that assignments will be graded anonymously, so please don't put your name or any other identifying information on the files you hand in.

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

    dts challenges config --docker-username <USERNAME> --docker-password <PASSWORD>
