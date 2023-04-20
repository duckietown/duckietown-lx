# **Learning Experience (LX): DD21 Localization**

![Duckietown logo](./assets/_images/dtlogo.png)

## About this learning experience

In this learning experience you will learn how to implement Monte Carlo localization on your Duckiedrone.

This learning experience is provided by the Duckietown team and can be run on Duckiedrones.
Visit us at the [Duckietown Website](https://www.duckietown.com) for more learning materials, documentation, and demos.

For guided setup instructions, lecture content, and more related to this LX, see:

* [Localization](https://learning.edge.edx.org/course/course-v1:BrownX+CS195R+2018_T1/block-v1:BrownX+CS195R+2018_T1+type@sequential+block@a81690dbee4d4cca9e097c67a588b726/block-v1:BrownX+CS195R+2018_T1+type@vertical+block@9e2217bd08bd47bdbd7bdcde89180445)

# Activity theory part

Before you start working through this activity, you should read through the following sections:

- [Background](./notebooks/00-theory/00-overview.md)
- [Introduction](./notebooks/00-theory/01-introduction.md)

**Important:** when you first open these documents a security pop-up will appear in the top right corner; click on it and select `Disable` in the menu that appears. This will allow you to visualize the theory content.

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


## 3. Work on the exercise

Use this [link](https://classroom.github.com/a/HF1mpBKW) to generate a Github repo for this project.

To clone your repository follow these steps:

1. *From the terminal* of your base station go inside the root of this learning experience and move in the directory `packages`:

    cd lx/packages/

1. Clone your assignment's repository by running the following command and following the instructions that pop up (remember to change `<yourGithubUsername>`):

    git clone https://github.com/h2r/project-localization-slam-2019-<yourGithubUsername>.git

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

## Deliverables [TODO: update]

This part of the project has **two deliverables** in your repository, which are to be accessed and submitted via GitHub Classroom:

1. A $\LaTeX$ PDF document `ukf2d_written_solutions.pdf`, generated from `ukf2d_written_solutions.tex`, with the answers to the UKF design and implementation questions.
2. Your implementation of the UKF written in the `state_estimators/student_state_estimator_ukf_2d.py` stencil code. In this stencil code file, we have placed `TODO` tags describing where you should write your solution code to the relevant problems.

In addition to implementing the UKF in code, we want you to learn about the design process, much of which occurs outside of the code that will run the UKF. Plus, we have some questions we want you to answer in writing to demonstrate your understanding of the UKF. Hence, you will be writing up some of your solutions in $\LaTeX$. We are having you write solutions in $\LaTeX$ because it will in particular enable you to write out some of the UKF math in a clear (and visually appealing!) format. In these documents, please try to follow our math notation wherever applicable.

When you need to edit $\LaTeX$ files from your repository, open up the `ukf2d_written_solutions.tex` file in your favorite $\LaTeX$ editor. This could be in Overleaf, your Brown CS department account, or locally on your own computer. 

**Important**:*Before submitting your document, please make sure it is compiled into a PDF. If you are having trouble with $\LaTeX$, please seek out the help of a TA.*

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
