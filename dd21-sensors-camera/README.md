# **Learning Experience (LX): DD21 Sensors**

## About these activities

In this learning experience you will learn how the sensors on your Duckiedrone work.

This learning experience is provided by the Duckietown team and can be run on Duckiedrones. Visit us at the
[Duckietown Website](https://www.duckietown.com) for more learning materials, documentation, and demos.

For lecture content, see:

* [Sensors](https://learning.edge.edx.org/course/course-v1:BrownX+CS195R+2018_T1/block-v1:BrownX+CS195R+2018_T1+type@sequential+block@3bde0261d3b04ccfa06f77eec394f97a)
* [Transforms](https://learning.edge.edx.org/course/course-v1:BrownX+CS195R+2018_T1/block-v1:BrownX+CS195R+2018_T1+type@sequential+block@3dd0e7c824e94017a36abda94cf18888)
* [Measuring Velocity and Position](https://learning.edge.edx.org/course/course-v1:BrownX+CS195R+2018_T1/block-v1:BrownX+CS195R+2018_T1+type@sequential+block@ccd9eede2624475b91ce4b55ee51ce87)

### Your Robot's Sensors

Your drone is equipped with three sensors:

1. An inertial measurement unit (IMU)
1. A time-of-flight (ToF) sensor
1. A downward facing camera.

Thanks to these sensors, the drone is equipped with enough understanding of its environment to control its flight and fly autonomously. Each sensor is described below. By interfacing with each of these sensors, you will gain exposure to core robotics concepts including frame conversions, interpreting digital signals, and computer vision.

### How this project fits into software stack

Take a look at the [software architecture diagram](https://docs.duckietown.org/daffy/opmanual_sky/out/software_architecture_intro.html) and notice the hardware components: `Flight Controller`, `Time of Flight Sensor`, and `Camera`. This is the hardware you'll be interfacing with in this project. Also notice the corresponding ROS nodes in the diagram. These are the ROS nodes you'll be creating to extract and publish sensor values.

## Instructions

### Launch the code editor

Open the code editor by running the following command,

```bash
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

This assignment comprises several parts:

1. ToF Theory
1. Affine Transforms
1. Rotation Representations
1. Optical Flow

Please complete all parts of this assignment.

### Setting up the activities

Click [**this link**](https://classroom.github.com/a/QKoUdfRa) to generate a Github repo for this project.

All activities have to be run inside the container with the software of the drone. Make sure you have started it by:

1. Connecting to your drone via ssh from your base station (where `<hostname>` is the hostname of your drone):

    ```bash
    ssh duckie@<hostname>
    ```

1. Going in the `~/catkin_ws/src/pidrone_pkg`:

    ```bash
    cd ~/catkin_ws/src/pidrone_pkg
    ```

1. Starting the container:

    ```bash
    rake start
    ```

1. Starting the `screen` session:

    ```bash
    screen -c pi.screenrc
    ```

1. Then, clone the repository inside the container on the drone by changing to `~/catkin_ws/src`, and then running:

    ```bash
    git clone https://github.com/h2r/project-sensors-implementation-<yourGithubUsername>
    ```

    **note**: You should [create a GitHub personal access token](https://docs.github.com/en/authentication/keeping-your-account-and-data-secure/creating-a-personal-access-token#creating-a-fine-grained-personal-access-token) for your drone to make this possible. It only needs permissions to read and write to repositories. 

### Working on the exercise

When you need to modify the code of an exercise and test it on the drone, do so by working on your base station inside this editor, committing the modified file and then pushing it to the remote repo with `git push`.

To clone your repository follow these steps:

1. *From the terminal* of your base station go inside the directory of this learning experience (`dd21-sensors-camera`) and move in the directory `packages`:

    ```bash
    cd dd21-sensors-camera/packages/
    ```

1. Clone your assignment's repository by running the following command and following the instructions that pop up (remember to change `<yourGithubUsername>`):

    ```bash
    git clone https://github.com/h2r/project-sensors-implementation-<yourGithubUsername>
    ```

1. You can now access your project in the sidebar by going in the folder `packages`

Make sure to commit the changes and push them to GitHub each time you modify some file.

```bash
cd project-sensors-implementation-<yourGithubName>
git add -A
git commit -a -m 'some commit message. maybe hand-in, maybe update'
git push
```

## How to get your changes on the Duckiedrone

To get the changes from your repo to the Duckiedrone, pull the updates on the drone from inside the container, by running:

```bash
cd ~/catkin_ws/src/project-sensors-implementation-<yourGithubUsername>
git pull
```

## Hand in - Theory questions

Use [this link](https://classroom.github.com/a/QKoUdfRa) to access the assignment on GitHub classroom. Commit the files to hand in.

Your handin should contain the following files:

* `solutions.tex`
* `solutions.pdf`

## Hand in - Activities

When you submit your assignment, your folder should contain modified versions of the following files in addition to the other files that came with your repo:

* `student_infrared_pub.py`
* `student_analyze_flow.py`
* `student_analyze_phase.py`
* `student_flight_controller_node.py`

Commit and push your changes before the assignment is due. This will allow us to access the files you pushed to GitHub and grade them accordingly. If you commit and push after the assignment deadline, we will use your latest commit as your final submission, and you will be marked late.

Note that assignments will be graded anonymously, so please don't put your name or any other identifying information on the files you hand in.
