# **Learning Experience (LX): DD21 Sensors**

# About these activities

In this learning experience you will learn how the sensors on your Duckiedrone work.

This learning experience is provided by the Duckietown team and can be run on Duckiedrones. Visit us at the 
[Duckietown Website](https://www.duckietown.com) for more learning materials, documentation, and demos.

For lecture content, see:

* [Sensors](https://learning.edge.edx.org/course/course-v1:BrownX+CS195R+2018_T1/block-v1:BrownX+CS195R+2018_T1+type@sequential+block@3bde0261d3b04ccfa06f77eec394f97a)
* [Transforms](https://learning.edge.edx.org/course/course-v1:BrownX+CS195R+2018_T1/block-v1:BrownX+CS195R+2018_T1+type@sequential+block@3dd0e7c824e94017a36abda94cf18888)
* [Measuring Velocity and Position](https://learning.edge.edx.org/course/course-v1:BrownX+CS195R+2018_T1/block-v1:BrownX+CS195R+2018_T1+type@sequential+block@ccd9eede2624475b91ce4b55ee51ce87)

# Instructions

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

This assignment comprises several parts: 

1. ToF Theory
1. Affine Transforms
1. Rotation Representations
1. Optical Flow

Please complete all parts of this assignment.


## 1. Make sure you run the exercises in the container
All exercises have to be run inside the container with the software of the drone. Make sure you have started it by:

1. Connecting to your drone via ssh from your base station (where `<hostname>` is the hostname of your drone):

    ```
    ssh duckie@<hostname>
    ```

1. Going in the `~/catkin_ws/src/pidrone_pkg`:

    ```
    cd ~/catkin_ws/src/pidrone_pkg
    ```

1. Starting the container:

    ```
    $ rake start
    ```

1. Starting the `screen` session:

    ```
    screen -c pi.screenrc
    ```


## 2. Working on the exercise

When you need to modify the code of an exercise and test it on the drone, do so by working on your base station, committing `commit -m "message here"` the modified file and then pushing it to the remote repo `git push`.

Pull the update

## Handin

Use [this link](https://classroom.github.com/a/QKoUdfRa) to access the assignment on GitHub classroom. Commit the
files to hand in, as you did in the Introduction assignment.

Your handin should contain the following files:

- `solutions.tex` 
- `solutions.pdf`

