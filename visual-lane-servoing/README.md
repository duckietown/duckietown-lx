<p align="center">
<img src="./assets/images/dtlogo.png" alt="Duckietown Logo" width="50%">
</p>

# **Activity: Visual Lane Servoing**

# About these activities

## 1. Pinhole-Camera:

This notebook gives a mathematical representation of how cameras, including those on the Duckiebot, capture images by projecting 3D points onto a 2D image plane.

## 2. Homeographies:

Homographies, are mathematical transformations used in computer vision to relate points between two planes, typically in the context of camera images. By taking into consideration
an Duckietown's enviroment, assumed to be planar, this assumption helps reduce ambiguity in projection by using a homography. This notebook provide a useful tool for mapping between
planes in image processing, particularly when the environment can safely be approximated as being planar.

## 3. Camera-calibration:

For this task the duckiebot cameral parameters are calibrated by following a some guidelines that impliment a mathematical approach. This is done by holding a white paper with 
x/y axis on one side of the paper and alternating partten of dark and white sqaures to the camera face.

## Image-Filtering:

## VIsual-Servoing: 


## challenges/observation

1. The concept in this activity though new to me, i found it interesting
2. Some of the subtask needed me to look for other suporting materials 


# Instructions


### 1. System update

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


### 3. 💻 Testing in simulation

To test in simulation, use the command

    $ dts code workbench --sim

There will be two URLs popping up to open in your browser: one is the direct view of the
simulated environment. The other is VNC and only useful for some exercises, follow the instructions
in the notebooks to see if you need to access VNC.



### 4.  🚙 Testing on a physical robot

You can test your agent on the robot using the command,

    dts code workbench --duckiebot duckiebot005

