<p align="center">
<img src="./assets/images/dtlogo.png" alt="Duckietown Logo" width="50%">
</p>

# **Activity: Braitenberg**

# About these activities

This activity comprises of three notebooks; Image-Manipulation, Image-Filtering and Braitenberg, which i use to 
develop computer vision techniques that manipulate and filter images to highlight specific objects; duckies on the road (duckietown) 
then utilize the filtered images to guide the development of my own Braitenberg agent implimentation.

The Braitenberg agent uses a mounted camera as a visual stimuli, using the connection.py file splits the observed image into two 
matrices; get_motor_left_matrix and get_motor_right_matrix essentially to visualize the image and preprocessing.py to get the best hsv
values to identify the duckies.


## challenges/observation of this activity

1. Knowledge of python has been helpful in performing this activity
2. The main observation in this activity is the difference in simulation and implementation in the duckiebot; in order to achieve this

   task i had to use an image of the real duckie enviroment, which required getting the real hsv values that identifies the duckie in the
   duckiebot environment. Within the `/assets/samples/iotlab/` folder is a picture of the duckiebot environment used for this task `real001.png` 



# Instructions

Provided instruction on step-by-step execution of the activity


## 1. System update

- 💻 Update the shell commands: `dts update`

- 💻 Update your laptop/desktop: `dts desktop update`

- 🚙 Update your Duckiebot: `dts duckiebot update duckiebot005`


## 2. Launch the code editor

Open the code editor by running the following command,

```
dts code editor
```

Wait for a URL to appear on the terminal, then click on it or copy-paste it in the address bar
of your browser to access the code editor. The first thing you will see in the code editor is
this same document, you can continue there.


## 3. 💻 Testing in simulation

To test in simulation, use the command

    $ dts code workbench --sim

## 4. 🚙 Testing on a physical robot

You can test your agent on the robot using the command,

    $ dts code workbench --duckiebot duckiebot005

The duckiebot should now execute the braitenberg agent

