# Activity and Exercise Instructions

Follow these instructions to run these activities. At the end will be instructions to submit the exercise.

## Phase 0: System update

- 💻 Always make sure your Duckietown Shell is updated to the latest version. See [installation instructions](https://github.com/duckietown/duckietown-shell)

- 💻 Update the shell commands: `dts update`

- 💻 Pull latest containers on your laptop: `dts desktop update`

- 🚙 Clean and update your Duckiebot: `dts duckiebot update ROBOTNAME` (where `ROBOTNAME` is the name of your Duckiebot chosen during the initialization procedure.)

- 🚙 Reboot your Duckiebot.


## Phase 1: Notebooks walkthrough

If there are any notebooks in the exercises (there are none in this template) then you can execute them with the following procedure.

 - Build the workspace with:
  
  💻$ `dts exercises build`
  
 - Run the notebooks **with the `--vnc` flag**

  💻$ `dts exercises lab --vnc`
  
This will run a [Jupyter Lab][lab] and open a browser window. Enter the password `quackquack`.

[lab]: https://jupyterlab.readthedocs.io/en/stable/

Click the notebooks in the left hand menu (usually it is important that they are done in order). 


## Phase 2: Evaluate and refine

You can run your "solution" and see how it behaves in the simulator using:

    💻$ `dts exercises test --sim` 

NB: You may need to stop your `dts exercises lab --vnc` command and rerun without the `--vnc` since we will also use VNC
for `dts exercises test`.

You can open up the noVNC browser and look at the image stream in `rqt_image_view` 

You can similarly run your agent on your Duckiebot using:

    💻$ `dts exercises test -b ![DUCKIEBOT_NAME]`

You can repeat the procedure above for viewing the detections overlayed on your camera image.


## Phase 3: Submit the homework

Once you are satisfied with your agent's performance, you can submit it. It's a good idea to evaluate it locally with the exact challenge conditions first. This can be done with:

    💻$ `dts challenges evaluate`
    
Then finally submit with 

    💻$ `dts challenges submit`

You can verify the score of submission on the challenge server. The challenge for this template exercise is the [ROS template](https://challenges.duckietown.org/v4/humans/challenges/mooc-ros-template/leaderboard). 
