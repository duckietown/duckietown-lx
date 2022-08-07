# Activity and Exercise Instructions

Follow these instructions to run these activities. At the end will be instructions to submit the exercise.

The final exercise counts towards grading and must be submitted for evaluation if you are pursuing the MOOC verified track.

## Phase 0: System update

- 💻 Always make sure your Duckietown Shell is updated to the latest version. See [installation instructions](https://github.com/duckietown/duckietown-shell)

-    If you are working on a fork of this repository, make sure to pull from the upstream remote

- 💻 Update the shell commands: `dts update`

- 💻 Pull latest containers on your laptop: `dts desktop update`

- 🚙 Clean and update your Duckiebot: `dts duckiebot update ROBOTNAME` (where `ROBOTNAME` is the name of your Duckiebot chosen during the initialization procedure.)

- 🚙 Reboot your Duckiebot.


## Phase 1: Notebooks walkthrough

 - Build the workspace with:

  💻$ `dts exercises build`
  
 - Run the notebooks 

  💻$ `dts exercises lab `
  
This will run a [Jupyter Lab][lab] and open a browser window. Enter the password `quackquack`.

[lab]: https://jupyterlab.readthedocs.io/en/stable/

The first two notebooks have simple pedagogical examples of the Kalman and particle filters. 
The third notebook contains an implementation of the histogram filter in duckietown with some parts in 
the code to be completed as part of the exercise. 


## Phase 2: Evaluate and refine your  solution

You can finetune your solution and see how it behaves in the simulator using:

    💻$ `dts exercises test --sim` 

Similar to the last exercises, you can open up the noVNC browser and look at the image stream in `rqt_image_view` to 
gauge the performance of your model. In noVNC, click on the `rqt_image_view` icon and in the dropdown menu select 
`agent/lane_filter_node/belief_img`. This image is a matrix representation of the belief histogram.

To make the robot move, you will have to put it into `LANE_FOLLOWING` mode. You can do this by opening the joystick
and clicking `a` (for autonomous). Hitting `s` with the joytsick window active will return you to 
`NORMAL_JOYSTICK_CONTROL` mode.

You can similarly run your agent on your Duckiebot using:

    💻$ `dts exercises test -b ![DUCKIEBOT_NAME]`

You can repeat the procedure above for viewing the detections overlayed on your camera image.


## Phase 3: Submit the homework

Once you are satisfied with your agent's performance, you can submit it. It's a good idea to evaluate it locally with the exact challenge conditions first. This can be done with:

    💻$ `dts challenges evaluate`
    
Then finally submit with 

    💻$ `dts challenges submit`


## Grading criteria

Your submitted agent should be able to navigate an entire loop in Duckietown. 

N.B. This bar could be subject to change if deemed to difficult due to the downstream controller tuning.

You can verify the scores of your sumbissions on the [mooc-state-estimation challenge leaderboard](https://challenges.duckietown.org/v4/humans/challenges/mooc-state-estimation/leaderboard) after your submission is evaluated.