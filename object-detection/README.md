# Activity and Exercise Instructions

Follow these instructions to run these activities. At the end will be instructions to submit the exercise.

## Phase 0: System update

- 💻 Always make sure your Duckietown Shell is updated to the latest version. See [installation instructions](https://github.com/duckietown/duckietown-shell)

- 💻 Update the shell commands: `dts update`

- 💻 Pull latest containers on your laptop: `dts desktop update`

- 🚙 Clean and update your Duckiebot: `dts duckiebot update ROBOTNAME` (where `ROBOTNAME` is the name of your Duckiebot chosen during the initialization procedure.)

- 🚙 Reboot your Duckiebot.


## Phase 1: Notebooks walkthrough

 - Build the workspace with:

  💻$ `dts exercises build`
  
 - Run the notebooks **with the `--vnc` flag**

  💻$ `dts exercises lab --vnc`
  
This will run a [Jupyter Lab][lab] and open a browser window. Enter the password `quackquack`.

[lab]: https://jupyterlab.readthedocs.io/en/stable/

Click through to `01-CNN` and then click on [`cnn.ipynb`](localhost:8888/lab/tree/01-CNN/cnn.ipynb). Once you have completed that notebook, move on to the next. Make sure that you go through them in order (especially `02`-`04`).

Since we are working with neural networks now, some of these exercises require you to train neural network models. We will use [Google Colaboratory](https://colab.research.google.com/) for this. As a result, having a Google account is a prerequisite. 



## Phase 2: Evaluate and refine your model and solution

You can finetune your solution and see how it behaves in the simulator using:

    💻$ `dts exercises test --sim` 

NB: You may need to stop your `dts exercises lab --vnc` command and rerun without the `--vnc` since we will also use VNC
for `dts exercises test`.

Similar to the last exercises, you can open up the noVNC browser and look at the image stream in `rqt_image_view` to 
gauge the performance of your model. In noVNC, click on the `rqt_image_view` icon and in the dropdown menu select 
`agent/object_detection_node/object_detections_img`. This image will show boxes with labels for any objects that
are reported by your model. 

You can similarly run your agent on your Duckiebot (if you have a Jetson Nano) using:

    💻$ `dts exercises test -b ![DUCKIEBOT_NAME]`

You can repeat the procedure above for viewing the detections overlayed on your camera image.


## Phase 3: Submit the homework

Once you are satisfied with your agent's performance, you can submit it. It's a good idea to evaluate it locally with the exact challenge conditions first. This can be done with:

    💻$ `dts challenges evaluate`
    
Then finally submit with 

    💻$ `dts challenges submit`

NB: it's a good idea to delete the `duckietown_dataset` that you created before submitting because it will make your submission unnessarily large. 

## Grading Criteria

In order to pass, the submitted agent must:
 
 - Have a _survival time_ > 59 (it should basically never crash)
 - Have a _Traveled distance_ > 1

You can verify the score of submission on the [objdet challenge leaderboard](https://challenges.duckietown.org/v4/humans/challenges/mooc-modcon/leaderboard) after your submission has finished evaluating. 
