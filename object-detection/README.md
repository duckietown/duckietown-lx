# **Exercise: Visual Lane Servoing**
<img src="./assets/images/dtlogo.png" alt="Duckietown" width="20%" style="margin-top:-75px; display: block; float: right">

# Instructions

The final exercise counts towards grading and must be submitted for evaluation if you are pursuing the MOOC verified track.


## Make sure your system is up to date

- 💻 Always make sure your Duckietown Shell is updated to the latest version. See [installation instructions](https://github.com/duckietown/duckietown-shell)

- 💻 Update the shell commands: `dts update`

- 💻 Update your laptop/desktop: `dts desktop update`

- 🚙 Update your Duckiebot: `dts duckiebot update ROBOTNAME` (where `ROBOTNAME` is the name of your Duckiebot chosen during the initialization procedure.)


## Execute the activities

- Clone this repository (or pull from the upstream remote if you already have a fork).

- Navigate to the exercise root directory and start the code editor with `dts code editor`. It will show a URL that you can open in your browser. The same instructions will appear on the web page, continue there.

- Open the first notebook ([01-CNN](./notebooks/01-CNN/cnn_tutorial.ipynb)), and follow the instructions. This first 
  notebook is optional - please open the notebook to assess your own understanding and determine whether to complete 
  the tutorial.

- You will have to execute the activities in order, from [01-CNN](./notebooks/01-CNN/cnn_tutorial.ipynb) to [04-Integration](./notebooks/04-Integration/integration.ipynb). Skipping activities might result in errors.


## Submit the homework

- After completing the activities, you can proceed to [04-Integration](.
  /notebooks/04-Visual-Servoing/visual_servoing_activity.ipynb) and follow the instructions to submit your assignment.

NB: it's a good idea to delete the `duckietown_dataset` that you created before submitting because it will make your submission unnessarily large. 

## Grading Criteria

In order to pass, the submitted agent must:
 
 - Have a _survival time_ > 59 (it should basically never crash)
 - Have a _Traveled distance_ > 1

You can verify the score of submission on the [objdet challenge leaderboard](https://challenges.duckietown.org/v4/humans/challenges/lx22-objdet/leaderboard) after your submission has finished evaluating. 

Note that we reserve the right to change these thresholds in order to filter out unstable agents.
