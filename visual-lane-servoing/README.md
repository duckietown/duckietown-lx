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

- Open the first notebook ([01-Pinhole-Camera](./notebooks/01-Pinhole-Camera/pinhole_camera_matrix.ipynb)), and follow the instructions.

- You will have to execute the activities in order, from [01-Pinhole-Camera](./notebooks/01-Pinhole-Camera/pinhole_camera_matrix.ipynb) to [04-Image-Filtering](./notebooks/04-Image-Filtering/image_filtering.ipynb). Skipping activities might result in errors.


## Submit the homework

- After completing the activities, you can proceed to [05-Visual-Servoing](./notebooks/05-Visual-Servoing/visual_servoing_activity.ipynb) and follow the instructions to submit your assignment.


## Grading criteria

In order to pass, your agent should be capable of navigating in its lane in a *reasonable* manner, where we define "reasonable" in terms of how well the vehicle stays centered in its lane (measured in terms of lateral deviation) and the total distance that the vehicle travels.

In particular, the following are both required in order to pass:
- Traveled distance > 4.8
- Lateral deviation < 4.0

You can monitor your performance using the [Challenge Leaderboard](https://challenges.duckietown.org/v4/humans/challenges/lx22-visservoing/leaderboard).

Note that we reserve the right to change these thresholds in order to filter out unstable agents.
