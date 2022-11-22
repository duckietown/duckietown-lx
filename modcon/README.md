# **Exercise: ModCon - Modeling and Control**
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

- Open the first notebook ([01-Representations](./notebooks/01-Representations/pose_representation.ipynb)), and follow the instructions.

- You will have to execute the activities in order, from [01-Representations](./notebooks/01-Representations/pose_representation.ipynb) to [05-PID-Control](./notebooks/05-PID-Control/PID_controller.ipynb). Skipping activities might result in errors.


## Submit the homework

- After completing the activities, you can proceed to [06-PID-Control-Homework](./notebooks/06-PID-Control-Homework/PID_controller_homework.ipynb) and follow the instructions to submit your assignment.


## Grading criteria

To pass, the submitted agent must produce a stable driving behavior.

The performance metrics that will determine pass / no pass are:
- Major Infractions smaller or equal to 20; **and**
- Survival Time bigger or equal to 40 seconds;

You can verify the scores of your submissions on the [Challenge Leaderboard](https://challenges.duckietown.org/v4/humans/challenges/mooc-modcon/leaderboard) after your submission is evaluated.
