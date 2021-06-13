# Instructions

Follow these instructions to run these activities.

The final exercise counts towards grading and must be submitted for evaluation if you are pursuing the MOOC verified track.

## Make sure your system is up to date

- 💻 Always make sure your Duckietown Shell is updated to the latest version. See [installation instructions](https://github.com/duckietown/duckietown-shell)

- 💻 Update the shell commands: `dts update`

- 💻 Pull latest containers on your laptop: `dts desktop update`

- 🚙 Clean and update your Duckiebot: `dts duckiebot update ROBOTNAME` (where `ROBOTNAME` is the name of your Duckiebot chosen during the initialization procedure.)

- 🚙 Reboot your Duckiebot.

## Execute the activities

- Clone this repository.

- Build the workspace: `dts exercises build`.

- Navigate to the folder and start the documentation with `dts exercises lab`. It will open a page in your browser. The login password is `quackquack`. Make sure you do not have other Jupyter notebooks already open.

- Go to the first folder (`01-Representations`), open the notebook file, and follow through.

- You will have to execute the activities in order, from `/01-Representations` to `/05-PID-Control`. Skipping activities might result in errors.

## Submit the homework

- After completing the activities, you can proceed to `/06-PID-Control-Homework` and follow the instructions to submit your assignment.

## Grading criteria (last updated on 13 June 2021)

To pass, the submitted agent must produce at least a stable driving behavior. 

The performance metrics that will determine pass / no pass are:

Major Infractions smaller or equal to 20 **and** survival time bigger or equal to 40.

You can verify the scores of your sumbissions on the [modcon challenge leaderboard](https://challenges.duckietown.org/v4/humans/challenges/mooc-modcon/leaderboard) after your submission is evaluated.

We reserve the right to change these performance metrics to filter out unstable agents at any time. 

