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

- Go to the first folder (`01-Pinhole-Camera`), open the notebook file, and follow through.

- You will have to execute the activities in order, from `/01-Pinhole-Camera` to `/04-Image-Filtering`. Skipping activities might result in errors.

## Submit the homework

- After completing the activities, you can proceed to `/05-Visual-Servoing` and follow the instructions to submit your assignment.

## Grading criteria

In order to pass, your agent should be capable of navigating in its lane in a *reasonable* manner, where we define "reasonable" in terms of how well the vehicle stays centered in its lane (measured in terms of lateral deviation) and the total distance that the vehicle travels.

In particular, the following are both required in order to pass:
* Traveled distance > 4.8
* Lateral deviation < 4.0

You can monitor your performance using the [mooc-visservoing challenge leaderboard](https://challenges.duckietown.org/v4/humans/challenges/mooc-visservoing/leaderboard).

Note that we reserve the right to change these thresholds in order to filter out unstable agents.
