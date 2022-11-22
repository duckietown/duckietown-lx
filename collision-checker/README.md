# **Exercise: Collision Checker**
<img src="./assets/images/dtlogo.png" alt="Duckietown" width="20%" style="margin-top:-75px; display: block; float: right">

# Instructions

The template contained in this folder is a fully functional (wrong) solution. Currently, the solution submits random guesses as to whether the robot collides with the environment or not. You can try to evaluate right away to see how it works.


## Make sure your system is up to date

- 💻 Always make sure your Duckietown Shell is updated to the latest version. See [installation instructions](https://github.com/duckietown/duckietown-shell)

- 💻 Update the shell commands: `dts update`

- 💻 Update your laptop/desktop: `dts desktop update`

- 🚙 Update your Duckiebot: `dts duckiebot update ROBOTNAME` (where `ROBOTNAME` is the name of your Duckiebot chosen during the initialization procedure.)


## Execute the activities

- Clone this repository (or pull from the upstream remote if you already have a fork).

- Navigate to the exercise root directory and start the code editor with `dts code editor`. It will show a URL that you can open in your browser. The same instructions will appear on the web page, continue there.

- Open the first notebook ([01-Collision-Checker](./notebooks/01-Collision-Checker/collision_checker.ipynb)), and follow the instructions.


## Submit the homework

After completing the activities, you can submit your assignment with `dts code submit`.

Note that the submission will be sent to two different challenges:

- [`mooc-collision-check-vali`][vali] is the **validation** challenge. You will be able to see the score and other output.
- [`mooc-collision-check-test`][test] is the **testing** challenge. You will not be able to see the scores.


## Grading criteria

In order to pass, your agent has to get at least 95% of the queries correct on the `mooc-collision-check-test` challenge. 
(This allows some slack, so that you can experiment with probabilistic algorithms).
