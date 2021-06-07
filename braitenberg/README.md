
# Instructions

## Phase 0: system update

Make sure you have an updated system using

    dts desktop update


### Update Duckiebot

If you have a Duckiebot, also do

    dts duckiebot update

## Phase 1: walkthrough of notebooks

Run

    dts exercises lab

Open the web link that will appear. You are in [Jupyter Lab][lab] in the 
folder `solution`. Open the notebook `braitenberg01`.

[lab]: https://jupyterlab.readthedocs.io/en/stable/

Work through the notebooks in sequence.

The notebooks guide you to fill out some functions in `preprocessing.py` and `connections.py`.

They will also indicate the use of other tools.

## Phase 2: experiment in finding connections, modifying the agent

In this phase you have to experiment with the rest of the agent.

The skeleton is in `solution/agent.py`. Read through and see how it uses the things you defined in the other Python files.

Feel free to change anything.

### 💻 Testing in simulation

To test in simulation, use the command

    $ dts exercises test --sim

There will be two URLs popping up to open in your browser: one is the direct view of the experiment.
The other is VNC and not useful for this exercise. 


### 🚙 Testing on the robot

You can test your agent on the robot using:
 
    dts exercises test --duckiebot_name YOUR_DUCKIEBOT

If you remember the video lecture, this is the modality "everything runs on the robot".

You can also test using 

    dts exercises test --duckiebot_name YOUR_DUCKIEBOT  --local 

This is the modality "drivers running on the robot, agent runs on the laptop."



[comment]: <> (For additional information on how to navigate the `dt-exercises` infrastructure you can watch [this tutorial]&#40;https://docs.duckietown.org/daffy/opmanual_duckiebot/out/running_exercies.html&#41;.)

## Phase 3: make a submission

At the end, to submit your homework, you should submit your agent using

    dts challenges submit

The robot should travel at least an average of 2 meters from the starting point. 

(**Note:** somebody already found a way to cheat the previous metric that was here; we need to implement this new metric.)

(Note: we might change the target to make it easier if we see it is too hard in the following days. Please understand that we are still calibrating against the huge variety of participants.)


### Happy life all together in harmony 

We run the cloud evaluation service based on limited resources from our universities and sponsorships. To avoid a denial-of-service-by-many-exercises-submissions attack, we implemented a priority queue: the more submissions you send, the lower priority you have and it will take more time to do them. This is fair in the sense that, if you have never submitted, you are on top of the queue. However, we try to evaluate all submissions. 

We hope this works and we don't need more stringent policies.

You can help us in the following ways.

### Retire obsolete submissions

Note that you can "retire" submissions.

If you do this: 

    dts challenges submit --retire-same-label

then the old submission to the challenge will be related.

There is also a command `dts challenges retire`.

Finally, you can do this [on the Challenges website](https://challenges.duckietown.org/v4/).

### Do local evaluations

We suggest you evaluate your work using the command

    dts challenges evaluate

This should take a few minutes. This is not supposed to be an interactive process: just let it run, and when you return, you will find the output in a folder, including videos, and trajectories, and all the statistics you find on the website.



# Updates
 

## April 16th, 2021

- Removed parameters in agent_env.yaml - it was unclear how to modify them.

## June 4th, 2021

- This [student-contributed tool](https://github.com/martin0004/color_signature_tool) could help in the HSV part of the exercise.

