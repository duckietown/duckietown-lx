
# Instructions



## Phase 0: system update


### Pull from the upstream remote

    git pull upstream
    git merge upstream/daffy
    

Make sure you have an updated system using

    dts desktop update


### Update Duckiebot

If you have a Duckiebot, also do

    dts duckiebot update



## Phase 1: Walkthrough of notebooks

Run

```
dts exercises build
dts exercises lab
```

Open the web link that will appear. You are in [Jupyter Lab][lab] in the 
folder `solution`. Open the notebook `braitenberg01`.

[lab]: https://jupyterlab.readthedocs.io/en/stable/

Work through the notebooks in sequence.

The notebooks guide you to fill out some functions in `preprocessing.py` and `connections.py`.

They will also indicate the use of other tools.

## Phase 2: Experiment in finding connections, modifying the agent

In this phase you have to experiment with the rest of the agent.

The skeleton is in `solution/agent.py`. Read through and see how it uses the things you defined in the other Python files.

Feel free to change anything.

**Do note that you will most probably need to edit the BraitenbergAgentConfig class! Its current gain and const values are off.** They are much larger than they should be, as a way to help you finetune them. 

At the begining of an evaluation episode, the `max` and `min` values for both motors will be off, but as the agent lives, it'll adjust itself. This slight change in motor values is normal.

### 💻 Testing in simulation

To test in simulation, use the command

    $ dts exercises test --sim

There will be two URLs popping up to open in your browser: one is the direct view of the experiment (probably `http://localhost:8090`).
The other is VNC and not useful for this exercise. Ignore it.

This simulation test will be very slow! We suggest opening the simulation viewer and enjoying a cup of tea/coffee while
your agent does its job. Monitor how it acts! You might get some ideas on how to fix your matrices or the agent.py.

This simulation test is just that, a test. Don't trust it fully. If you want a more accurate metric of performance, continue
reading to the `Do local evaluations` section below.


### 🚙 Testing on the robot

You can test your agent on the robot using:
 
    dts exercises test --duckiebot_name YOUR_DUCKIEBOT

This is the modality "everything runs on the robot".

You can also test using 

    dts exercises test --duckiebot_name YOUR_DUCKIEBOT  --local 

This is the modality "drivers running on the robot, agent runs on the laptop."



[comment]: <> (For additional information on how to navigate the `dt-exercises` infrastructure you can watch [this tutorial]&#40;https://docs.duckietown.org/daffy/opmanual_duckiebot/out/running_exercies.html&#41;.)

## Phase 3: Make a submission

At the end, to submit your homework, you should submit your agent using

    dts challenges submit

The robot should travel at least an average of 2 meters from the starting point.


If an error of this form occurs

```bash
Traceback (most recent call last):
  File "/usr/local/lib/python3.8/dist-packages/duckietown_challenges_cli/cli.py", line 76, in dt_challenges_cli_main
    dt_challenges_cli_main_(args=args, sections=sections, main_cmd="challenges")
  File "/usr/local/lib/python3.8/dist-packages/duckietown_challenges_cli/cli.py", line 203, in dt_challenges_cli_main_
    f(rest, environment)
  File "/usr/local/lib/python3.8/dist-packages/duckietown_challenges_cli/cli_submit.py", line 165, in dt_challenges_cli_submit
    br = submission_build(
  File "/usr/local/lib/python3.8/dist-packages/duckietown_challenges_cli/cmd_submit_build.py", line 41, in submission_build
    raise ZException(msg, available=list(credentials))
zuper_commons.types.exceptions.ZException: Credentials for registry docker.io not available
available:
```

you need to log into docker using `dts`. Use this command: 

```
dts challenges config --docker-username <USERNAME> --docker-password <PASSWORD>
```

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



