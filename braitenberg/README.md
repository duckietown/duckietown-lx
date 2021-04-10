

# Instructions

Run

    dts exercises notebooks

Go inside and read and work through the following notebooks (in sequence):

- `braitenberg01`
- `braitenberg02`
- `braitenberg03`

In those notebooks you need to fill out some functions that will create your agent.

You can test your agent using these modalities:

1. 💻 `dts exercises test --sim` -- tries the agent in simulation.
2. 🚙 `dts exercises test --duckiebot_name DUCKIEBOT` -- tries the agent on your robot.
3. 🚙 `dts exercises test --local --duckiebot_name DUCKIEBOT` -- tries the agent on your robot but running on the laptop.

For additional information on how to navigate the `dt-exercises` infrastructure you can watch [this tutorial](https://docs.duckietown.org/daffy/opmanual_duckiebot/out/running_exercies.html).

You can the evaluate your work using

    dts challenges evaluate

At the end, to submit your homework, you should submit your agent using

    dts challenges submit

The robot should travel at least an average of 2 meters without hitting any duckie.
