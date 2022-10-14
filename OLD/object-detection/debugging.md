## Debugging session

I did a bit of cleanup. Most things should have been fixed anyway.

See results here: https://challenges.duckietown.org/v4/humans/submissions/15837 


### Make the two `.sh` files as similar as possible

- There was an additional `source` in `submit.sh` - removed

- In `submit.sh` we didn't use the `dt-exec` thing. (This is still a TODO.)

### Other bash stuff

- added `set -eux` to the bash files

### Configuration problems

We were very sloppy in running `solution.py` (`agent.launch`).

- `solution.py` was not run with `required=true` - it could crash and we would not detect.
- `solution.py `was not run with `output=screen`

- `agent.launch` is now run **synchronously** at the end of `submit.sh`. In this way:

  - the container will exit if it crashes,
  - we have its return code as a record, 
  - We can also provide clean shutdown.

- In `solution.py` we now shutdown cleanly rospy when the node has finished during `on_finish()`.

### Other ROS complaints/problems

ROS complained that it could not write to `$HOME/.log `: set `ROS_HOME=/tmp` to solve this.

### Problems related to stdout/stderr

Note that in ROS, `INFO` and `DEBUG` go to `stdout`, not `stderr`. [source](http://wiki.ros.org/roscpp/Overview/Logging)

This should make ROS stdout LINE buffered:

```bash
export ROSCONSOLE_STDOUT_LINE_BUFFERED=1
```

Anyway, people complain a lot about line buffering in ROS. Things seem weird. One suggestion:
using  `stdbuf -o L <program>`

### Making log files available

Now we copy all log files for each node for the user to see

```bash
# copy the log files  
find /tmp/log  -type f  -name "*.log" -exec cp {} /challenges/challenge-solution-output \\;
```


