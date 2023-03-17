# Tests

These tests should be implemented as unit tests, especialy the check on `step()` output range

1. The output of `step()` function in `student_pid_class.py` should be between 1100 and 1900.

2. The simulated drone should converge exactly at the setpoint and not oscillate for:    
- Idealized PID, `python sim.py`,  
- PID with latency, `python sim.py -l 6`, 
- PID with latency, noise and drag, `python sim.py -l 3 -n 0.5 -d 0.02`     