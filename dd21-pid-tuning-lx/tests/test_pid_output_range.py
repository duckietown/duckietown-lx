from utils.writer import load_gains
from drone_sim import VerticalDrone
import matplotlib.pyplot as plt

### TEMP
import sys

# TODO: modify <yourGitHubName> to make this the path to your exercise folder
EXERCISE_DIRECTORY = "/code/dd21-pid-tuning-lx/packages/project-pid-implementation-<yourGitHubName>"
GAINS_PATH = EXERCISE_DIRECTORY+"/z_pid.yaml"

sys.path.append(EXERCISE_DIRECTORY)
####

class TestPIDOutput():
    def __init__(self, gains_path : str, PID, setpoint : float = 1) -> None:
        self.z_0 = 0
        self.setpoint = setpoint
        self.gains_path = gains_path
        self.pwm_upper_limit = 1900
        self.pwm_lower_limit = 1100
        self.PID = PID

    def simulate(self,drag_coeff=0,latency=0,noise=0):

        pid_gains = load_gains(self.gains_path)

        my_pid_instance = self.PID(
        kp=pid_gains['Kp'],
        kd=pid_gains['Kd'],
        ki=pid_gains['Ki'],
        k=pid_gains['K'],
        )

        sim = VerticalDrone(
                        pid=my_pid_instance,
                        step_size=10,
                        drag_coeff=drag_coeff,
                        latency=latency,
                        sensor_noise=noise,
                        )

        sim.update_setpoint(height=self.setpoint)
        sim.simulate()
        return sim.pwm_commands

    def simulate_with_limits(self):
        pwm_commands=self.simulate()
        fig,ax, = plt.subplots()
        ax.plot(pwm_commands)

        border_ticks = 100

        ax.set_ylim([self.pwm_lower_limit-border_ticks, self.pwm_upper_limit+border_ticks])

        ax.set_xlabel('Simulation step')
        ax.set_ylabel('PWM Control Output')
        ax.set_title(f"PWM Control Output, setpoint {self.setpoint}m")
        
        plt.axhline(y=self.pwm_lower_limit,xmin=-100,xmax=100,color='r',linestyle='-')
        plt.axhline(y=self.pwm_upper_limit,xmin=-100,xmax=100,color='r',linestyle='-')
        fig.show()

if __name__ == "__main__":
    from student_pid_class import PID
    test = TestPIDOutput(GAINS_PATH,PID)
    test.simulate_with_limits()