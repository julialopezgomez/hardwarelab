# from motor_control.AROMotorControl import AROMotorControl
# mc = AROMotorControl()
# mc.readPosition(motorid=1)

# from template import run_until
# dt = 0.005
# # Calculate how many iterations the motor should run 
# # based on a 2-second duration and the time step.
# N = int(2. / dt) 
# try:
#     run_until(mc.applyTorqueToMotor, N=N, dt=0.005, motorid=1, torque=0.02)
# except KeyboardInterrupt:
#     print("KeyboardInterrupt received, stopping motors...")
# except Exception as e:
#     print(f"an error occurred: {e}")
# finally:
#     mc.applyTorqueToMotor(1, 0) # stop the motor
#     mc.applyTorqueToMotor(2, 0)
#     # applyTorqueToMotor takes the arguments:
#     # motorid: the motor to apply the torque to
#     # torque: the torque to apply to the motor
#     print("motors stopped!")


# import matplotlib.pyplot as plt
# anglevalues =[]
# def store_values_and_apply_torques(motorid, torque):
#     global anglevalues
#     mc.applyTorqueToMotor(motorid=motorid, torque=torque)
#     anglevalues.append(mc.readPosition(motorid=motorid))

# try:
#     run_until(store_values_and_apply_torques, N=N, dt=0.005, motorid=1, torque=0.02)
# except KeyboardInterrupt:
#     print("KeyboardInterrupt received, stopping motors...")
# except Exception as e:
#     print(f"an error occurred: {e}")
# finally:
#     mc.applyTorqueToMotor(1, 0)
#     mc.applyTorqueToMotor(2, 0)
#     print("motors stopped!")


# time_values = [i * dt for i in range(len(anglevalues))]

# # Plotting the angle values
# plt.figure(figsize=(10, 5))
# plt.plot(time_values, anglevalues, marker='o', linestyle='-')
# plt.title('Motor Angle Values Over Time')
# plt.xlabel('Time (s)')
# plt.ylabel('Angle Values (% 2Pi)')
# plt.grid()
# plt.show()

from template import run_until
import matplotlib.pyplot as plt
from motor_control.AROMotorControl import AROMotorControl



def clip(output):
    outabs = abs(output)
    if outabs < 1e-4:
        return 0
    clipped = max(min(outabs, 0.1), 0.02)
    return clipped if output > 0 else -clipped

class PController:
    def __init__(self, Kp):
        self.Kp = Kp
    def shortest_path_error(self, target, current):
        diff = ( target - current + 180 ) % 360 - 180;
        if diff < -180:
            diff = diff + 360
        if (current + diff) % 360 == target:
            return diff
        else:
            return -diff
        
    def compute(self, target, current):
        error = self.shortest_path_error(target, current)
        output = self.Kp*error
        return clip(output)