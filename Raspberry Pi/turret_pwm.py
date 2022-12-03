from pca9685 import pca9685
import time

class TURRET_PWM(object):
    def __init__(self, freq=60):
        self.device = pca9685()
        self.device.set_pwm_freq(freq)
        
        # Configure min and max servo pulse lengths
        self.yaw_servo_lim = [150, 600]  # Pulse length out of 4096
        self.pitch_servo_lim = [150, 600]
        self.servo_abs_max = 4096
        
    def set_servo_pulse(self, channel, pulse):
        self.device.set_pwm(channel, 0, pulse)
        return False
    
    def set_yaw(self, norm_val):
        
        val = int((norm_val + 1.0)/2.0*(self.yaw_servo_lim[1]-self.yaw_servo_lim[0]) + self.yaw_servo_lim[0])

        if val > self.yaw_servo_lim[1]:   
            self.set_servo_pulse(1, self.yaw_servo_lim[1])
            return False
            
        if val < self.yaw_servo_lim[0]:
            self.set_servo_pulse(1, self.yaw_servo_lim[0])
            return False
            
        self.set_servo_pulse(1, val)
        return True
    
    def set_pitch(self, norm_val):
        
        val = int((norm_val + 1.0)/2.0*(self.pitch_servo_lim[1]-self.pitch_servo_lim[0]) + self.pitch_servo_lim[0])
        
        if val > self.pitch_servo_lim[1]:   
            self.set_servo_pulse(0, self.pitch_servo_lim[1])
            return False
            
        if val < self.pitch_servo_lim[0]:
            self.set_servo_pulse(0, self.pitch_servo_lim[0])
            return False
        print(val)
        self.set_servo_pulse(0, val)
        return True
    
if __name__ == '__main__':
    
    pwm = TURRET_PWM()
    
    while True:
        # Move servo on channel O between extremes.
        pwm.set_pitch(0.1)
        pwm.set_yaw(0.1)
        time.sleep(1)
        pwm.set_pitch(0.9)
        pwm.set_yaw(0.9)
        time.sleep(1)    
            
