from mpu6050 import mpu6050
import math
import time

class TURRET_IMU(object):
    def __init__(self):
        self.q = [1., 0., 0., 0.]
        self.aRes = 1.0
        self.gRes = 1.0
        self.prev_time = time.time()
        self.gBias = [0.0, 0.0, 0.0]
        self.aBias = [0.0, 0.0, 0.0] # Bias corrections for gyro and accelerometer

        GyroMeasError = math.pi * (40.0 / 180.0)     # gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
        self.beta = math.sqrt(3.0 / 4.0) * GyroMeasError;  # compute beta
        GyroMeasDrift = math.pi * (2.0 / 180.0);      # gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
        self.zeta = math.sqrt(3.0 / 4.0) * GyroMeasDrift  # compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value

        self.setup()

    def setup(self, I2Caddress=0x68):
        self.mpu = mpu6050(I2Caddress)
        return None

    def getYPR(self):
        yaw   = math.atan2(2.0 * (self.q[1] * self.q[2] + self.q[0] * self.q[3]), self.q[0] * self.q[0] + self.q[1] * self.q[1] - self.q[2] * self.q[2] - self.q[3] * self.q[3])
        pitch = -math.asin(2.0 * (self.q[1] * self.q[3] - self.q[0] * self.q[2]))
        roll  = math.atan2(2.0 * (self.q[0] * self.q[1] + self.q[2] * self.q[3]), self.q[0] * self.q[0] - self.q[1] * self.q[1] - self.q[2] * self.q[2] + self.q[3] * self.q[3])
        pitch *= 180.0 / math.pi
        yaw   *= 180.0 / math.pi
        roll  *= 180.0 / math.pi
        return [yaw, pitch, roll]

    def update(self):
        aCount = self.mpu.get_accel_data()
        ax = aCount['x']*self.aRes
        ay = aCount['y']*self.aRes
        az = aCount['z']*self.aRes

        gCount = self.mpu.get_gyro_data()
        gx = gCount['x']*self.gRes
        gy = gCount['y']*self.gRes
        gz = gCount['z']*self.gRes

        q1 = self.q[0]
        q2 = self.q[1]
        q3 = self.q[2]
        q4 = self.q[3]

        # Auxiliary variables to avoid repeated arithmetic
        _halfq1 = 0.5 * q1
        _halfq2 = 0.5 * q2
        _halfq3 = 0.5 * q3
        _halfq4 = 0.5 * q4
        _2q1 = 2.0 * q1
        _2q2 = 2.0 * q2
        _2q3 = 2.0 * q3
        _2q4 = 2.0 * q4
        _2q1q3 = 2.0 * q1 * q3
        _2q3q4 = 2.0 * q3 * q4

        # Normalise accelerometer measurement
        norm = math.sqrt(ax * ax + ay * ay + az * az)
        if (norm == 0.0):
            return None # handle NaN
        norm = 1.0/norm
        ax *= norm
        ay *= norm
        az *= norm
        
        # Compute the objective function and Jacobian
        f1 = _2q2 * q4 - _2q1 * q3 - ax
        f2 = _2q1 * q2 + _2q3 * q4 - ay
        f3 = 1.0 - _2q2 * q2 - _2q3 * q3 - az
        J_11or24 = _2q3
        J_12or23 = _2q4
        J_13or22 = _2q1
        J_14or21 = _2q2
        J_32 = 2.0 * J_14or21
        J_33 = 2.0 * J_11or24

        # Compute the gradient (matrix multiplication)
        hatDot1 = J_14or21 * f2 - J_11or24 * f1
        hatDot2 = J_12or23 * f1 + J_13or22 * f2 - J_32 * f3
        hatDot3 = J_12or23 * f2 - J_33 *f3 - J_13or22 * f1
        hatDot4 = J_14or21 * f1 + J_11or24 * f2
        
        # Normalize the gradient
        norm = math.sqrt(hatDot1 * hatDot1 + hatDot2 * hatDot2 + hatDot3 * hatDot3 + hatDot4 * hatDot4)
        hatDot1 /= norm
        hatDot2 /= norm
        hatDot3 /= norm
        hatDot4 /= norm
        
        # Compute estimated gyroscope biases
        gerrx = _2q1 * hatDot2 - _2q2 * hatDot1 - _2q3 * hatDot4 + _2q4 * hatDot3
        gerry = _2q1 * hatDot3 + _2q2 * hatDot4 - _2q3 * hatDot1 - _2q4 * hatDot2
        gerrz = _2q1 * hatDot4 - _2q2 * hatDot3 + _2q3 * hatDot2 - _2q4 * hatDot1

        # Calculate time since last
        now = time.time()
        dt = now - self.prev_time
        self.prev_time = now
        
        # Compute and remove gyroscope biases
        self.gBias[0] += gerrx * dt * self.zeta
        self.gBias[1] += gerry * dt * self.zeta
        self.gBias[2] += gerrz * dt * self.zeta
        gx -= self.gBias[0]
        gy -= self.gBias[1]
        gz -= self.gBias[2]
        
        # Compute the quaternion derivative
        qDot1 = -_halfq2 * gx - _halfq3 * gy - _halfq4 * gz
        qDot2 =  _halfq1 * gx + _halfq3 * gz - _halfq4 * gy
        qDot3 =  _halfq1 * gy - _halfq2 * gz + _halfq4 * gx
        qDot4 =  _halfq1 * gz + _halfq2 * gy - _halfq3 * gx

        # Compute then integrate estimated quaternion derivative
        q1 += (qDot1 -(self.beta * hatDot1)) * dt
        q2 += (qDot2 -(self.beta * hatDot2)) * dt
        q3 += (qDot3 -(self.beta * hatDot3)) * dt
        q4 += (qDot4 -(self.beta * hatDot4)) * dt

        # Normalize the quaternion
        norm = math.sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4)    # normalise quaternion
        norm = 1.0/norm
        self.q = [q1 * norm,
                  q2 * norm,
                  q3 * norm,
                  q4 * norm]

        return None

if __name__ == '__main__':
    imu = TURRET_IMU()
    
    while True:
        imu.update()
        [pitch, roll, yaw] = imu.getYPR()
        print(f'Yaw: {yaw} | Pitch: {pitch} | Roll: {roll} ')