"""
#		    _/  _/    _/_/_/_/    _/    _/  _/   
#		   _/  _/    _/        _/  _/  _/  _/    
#		  _/_/_/_/  _/_/_/    _/  _/  _/_/_/_/   
#		     _/          _/  _/  _/      _/      
#		    _/    _/_/_/      _/        _/ 
"""

'''
Producer-consumer multi-threading reference: https://www.bogotobogo.com/python/Multithread/python_multithreading_Synchronization_Producer_Consumer_using_Queue.php
'''

import rev
import ctre
import wpilib

class __singleStage__(object):
    def __init__(self, CANAddress1):
        self.CANAddress1 = CANAddress1
        self.initialize_motors(CANAddress1)

    
    def initialize_motors(self, CANAddress1):
        self.motor1 = rev.CANSparkMax(CANAddress1, rev.CANSparkMaxLowLevel.MotorType.kBrushless)
        self.motor1.setInverted(False)
        return None

    def invert_motor(self):
        self.motor1.setInverted(True)
        return None

    def set_motors(self, value):
        self.motor1.set(value)
        return None

class __doubleStage__(object):
    def __init__(self, CANAddress1, CANAddress2):
        self.CANAddress1 = CANAddress1
        self.CANAddress2 = CANAddress2
        self.initialize_motors(CANAddress1, CANAddress2)
        return None
    
    def initialize_motors(self, CANAddress1, CANAddress2):
        self.motor1 = rev.CANSparkMax(CANAddress1, rev._rev.CANSparkMaxLowLevel.MotorType.kBrushless)
        self.motor1.setInverted(False)

        self.motor2 = rev.CANSparkMax(CANAddress2, rev._rev.CANSparkMaxLowLevel.MotorType.kBrushless)
        self.motor2.setInverted(False)

        self.motor2.follow(self.motor1)
        return None

    def invert_motor(self, id):
        if id == self.CANAddress1:
            self.motor1.setInverted(True)

        elif id == self.CANAddress2:
            self.motor2.setInverted(True)

        return None

    def set_motors(self, value):
        self.motor1.set(value)
        return None

# %% SHOOTER CLASSESS

class SHOOTER(object):

    def __init__(self):
        self.__initialize_intake__()
        self.__initialize_shooter__()
        self.__initialize_stage1__()
        self.__initialize_stage2__()
        self.timer = -1
        self.intake_arm_position = 0

    def purge(self):
        self.__set_intake__(-1)
        self.__set_shooter__(-1)
        self.__set_stage1__(-1)
        self.__set_stage2__(-1)   
        return None  

    def advance_ball(self):
        self.__set_stage1__(-0.25)
        self.__set_stage2__(-0.25) 
        return None

    # Intake Code ##################################################
    def __initialize_intake__(self):
        self.intake = __singleStage__(3)
        self.left_arm_piston = wpilib.Solenoid(9, wpilib.PneumaticsModuleType.REVPH, 1)
        self.right_arm_piston = wpilib.Solenoid(9, wpilib.PneumaticsModuleType.REVPH, 14)
        return None
    
    def __set_intake__(self, value):
        self.intake.set_motors(value)
        return None 

    def disable_intake(self):
        self.__set_intake__(0)
        return None

    def enable_intake(self, inverted=False):
        if self.intake_arm_position == 0:
            if inverted == True:
                self.__set_intake__(-0.5)
            else:
                self.__set_intake__(0.5)
        return None

    def set_intake_position(self, position):
        if self.intake_arm_position != position:
            if position == 1:
                self.left_arm_piston.set(1)
                self.right_arm_piston.set(1)
                self.enable_intake()
                self.enable_stage1()

            if position == 0:
                self.left_arm_piston.set(0)
                self.right_arm_piston.set(0)
                self.disable_intake()
                self.disable_stage1()
                
            self.intake_arm_position = position
        
        return None 

    def disable_intake(self):
        self.__set_intake__(0.0)
        self.__set_stage1__(0.0)

    # Stage 1 ##################################################
    def __initialize_stage1__(self):
        self.__stage1__ = __singleStage__(1)
        return None

    def __set_stage1__(self, value):
        self.__stage1__.set_motors(value)
        return None

    def enable_stage1(self):
        self.__set_stage1__(0.4)
        return None

    def enable_stage1_purge(self):
        self.__set_stage1__(-0.4)
        return None

    def disable_stage1(self):
        self.__set_stage1__(0)
        return None

    # Stage 2 ##################################################
    def __initialize_stage2__(self):
        self.__stage2__ = __singleStage__(2)
        return None

    def __set_stage2__(self, value):
        self.__stage2__.set_motors(-value)
        return None

    def enable_stage2(self):
        self.__set_stage2__(0.4)
        return None

    def enable_stage2_purge(self):
        self.__set_stage2__(-0.2)
        return None

    def disable_stage2(self):
        self.__set_stage2__(0.0)
        return None

    # Shooter Code ##################################################
    def __initialize_shooter__(self):
        self.__shooter__ = __doubleStage__(4, 5)
        self.__shooter__.invert_motor(4)
        return None

    def __set_shooter__(self, value):
        self.__shooter__.set_motors(value)
        return None

    def enable_shooter(self, value):
        self.__set_shooter__(value)
        return None   

    def disable_shooter(self):
        self.__set_shooter__(0.0)
        return None 

    # def enable_shooter(self):
    #     self.__set_shooter__(0.8)
    #     return None   

    # def disable_shooter(self):
    #     self.__set_shooter__(0.0)
    #     return None   

# %% ELEVATOR CLASSES

class ELEVATOR(object):
    def __init__(self):
        # self.__elevator__ =  __doubleStage__(6, 7)
        self.__elevatorL__ = __singleStage__(6)
        self.__elevatorR__ = __singleStage__(7)
        self.__elevatorL__.motor1.setIdleMode(rev.CANSparkMax.IdleMode(1))
        self.__elevatorR__.motor1.setIdleMode(rev.CANSparkMax.IdleMode(1))
        self.speed = [0]*5
        return None

    def extend_elevators(self, speed):
        self.__elevatorR__.set_motors(-speed)
        self.__elevatorL__.set_motors(-speed)
        # self.speed = self.speed[1:] + [speed]
        # avg_speed = average(self.speed)
        # if avg_speed > 0.05:
        #     self.__elevators__.set_motors(avg_speed)

    def retract_elevators(self, speed):
        self.__elevatorR__.set_motors(speed)
        self.__elevatorL__.set_motors(speed)
        # self.speed = self.speed[1:] + [speed]
        # avg_speed = average(self.speed)
        # if avg_speed > 0.05:
        #     self.__elevators__.set_motors(-avg_speed)

    def disable_elevators(self):
        self.__elevatorR__.set_motors(0)
        self.__elevatorL__.set_motors(0)

# %% DRIVETRAIN CLASS

def average(val_array):
    return sum(val_array)/len(val_array)

class DRIVETRAIN(object):
    def __init__(self, axel_length=1, wheel_diameter=0.15, ticks_per_rotation=4096, gear_ratio_low=1, gear_ratio_high=10):
        self.current_gear = 0
        self.turning_constant = axel_length/wheel_diameter*ticks_per_rotation/360.0
        self.drive_constant = ticks_per_rotation/wheel_diameter
        self.gear_ratio_low = gear_ratio_low
        self.gear_ratio_high = gear_ratio_high
        self.left_speed = [0]*5
        self.right_speed = [0]*5
        self.__initialize_drivetrain__()
        self.__initialize_shifters__()
        return None

    def __initialize_drivetrain__(self):
        self.__leftDrive__ = self.__initialize_drive_side__(10, 11, inverted=True)
        self.__rightDrive__ = self.__initialize_drive_side__(12, 13)     
        return None 

    def __initialize_shifters__(self):
        self.__leftGearbox__ = wpilib.Solenoid(9, wpilib.PneumaticsModuleType.REVPH, 0)
        self.__rightGearbox__ = wpilib.Solenoid(9, wpilib.PneumaticsModuleType.REVPH, 15)
        return None

    def __initialize_drive_side__(self, CANAddress1, CANAddress2, inverted=False):
        motor1 = ctre.TalonFX(CANAddress1)
        motor1.setInverted(inverted)

        motor2 = ctre.TalonFX(CANAddress2)
        motor2.setInverted(inverted)

        return motor1

    def __set_drivetrain__(self, side, value):
        side.set(ctre._ctre.TalonFXControlMode.PercentOutput, value)
        return None

    # Teleop Code ##################################################
    def left_drive(self, speed):
        self.left_speed = self.left_speed[1:] + [speed]
        avg_speed = average(self.left_speed)
        self.__leftDrive__.set(ctre._ctre.TalonFXControlMode.PercentOutput, avg_speed)
        return None

    def right_drive(self, speed):
        self.right_speed = self.right_speed[1:] + [speed]
        avg_speed = average(self.right_speed)
        self.__rightDrive__.set(ctre._ctre.TalonFXControlMode.PercentOutput, avg_speed)
        return None

    def shift_up(self):

        if self.current_gear != 1:
            self.__rightGearbox__.set(1)
            self.__leftGearbox__.set(1)

            # Keep track of current gearbox setting
            self.current_gear = 1

        return None

    def shift_down(self):
        
        if self.current_gear != 0:

            self.__rightGearbox__.set(0)
            self.__leftGearbox__.set(0)

            # Keep track of current gearbox setting
            self.current_gear = 0

        return None 

    # def change_drive_direction(self, direction):
    #     if direction == 1:
    #         self.__leftDrive__ = self.__initialize_drive_side__(12, 13, inverted = True)
    #         self.__rightDrive__ = self.__initialize_drive_side__(10, 11)
        
    #     elif direction == 0:
    #         self.__leftDrive__ = self.__initialize_drive_side__(10, 11, inverted = True)
    #         self.__rightDrive__ = self.__initialize_drive_side__(12, 13)

    # Autonomous Code ##################################################
    def __move_distance__(self, side, distance):
        side.set(ctre._ctre.TalonFXControlMode.MotionMagic, distance)
        return None

    def __is_finished__(self):
        if self.__rightDrive__.isMotionProfileFinished() and self.__leftDrive__.isMotionProfileFinished():
            return True

        else:
            return False

    def move(self, distance):
        # Calculate turning constant to compensate for gear ratio
        if self.current_gear == 0:
            distance = self.gear_ratio_low*self.drive_constant*distance

        elif self.current_gear == 1:
            distance = self.gear_ratio_high*self.drive_constant*distance

        else:
            distance = 0

        # Move Distance
        self.__move_distance__(self.__leftDrive__, distance)
        self.__move_distance__(self.__rightDrive__, distance)
        
        return None
    
    def turn(self, angle):
        # Calculate turning constant to compensate for gear ratio
        if self.current_gear == 0:
            distance = self.gear_ratio_low*self.turning_constant*angle

        elif self.current_gear == 1:
            distance = self.gear_ratio_high*self.turning_constant*angle

        else:
            distance = 0

        # Move Distance
        self.__move_distance__(self.__leftDrive__, distance)
        self.__move_distance__(self.__rightDrive__, -distance)
        return None

# %% CAMERA CLASSES

class CAMERA(object):
    def __init__(self):
        self.__camera__ = wpilib.CameraServer
        self.__camera__.launch()     

# %% LEDS CLASS

class LEDS(object):
    def __init__(self):
        self.blinkin = wpilib.PWMMotorController("blinkin", 0)
        self.current_color = 0
        return None

    def setSolidColor(self, color):
        if self.current_color != color:
            if color == "hot_pink":
                self.blinkin.set(0.57)
            elif color == "dark_red":
                self.blinkin.set(0.59)
            elif color == "red":
                self.blinkin.set(0.61)
            elif color == "red_orange":
                self.blinkin.set(0.63)
            elif color == "orange":
                self.blinkin.set(0.65)
            elif color == "gold":
                self.blinkin.set(0.67)
            elif color == "yellow":
                self.blinkin.set(0.69)
            elif color == "lawn_green":
                self.blinkin.set(0.71)
            elif color == "lime":
                self.blinkin.set(0.73)
            elif color == "dark_green":
                self.blinkin.set(0.75)
            elif color == "green":
                self.blinkin.set(0.77)
            elif color == "blue_green":
                self.blinkin.set(0.79)        
            elif color == "aqua":
                self.blinkin.set(0.81)
            elif color == "sky_blue":
                self.blinkin.set(0.83)
            elif color == "dark_blue":
                self.blinkin.set(0.85)
            elif color == "blue":
                self.blinkin.set(0.87)
            elif color == "blue_violet":
                self.blinkin.set(0.89)
            elif color == "violet":
                self.blinkin.set(0.91)
            elif color == "white":
                self.blinkin.set(0.93)
            elif color == "gray":
                self.blinkin.set(0.95)
            elif color == "dark_gray":
                self.blinkin.set(0.97)
            elif color == "black":
                self.blinkin.set(0.99)

            self.current_color = color

        return None
    
    def disable(self):
        self.blinkin.set(1995)
        return None   