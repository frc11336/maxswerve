import commands2
import wpilib
import rev
import time
import math
from constants import AuxConstants
from rev import SparkMax, SparkMaxConfig
from commands2.button import CommandXboxController
import asyncio

# 1. Subsystem
class ShooterCommands(commands2.Subsystem):
    def __init__(self, shooter):
        super().__init__()
        self.shooter = shooter


    def get_power(self, distance):
        power = (0.00348612 * distance) + 0.328365
        return (power)
    
    def fire(self, distance):
        speed = self.get_power(distance)
        self.shooter.Shooter_set_speed(speed)
        time.sleep(0.7)
        self.shooter.Feeder_set_speed(.9)
    
    def fire_Power(self, Power):
        print ("activating")
        self.shooter.Shooter_set_speed(Power)
        time.sleep(0.5)
        self.shooter.Feeder_set_speed(1)
        
