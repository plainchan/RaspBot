#!/usr/bin/env python3


class Robot_msgs():
    def __init__(self) -> None:
        self.voltage = 0
        self.l_encoder_pulse = 0
        self.r_encoder_pulse = 0
        self.acc = [0.0,0.0,0.0]
        self.gyr = [0.0,0.0,0.0]
        self.mag = [0.0,0.0,0.0]
        self.elu = [0.0,0.0,0.0]
        
        