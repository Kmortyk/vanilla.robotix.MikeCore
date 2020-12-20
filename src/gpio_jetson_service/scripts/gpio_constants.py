from enum import Enum


class GPIOPins(Enum):
    LEFT_PWM = 32
    LEFT_FORWARD = 38
    LEFT_BACKWARD = 40
    RIGHT_PWM = 33
    RIGHT_FORWARD = 37
    RIGHT_BACKWARD = 35


GPIO_PINS_ALL = [GPIOPins.LEFT_PWM.value, GPIOPins.RIGHT_PWM.value, GPIOPins.RIGHT_BACKWARD.value,
                 GPIOPins.RIGHT_FORWARD.value, GPIOPins.LEFT_FORWARD.value, GPIOPins.LEFT_BACKWARD.value]

PWM_FREQUENCY = 20
PWM_LOW = 25
PWM_MID = 65
PWM_FAST = 100


class Command(Enum):
    FULL_STOP = 0
    LEFT_STOP = 1
    RIGHT_STOP = 2
    FORWARD_LOW = 3
    FORWARD_MIDDLE = 4
    FORWARD_FAST = 5
    BACKWARD_LOW = 6
    BACKWARD_MIDDLE = 7
    BACKWARD_FAST = 8
    LEFT_FORWARD_LOW = 9
    LEFT_FORWARD_MIDDLE = 10
    LEFT_FORWARD_FAST = 11
    LEFT_BACKWARD_LOW = 12
    LEFT_BACKWARD_MIDDLE = 13
    LEFT_BACKWARD_FAST = 14
    RIGHT_FORWARD_LOW = 15
    RIGHT_FORWARD_MIDDLE = 16
    RIGHT_FORWARD_FAST = 17
    RIGHT_BACKWARD_LOW = 18
    RIGHT_BACKWARD_MIDDLE = 19
    RIGHT_BACKWARD_FAST = 20


class Motor(Enum):
    left = 0
    right = 1


class Direction(Enum):
    forward = 0
    backward = 1


class Speed(Enum):
    low = 0
    middle = 1
    fast = 2


class PinDirection(Enum):
    d_in = 0
    d_out = 1
