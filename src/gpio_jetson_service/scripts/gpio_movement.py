import rospy
import Jetson.GPIO as GPIO
from gpio_constants import *

gpio_initialized = 0
# noinspection PyTypeChecker
pwm_left = None  # type: GPIO.PWM
# noinspection PyTypeChecker
pwm_right = None  # type: GPIO.PWM


def gpio_init():
    global pwm_left, pwm_right, gpio_initialized
    if gpio_initialized:
        rospy.logerr("GPIO Already init!")
        return 0
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(GPIO_PINS_ALL, GPIO.OUT, initial=GPIO.LOW)
    pwm_left = GPIO.PWM(GPIOPins.LEFT_PWM.value, PWM_FREQUENCY)
    pwm_right = GPIO.PWM(GPIOPins.RIGHT_PWM.value, PWM_FREQUENCY)
    gpio_initialized = 1
    return gpio_initialized


def gpio_deinit():
    global pwm_left, pwm_right, gpio_initialized
    if not gpio_initialized:
        rospy.logerr("GPIO doesn't init!")
        return 0
    pwm_left.stop()
    pwm_right.stop()
    GPIO.cleanup()
    gpio_initialized = 0
    return 1


def gpio_stop(motor):
    global pwm_left, pwm_right, gpio_initialized
    if not gpio_initialized:
        rospy.logerr("GPIO doesn't init!")
        return 0
    if motor is Motor.left:
        GPIO.output([GPIOPins.LEFT_FORWARD.value, GPIOPins.LEFT_BACKWARD.value], GPIO.LOW)
        pwm_left.stop()
    elif motor is Motor.right:
        GPIO.output([GPIOPins.RIGHT_FORWARD.value, GPIOPins.RIGHT_BACKWARD.value], GPIO.LOW)
        pwm_right.stop()
    else:
        rospy.logerr("Motor doesn't exists!")
        return 0
    return 1


def gpio_move(motor, direction, speed):
    global pwm_left, pwm_right, gpio_initialized
    if not gpio_initialized:
        rospy.logerr("GPIO doesn't init!")
        return 0

    if motor is Motor.left:

        if speed is Speed.low:
            pwm_left.start(PWM_LOW)
        elif speed is Speed.middle:
            pwm_left.start(PWM_MID)
        elif speed is Speed.fast:
            pwm_left.start(PWM_FAST)
        else:
            rospy.logerr("Speed doesn't exists!")
            return 0

        if direction is Direction.forward:
            GPIO.output(GPIOPins.LEFT_BACKWARD.value, GPIO.LOW)
            GPIO.output(GPIOPins.LEFT_FORWARD.value, GPIO.HIGH)

        elif direction is Direction.backward:
            GPIO.output(GPIOPins.LEFT_FORWARD.value, GPIO.LOW)
            GPIO.output(GPIOPins.LEFT_BACKWARD.value, GPIO.HIGH)

        else:
            rospy.logerr("Direction doesn't exists!")
            return 0

    elif motor is Motor.right:

        if speed is Speed.low:
            pwm_right.start(PWM_LOW)
        elif speed is Speed.middle:
            pwm_right.start(PWM_MID)
        elif speed is Speed.fast:
            pwm_right.start(PWM_FAST)
        else:
            rospy.logerr("Speed doesn't exists!")
            return 0

        if direction is Direction.forward:
            GPIO.output(GPIOPins.RIGHT_BACKWARD.value, GPIO.LOW)
            GPIO.output(GPIOPins.RIGHT_FORWARD.value, GPIO.HIGH)

        elif direction is Direction.backward:
            GPIO.output(GPIOPins.RIGHT_FORWARD.value, GPIO.LOW)
            GPIO.output(GPIOPins.RIGHT_BACKWARD.value, GPIO.HIGH)

        else:
            rospy.logerr("Direction doesn't exists!")
            return 0

    else:
        rospy.logerr("Motor doesn't exists!")
        return 0

    return 1
