"""EPuckGoForward controller."""

from controller import Robot, Motor

TIME_STEP = 32

MAX_SPEED = 6.28

# create the Robot instance.
robot = Robot()

# get a handler to the motors and set target position to infinity (speed control)
leftMotor = robot.getMotor('left wheel motor')
rightMotor = robot.getMotor('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))

# set up the motor speeds at 10% of the MAX_SPEED.
leftMotor.setVelocity(0.3 * MAX_SPEED)
rightMotor.setVelocity(0.3 * MAX_SPEED)

while robot.step(TIME_STEP) != -1:
   pass