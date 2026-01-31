#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from pyfirmata import Arduino, OUTPUT, PWM

# Define Arduino connection (Make sure the port is correct)
board = Arduino('/dev/ttyUSB1')  # Change this if needed (check using ls /dev/tty* on Linux)

# Define motor control pins
ENA = 9   # Speed Control Motor A (PWM)
IN1 = 4   # Motor A Direction
IN2 = 5   # Motor A Direction
ENB = 10  # Speed Control Motor B (PWM)
IN3 = 6   # Motor B Direction
IN4 = 7   # Motor B Direction

# Set direction control pins as OUTPUT
for pin in [IN1, IN2, IN3, IN4]:
    board.digital[pin].mode = OUTPUT

# Set speed control pins as PWM
board.digital[ENA].mode = PWM
board.digital[ENB].mode = PWM

def motor_control(data):
    speed = abs(data.linear.x)  # Extract speed (0 to 1 expected)
    turn = data.angular.z      # Extract turning direction (-1 to 1)

    pwm_value = speed  # PWM expects values between 0 and 1
    if(speed == 0): 
     pwm_value = abs(turn)

    # Determine movement direction
    if data.linear.x > 0:  # Move Forward
        board.digital[IN1].write(1)
        board.digital[IN2].write(0)
        board.digital[IN3].write(1)
        board.digital[IN4].write(0)
    elif data.linear.x < 0:  # Move Backward
        board.digital[IN1].write(0)
        board.digital[IN2].write(1)
        board.digital[IN3].write(0)
        board.digital[IN4].write(1)
    elif turn < 0:  # Turn Left
        board.digital[IN1].write(0)
        board.digital[IN2].write(1)
        board.digital[IN3].write(1)
        board.digital[IN4].write(0)
    elif turn > 0:  # Turn Right
        board.digital[IN1].write(1)
        board.digital[IN2].write(0)
        board.digital[IN3].write(0)
        board.digital[IN4].write(1)
    else:  # Stop Motors
        board.digital[IN1].write(0)
        board.digital[IN2].write(0)
        board.digital[IN3].write(0)
        board.digital[IN4].write(0)

    # Set motor speeds using PWM (Ensure value is between 0 and 1)
    board.digital[ENA].write(pwm_value)  # Speed for Motor A
    board.digital[ENB].write(pwm_value)  # Speed for Motor B

def listener():
    rospy.init_node('motor_control_node', anonymous=True)
    rospy.Subscriber("/cmd_vel", Twist, motor_control)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
