#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from pyfirmata import Arduino, OUTPUT, PWM

# Define Arduino connection (Make sure the port is correct)
PORT = "/dev/ttyUSB1"  # Adjust if needed (check using `ls /dev/tty*`)
board = Arduino(PORT)

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

# Define maximum speed (tune according to your motor's capability)
MAX_SPEED = 1.0  # Adjust between 0 and 1 (1 = full speed)

def motor_control(cmd_vel):
    """
    Callback function to control motors based on ROS cmd_vel messages.
    """
    linear_speed = cmd_vel.linear.x*2  # Forward/Backward Speed
    angular_speed = cmd_vel.angular.z*2 # Rotation Speed

    # Convert linear speed to PWM (Ensure it's within [0,1])
    pwm_value = min(abs(linear_speed), MAX_SPEED)

    # Determine movement direction
    if linear_speed > 0:  # Move Forward
        board.digital[IN1].write(1)
        board.digital[IN2].write(0)
        board.digital[IN3].write(1)
        board.digital[IN4].write(0)
    elif linear_speed < 0:  # Move Backward
        board.digital[IN1].write(0)
        board.digital[IN2].write(1)
        board.digital[IN3].write(0)
        board.digital[IN4].write(1)
    elif angular_speed > 0:  # Turn Left
        board.digital[IN1].write(0)
        board.digital[IN2].write(1)
        board.digital[IN3].write(1)
        board.digital[IN4].write(0)
    elif angular_speed < 0:  # Turn Right
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
    """
    ROS node to listen for cmd_vel messages from navigation stack.
    """
    rospy.init_node("motor_control_node", anonymous=True)
    rospy.Subscriber("/cmd_vel", Twist, motor_control)
    rospy.loginfo("Motor Controller Node Started - Waiting for /cmd_vel commands...")
    rospy.spin()

if __name__ == "__main__":
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
