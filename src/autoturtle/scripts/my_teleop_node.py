#!/usr/bin/env python3
import rospy
import curses
from geometry_msgs.msg import Twist

def main():
    # Initialize the ROS node with a name
    rospy.init_node('my_teleop_node', anonymous=False)
    
    # Create a publisher on the topic /turtle1/cmd_vel with message type Twist
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    
    # Set a Rate object to regulate the loop speed
    rate = rospy.Rate(10)  # 10 Hz

    # Initialize curses for non-blocking keyboard input
    stdscr = curses.initscr()
    curses.cbreak()
    stdscr.keypad(True)
    stdscr.nodelay(True)

    # Display instructions on the terminal screen
    stdscr.addstr(0, 0, "Use keys: w-forward, s-backward, a-turn left, d-turn right. Press q to quit.")
    
    try:
        while not rospy.is_shutdown():
            # Read a keystroke; nodelay ensures getch() doesn't block execution
            key = stdscr.getch()
            move_cmd = Twist()  # Create a new Twist message for each key press

            if key == ord('w'):
                # Move forward with positive linear velocity
                move_cmd.linear.x = 0.3
                move_cmd.angular.z = 0.0
                pub.publish(move_cmd)
                stdscr.addstr(2, 0, "Moving forward    ")
            elif key == ord('s'):
                # Move backward with negative linear velocity
                move_cmd.linear.x = -0.3
                move_cmd.angular.z = 0.0
                pub.publish(move_cmd)
                stdscr.addstr(2, 0, "Moving backward   ")
            elif key == ord('a'):
                # Rotate left (counterclockwise) without forward motion
                move_cmd.linear.x = 0.0
                move_cmd.angular.z = 0.5
                pub.publish(move_cmd)
                stdscr.addstr(2, 0, "Turning left      ")
            elif key == ord('d'):
                # Rotate right (clockwise) without forward motion
                move_cmd.linear.x = 0.0
                move_cmd.angular.z = -0.5
                pub.publish(move_cmd)
                stdscr.addstr(2, 0, "Turning right     ")
            elif key == ord('q'):
                # Quit on pressing 'q'
                break

            # Sleep to maintain the loop rate
            rate.sleep()
    finally:
        # Cleanup curses settings upon exit
        curses.nocbreak()
        stdscr.keypad(False)
        curses.echo()
        curses.endwin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass