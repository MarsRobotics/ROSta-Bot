import rospy


#This class serves as the API for the sabertooth motor controllers and the drive motors they control.
class TransportDriveMotorAPI:
    FORWARD_LABEL = 1
    BACKWARD_LABEL = 2

    def __init__(self):
        # Initialize publishers for speed and direction topics.
        self.speed_publisher = rospy.Publisher("drive_speed", Char, queue_size=5)
        self.dir_publisher = rospy.Publisher("drive_direction", Char, queue_size=5)
        rospy.init_node("transport_drive_API")
        return

    # Set the intended speed to 100/127 and the direction to Backwards.
    def simple_drive_backwards(self):
        self.dir_publisher.publish(self.BACKWARD_LABEL)
        self.speed_publisher.publish(100)

    # Set the intended speed to 101/127 and the direction to Forwards.
    def simple_drive_forwards(self):
        self.dir_publisher.publish(self.FORWARD_LABEL)
        self.speed_publisher.publish(101)

    # Set the intended speed of motion to zero.
    def do_not_move(self):
        self.speed_publisher.publish(0)