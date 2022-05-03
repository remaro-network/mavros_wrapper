import rospy
from mavros_msgs.msg import OverrideRCIn
from mavros_wrapper.mavros_wrapper import *

# I would say that it more of a bluerov ardusub wrapper
class ArduSubWrapper(MavrosWrapper):
    def __init__(self):
        super().__init__()

    # arguments should be a normalized float value ranging from -1 to 1
    # 0 is translated to 1500 pwm, -1 to 1100, and 1 to 1900
    def rc_overide(self, pitch=0, roll=0, throttle=0, yaw=0,
                    forward=0, lateral=0, camera_pan=0,
                    camera_tilt=0, light_level1=0, light_level2=0,
                    video_switch=0):

        rc_overide_pub = rospy.Publisher(
            'mavros/rc/override',
            OverrideRCIn,
            queue_size = 10,
            latch=False)

        # convert normalized value(-1 to 1) to a pwm value(1100 to 1900)
        normalized_to_pwm = lambda x: int((1900-1500)*x + 1500)
        channels = [normalized_to_pwm(pitch),
                    normalized_to_pwm(roll),
                    normalized_to_pwm(throttle),
                    normalized_to_pwm(yaw),
                    normalized_to_pwm(forward),
                    normalized_to_pwm(lateral),
                    normalized_to_pwm(camera_pan),
                    normalized_to_pwm(camera_tilt),
                    normalized_to_pwm(light_level1),
                    normalized_to_pwm(light_level2),
                    normalized_to_pwm(video_switch),
                    0, 0, 0, 0, 0, 0, 0]

        override_msg = OverrideRCIn()
        override_msg.channels = channels

        rc_overide_pub.publish(override_msg)
