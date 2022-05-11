from mavros_msgs.msg import OverrideRCIn
from mavros_wrapper.mavros_wrapper import *

class BlueROVArduSubWrapper(MavrosWrapper):
    def __init__(self, node_name='bluerov_mavros'):
        super().__init__(node_name)

        self.rc_override_pub  = self.create_publisher(OverrideRCIn,
            'mavros/rc/override', 10)

        self.override_timer = None

        self.pitch=0
        self.roll=0
        self.throttle=0
        self.yaw=0
        self.forward=0
        self.lateral=0
        self.camera_pan=0
        self.camera_tilt=0
        self.light_level1=0
        self.light_level2=0
        self.video_switch=0

    def rc_override_publish_cb(self):
        # convert normalized value(-1 to 1) to a pwm value(1100 to 1900)
        normalized_to_pwm = lambda x: int((1900-1500)*x + 1500)
        channels = [normalized_to_pwm(self.pitch),
                    normalized_to_pwm(self.roll),
                    normalized_to_pwm(self.throttle),
                    normalized_to_pwm(self.yaw),
                    normalized_to_pwm(self.forward),
                    normalized_to_pwm(self.lateral),
                    normalized_to_pwm(self.camera_pan),
                    normalized_to_pwm(self.camera_tilt),
                    normalized_to_pwm(self.light_level1),
                    normalized_to_pwm(self.light_level2),
                    normalized_to_pwm(self.video_switch),
                    0, 0, 0, 0, 0, 0, 0]

        override_msg = OverrideRCIn()
        override_msg.channels = channels

        self.rc_override_pub.publish(override_msg)

    # arguments should be a normalized float value ranging from -1 to 1
    # 0 is translated to 1500 pwm, -1 to 1100, and 1 to 1900
    def set_rc_override_channels(self, pitch=0, roll=0, throttle=0, yaw=0,
                    forward=0, lateral=0, camera_pan=0,
                    camera_tilt=0, light_level1=0, light_level2=0,
                    video_switch=0):
        self.pitch=pitch
        self.roll=roll
        self.throttle=throttle
        self.yaw=yaw
        self.forward=forward
        self.lateral=lateral
        self.camera_pan=camera_pan
        self.camera_tilt=camera_tilt
        self.light_level1=light_level1
        self.light_level2=light_level2
        self.video_switch=video_switch

    def toogle_rc_override(self, run_boolean):
        if run_boolean and self.override_timer == None:
            timer_period = 0.5  # seconds
            self.override_timer = self.create_timer(timer_period,
                self.rc_override_publish_cb)
        elif run_boolean and self.override_timer.is_canceled():
            self.override_timer.reset()
        elif not run_boolean and not self.override_timer.is_canceled():
            self.override_timer.cancel()
