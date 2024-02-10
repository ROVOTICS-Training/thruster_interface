import rclpy
from rclpy.node import Node
from core_lib import pca9685
from geometry_msgs.msg import Twist
from time import sleep

class thrusters(Node):
    def __init__(self):
        super().__init__('thrusters')
        self.subscription = self.create_subscription(Twist, 'converted_vectors', self.callback, 10)
        self.logger = self.get_logger()
        self.check = False
        try:
            self.thrusters = pca9685.PCA9685(bus=1)
            self.thrusters.set_pwm_frequency(100)
            self.thrusters.output_enable()
            self.thrusters.channels_set_duty_all(0.15)
            self.check = True
            sleep(1)
        except:
            self.logger.info('no thrusters found')
        

    def callback(self, msg):
        angx = msg.angular.x
        angz = msg.angular.z
        linx = msg.linear.x
        liny = msg.linear.y
        linz = msg.linear.z
        if self.check == True:
            self.thrusters.channels_set_duty(0, 0.15 - (linx-liny-angz) / 25)
            self.thrusters.channels_set_duty(1, 0.15 - (linx+liny+angz) / 25)
            self.thrusters.channels_set_duty(2, 0.15 - (angz-linx-liny) / 25)
            self.thrusters.channels_set_duty(3, 0.15 - (liny-linx-angz) / 25)
            self.thrusters.channels_set_duty(4, 0.15 - (-1*linz-angx) / 25)
            self.thrusters.channels_set_duty(5, 0.15 - (-1*linz+angx) / 25)
        else:
            self.logger.info(str(0.15 - (linx-liny-angz) / 25))
            self.logger.info(str(0.15 - (linx+liny+angz) / 25))
            self.logger.info(str(0.15 - (angz-linx-liny) / 25))
            self.logger.info(str(0.15 - (liny-linx-angz) / 25))
            self.logger.info(str(0.15 - (-1*linz-angx) / 25))
            self.logger.info(str(0.15 - (-1*linz+angx) / 25))
            self.logger.info('\n')



def main(args=None):
    rclpy.init()
    thruster = thrusters()
    rclpy.spin(thruster)
    
    thruster.destroy_node()
    rclpy.shutdown()



if __name__ == 'main':
    main()
