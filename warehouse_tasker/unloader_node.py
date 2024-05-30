import time

import rclpy
from rclpy.node import Node

from std_srvs.srv import SetBool

import RPi.GPIO as GPIO

class UnloaderNode(Node):
    def __init__(self) -> None:
        super().__init__('unloader_node')

        self.open: bool = False
        self.pin = 18

        GPIO.setmode(GPIO.BCM)  # You can also use GPIO.BOARD
        GPIO.setup(self.pin, GPIO.OUT)

        self.door_service = self.create_service(
            srv_type=SetBool,
            srv_name='open_door',
            callback=self.service_callback
        )

    def service_callback(self, request, response):
        if request.data:
            GPIO.output(self.pin, GPIO.HIGH)
            time.sleep(1)
            GPIO.output(self.pin, GPIO.LOW)

        response.success = True
        return response

def main(args = None) -> None:
    rclpy.init(args=args)
    node = UnloaderNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
