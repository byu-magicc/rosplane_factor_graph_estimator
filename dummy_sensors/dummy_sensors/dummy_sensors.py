#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import random

from rosflight_msgs.msg import Airspeed, Barometer, Status, GNSS
from sensor_msgs.msg import Imu


class DummyRandomSensors(Node):
    def __init__(self):
        super().__init__('dummy_random_sensors')

        self.enable_imu = False

        # Storage for random walk
        self.baro_mu = 85000.0
        self.airspeed_mu = 100.0
        self.gps_mu = [0.0, 0.0]

        # Publishers
        self.pub_imu = self.create_publisher(Imu, '/imu/data', 10)
        self.pub_baro = self.create_publisher(Barometer, '/baro', 10)
        self.pub_gps = self.create_publisher(GNSS, '/gnss', 10)
        self.pub_airspeed = self.create_publisher(Airspeed, '/airspeed', 10)
        self.pub_status = self.create_publisher(Status, '/status', 10)

        # Timers (Hz → period = 1/rate)
        self.create_timer(0.01, self.publish_imu)       # 100 Hz
        self.create_timer(0.01, self.publish_baro)      # 100 Hz
        self.create_timer(0.1, self.publish_gps)        # 10 Hz
        self.create_timer(0.01, self.publish_airspeed)  # 100 Hz
        self.create_timer(0.1, self.publish_status)    # 10 Hz

    def publish_imu(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()

        if self.enable_imu:
            msg.angular_velocity.x = random.uniform(-0.1, 0.1)
            msg.angular_velocity.y = random.uniform(-0.1, 0.1)
            msg.angular_velocity.z = random.uniform(-0.1, 0.1)

            msg.linear_acceleration.x = random.uniform(-1.0, 1.0)
            msg.linear_acceleration.y = random.uniform(-1.0, 1.0)
            msg.linear_acceleration.z = random.uniform(-10.0, -9.0)  # gravity-ish

        self.pub_imu.publish(msg)

    def publish_baro(self):
        if self.baro_mu > 86000.0:
            self.baro_mu = 85000.0
        else:
            self.baro_mu += 0.1 + random.gauss(0, 1)

        msg = Barometer()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pressure = 85000 + random.gauss(self.baro_mu, 5)
        self.pub_baro.publish(msg)

    def publish_gps(self):
        self.gps_mu[0] += 1 + random.uniform(-0.001, 0.001)
        self.gps_mu[1] += 0.05 + random.uniform(-0.001, 0.001)

        msg = GNSS()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.fix_type = 3
        msg.lat = 40.362 + random.gauss(0.0, 0.001)
        msg.lon = -111.904 + random.gauss(0.0, 0.001)
        msg.alt = 1450.0 + random.gauss(0.0, 5.0)
        msg.vel_n = 10 + random.gauss(0.0, 0.2)
        msg.vel_e = 0 + random.gauss(0.0, 0.2)
        msg.vel_d = 0 + random.gauss(0.0, 0.2)

        self.pub_gps.publish(msg)

    def publish_airspeed(self):
        if 100.0 < self.airspeed_mu < 120.0:
            self.airspeed_mu += 0.01 + random.gauss(0, 0.1)
        else:
            self.airspeed_mu = 101.0

        msg = Airspeed()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.differential_pressure = self.airspeed_mu + random.gauss(5)
        self.pub_airspeed.publish(msg)

    def publish_status(self):
        msg = Status()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.armed = True
        self.pub_status.publish(msg)


def main():
    rclpy.init()
    node = DummyRandomSensors()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

