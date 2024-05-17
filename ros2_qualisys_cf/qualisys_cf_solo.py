

import rclpy
from rclpy.node import Node
from qfly import Pose, QualisysCrazyflie, World, utils


class QualisysCrazyfliePublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        # self.cf_body_name = 'cf2'  # QTM rigid body name
        # self.cf_uri = 'radio://0/80/2M/E7E7E7E734'  # Crazyflie address
        # self.cf_marker_ids = [21, 22, 23, 24] # Active marker IDs

        self.cf_body_name = 'cf1'  # QTM rigid body name
        self.cf_uri = 'radio://0/80/2M/E7E7E7E734'  # Crazyflie address
        self.cf_marker_ids = [11, 12, 13, 14] # Active marker IDs
        self.qtm_ip = '192.168.123.2'
        # Set up world - the World object comes with sane defaults
        self.world = World()
        self.qcf = QualisysCrazyflie(self.cf_body_name,
                    self.cf_uri,
                    self.world,
                    marker_ids=self.cf_marker_ids, qtm_ip=self.qtm_ip)
        self.qcf.__enter__()
        self.timer_period = 0.005  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.dt = 0


    def timer_callback(self):
        
        self.dt += self.timer_period
        circle_radius = 0.5 # Radius of the circular flight path
        circle_speed_factor = 0.12 # How fast the Crazyflie should move along circle
        if (self.qcf.is_safe()):
            # Calculate Crazyflie's angular position in circle, based on time
            phi = circle_speed_factor * self.dt * 360
            # Take off and hover in the center of safe airspace for 5 seconds
            if self.dt < 5:
                print(f'[t={int(self.dt)}] Maneuvering - Center...')
                # Set target
                target = Pose(self.world.origin.x, self.world.origin.y, self.world.expanse)
                # Engage
                self.qcf.safe_position_setpoint(target)

            # Move out and circle around Z axis
            elif self.dt < 20:
                print(f'[t={int(self.dt)}] Maneuvering - Circle around Z...')
                # Set target
                _x, _y = utils.pol2cart(circle_radius, phi)
                target = Pose(self.world.origin.x + _x,
                            self.world.origin.y + _y,
                            self.world.expanse)
                # Engage
                self.qcf.safe_position_setpoint(target)

            # Back to center
            elif self.dt < 25:
                print(f'[t={int(self.dt)}] Maneuvering - Center...')
                # Set target
                target = Pose(self.world.origin.x, self.world.origin.y, self.world.expanse)
                # Engage
                self.qcf.safe_position_setpoint(target)

            # Move out and circle around Y axis
            elif self.dt < 40:
                print(f'[t={int(self.dt)}] Maneuvering - Circle around X...')
                # Set target
                _x, _z = utils.pol2cart(circle_radius, phi)
                target = Pose(self.world.origin.x + _x,
                            self.world.origin.y,
                            self.world.expanse + _z)
                # Engage
                self.qcf.safe_position_setpoint(target)

            # Back to center
            elif self.dt < 45:
                print(f'[t={int(self.dt)}] Maneuvering - Center...')
                # Set target
                target = Pose(self.world.origin.x, self.world.origin.y, self.world.expanse)
                # Engage
                self.qcf.safe_position_setpoint(target)

            # Move and circle around X axis
            elif self.dt < 60:
                print(f'[t={int(self.dt)}] Maneuvering - Circle around X...')
                # Set target
                _y, _z = utils.pol2cart(circle_radius, phi)
                target = Pose(self.world.origin.x,
                            self.world.origin.y + _y,
                            self.world.expanse + _z)
                # Engage
                self.qcf.safe_position_setpoint(target)

            # Back to center
            elif self.dt < 65:
                print(f'[t={int(self.dt)}] Maneuvering - Center...')
                # Set target
                target = Pose(self.world.origin.x, self.world.origin.y, self.world.expanse)
                # Engage
                self.qcf.safe_position_setpoint(target)
            else:
                # Land
                if (self.qcf.pose.z > 0.1):
                    self.qcf.land_in_place()



def main(args=None):
    rclpy.init(args=args)
    qualisys_cf_publisher = QualisysCrazyfliePublisher()
    rclpy.spin(qualisys_cf_publisher)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    qualisys_cf_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

