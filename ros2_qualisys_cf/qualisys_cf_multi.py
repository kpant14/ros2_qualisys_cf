

import time
import rclpy
from rclpy.node import Node
from qfly import Pose, QualisysCrazyflie, World, utils, ParallelContexts


class QualisysCrazyfliePublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        
        self.cf_body_names = ['cf1','cf2', 'cf3', 'cf4']  # QTM rigid body name
        self.cf_uris = ['radio://0/80/2M/E7E7E7E701',
                        'radio://0/80/2M/E7E7E7E702',
                        'radio://1/80/2M/E7E7E7E703',
                        'radio://1/80/2M/E7E7E7E704'
                        ]  # Crazyflie address
        self.cf_marker_ids = [[11, 12, 13, 14],
                              [21, 22, 23, 24],
                              [31, 32, 33, 34],
                              [41, 42, 43, 44]] # Active marker IDs
        
        # self.cf_body_names = ['cf1','cf2']  # QTM rigid body name
        # self.cf_uris = ['radio://0/80/2M/E7E7E7E701',
        #                 'radio://1/80/2M/E7E7E7E702',
        #                 ]  # Crazyflie address
        # self.cf_marker_ids = [[11, 12, 13, 14],
        #                       [21, 22, 23, 24]] # Active marker IDs
        
        self.qtm_ip = '192.168.123.2'
        # Set up world - the World object comes with sane defaults
        self.world = World(expanse=3.0)
        # Stack up context managers
        self.qcfs_ = [QualisysCrazyflie(cf_body_name,
                                cf_uri,
                                self.world,
                                marker_ids=cf_marker_id, qtm_ip='192.168.123.2')
                for cf_body_name, cf_uri, cf_marker_id
                in zip(self.cf_body_names, self.cf_uris, self.cf_marker_ids)]
        self.qcfs = ParallelContexts(*self.qcfs_)
        self.qcfs = self.qcfs.__enter__()
        self.timer_period = 0.05  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.dt = 0


    def timer_callback(self):
        if (all(qcf.is_safe() for qcf in self.qcfs)):
            #Cycle all drones
            for idx, qcf in enumerate(self.qcfs): 
                if idx == 0:   
                    target = Pose(0.5,
                                0.0,
                                0.5)
                    # Engage
                    qcf.safe_position_setpoint(target)
                    qcf.set_led_ring(7)
                    qcf.cf.param.set_value('ring.solidGreen', 200)

                elif idx == 1:
                    target = Pose(0.5,
                                0.5,
                                0.5)
                    # Engage
                    qcf.safe_position_setpoint(target)
                    qcf.set_led_ring(7)
                    qcf.cf.param.set_value('ring.solidGreen', 200)
                elif idx == 2:
                    target = Pose(0.5,
                                -0.5,
                                0.5)
                    # Engage
                    qcf.safe_position_setpoint(target)
                    qcf.set_led_ring(7)
                    qcf.cf.param.set_value('ring.solidGreen', 200)
                elif idx == 3:
                    target = Pose(0.5,
                                -1.0,
                                0.5)
                    # Engage
                    qcf.safe_position_setpoint(target)    
                    qcf.set_led_ring(7)
                    qcf.cf.param.set_value('ring.solidGreen', 200)
    
        else:
            # Land
            for idx, qcf in enumerate(self.qcfs):
                qcf.land_in_place()
        #print('yess')



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

