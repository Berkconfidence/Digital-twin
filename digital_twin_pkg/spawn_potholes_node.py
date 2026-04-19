#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import carla
import random

class PotholeSpawner(Node):
    def __init__(self):
        super().__init__('spawn_potholes_node')
        
        self.declare_parameter('host', 'localhost')
        self.declare_parameter('port', 2000)
        self.declare_parameter('number_of_potholes', 100)

        host = self.get_parameter('host').value
        port = self.get_parameter('port').value
        self.num_potholes = self.get_parameter('number_of_potholes').value

        self.get_logger().info(f'Connecting to CARLA at {host}:{port}')
        
        try:
            self.client = carla.Client(host, port)
            self.client.set_timeout(10.0)
            self.world = self.client.get_world()
            self.get_logger().info('Successfully connected to CARLA.')
            
            self.spawn_potholes()
            
        except Exception as e:
            self.get_logger().error(f'Failed to connect to CARLA or spawn potholes: {e}')
            raise

    def spawn_potholes(self):
        blueprint_library = self.world.get_blueprint_library()
        
        damage_bps = [
            'static.prop.dirtdebris01', 
            'static.prop.dirtdebris02', 
            'static.prop.dirtdebris03',
            'static.prop.brokentile01',
            'static.prop.brokentile02'
        ]
        
        map = self.world.get_map()
        spawn_points = map.generate_waypoints(distance=2.0)
        
        if not spawn_points:
            self.get_logger().error('No spawn points (waypoints) found in the map.')
            return

        self.get_logger().info(f'Attempting to spawn {self.num_potholes} potholes/debris...')
        
        count = 0
        for _ in range(self.num_potholes):
            wp = random.choice(spawn_points)
            bp_name = random.choice(damage_bps)
            bp = blueprint_library.find(bp_name)
            
            transform = wp.transform
            transform.location.z += 0.05
            
            actor = self.world.try_spawn_actor(bp, transform)
            if actor:
                actor.set_simulate_physics(False)
                count += 1

        self.get_logger().info(f'Successfully spawned {count} road damage props (potholes).')

def main(args=None):
    rclpy.init(args=args)
    pothole_spawner = PotholeSpawner()
    
    try:
        rclpy.spin(pothole_spawner)
    except KeyboardInterrupt:
        pass
    finally:
        pothole_spawner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
