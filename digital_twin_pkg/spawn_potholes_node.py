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
        self.declare_parameter('number_of_potholes', 250)

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
            'static.prop.brokentile02',
            'static.prop.brokentile03',
            'static.prop.brokentile04',
        ]

        # BrokenTile propları için hedef boyut.
        # CARLA blueprint 'size' attribute'u: 'tiny', 'small', 'medium', 'big', 'huge'
        brokentile_target_size = 'huge'
        
        carla_map = self.world.get_map()
        spawn_points = carla_map.generate_waypoints(distance=2.0)
        
        if not spawn_points:
            self.get_logger().error('No spawn points (waypoints) found in the map.')
            return

        # BrokenTile size attribute bilgisini logla (ilk seferde)
        sample_bp = blueprint_library.find('static.prop.brokentile01')
        if sample_bp.has_attribute('size'):
            size_attr = sample_bp.get_attribute('size')
            self.get_logger().info(
                f'BrokenTile size attribute — '
                f'current: "{size_attr.as_str()}", '
                f'modifiable: {size_attr.is_modifiable}, '
                f'recommended: {size_attr.recommended_values}')
        
        self.get_logger().info(f'Attempting to spawn {self.num_potholes} potholes/debris...')
        
        count = 0
        for _ in range(self.num_potholes):
            wp = random.choice(spawn_points)
            bp_name = random.choice(damage_bps)
            bp = blueprint_library.find(bp_name)

            # BrokenTile proplarının size attribute'unu büyüt
            if 'brokentile' in bp_name and bp.has_attribute('size'):
                try:
                    bp.set_attribute('size', brokentile_target_size)
                except RuntimeError as e:
                    # İlk hatada bir kez logla, spawn'a devam et
                    if count == 0:
                        self.get_logger().warn(
                            f'size attribute set edilemedi: {e}')
            
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
