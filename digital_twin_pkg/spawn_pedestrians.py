#!/usr/bin/env python3
"""
CARLA Pedestrian Spawner Node for ROS 2.

Spawns AI-controlled walkers into the CARLA world. Works correctly
with the CARLA ROS Bridge in synchronous mode.

Critical implementation detail:
  In CARLA synchronous mode, the spawning client MUST keep calling
  world.wait_for_tick() continuously for AI walkers to actually move.
  This script uses its own main loop that interleaves CARLA tick waits
  with ROS 2 spinning, rather than relying solely on rclpy.spin().
"""

import rclpy
from rclpy.node import Node
import carla
import random
import time


class SpawnPedestriansNode(Node):
    def __init__(self):
        super().__init__('spawn_pedestrians')

        self.declare_parameter('host', '127.0.0.1')
        self.declare_parameter('port', 2000)
        self.declare_parameter('number_of_walkers', 100)
        self.declare_parameter('percentage_pedestrians_crossing', 0.3)
        self.declare_parameter('percentage_pedestrians_running', 0.1)

        host = self.get_parameter('host').value
        port = self.get_parameter('port').value
        self.number_of_walkers = self.get_parameter('number_of_walkers').value
        self.crossing_factor = self.get_parameter('percentage_pedestrians_crossing').value
        self.running_factor = self.get_parameter('percentage_pedestrians_running').value

        self.get_logger().info(
            f'Connecting to CARLA at {host}:{port} — '
            f'requesting {self.number_of_walkers} walkers '
            f'(crossing={self.crossing_factor}, running={self.running_factor})')

        self.client = carla.Client(host, port)
        self.client.set_timeout(20.0)

        self.world = self.client.get_world()
        self.walkers_list = []   # [{"id": ..., "con": ...}, ...]
        self.all_id = []         # [controller_id, walker_id, ...]

        self._spawn_pedestrians()

    # ------------------------------------------------------------------
    # Spawning
    # ------------------------------------------------------------------
    def _spawn_pedestrians(self):
        blueprints_walkers = self.world.get_blueprint_library().filter(
            'walker.pedestrian.*')
        controller_bp = self.world.get_blueprint_library().find(
            'controller.ai.walker')

        if not blueprints_walkers:
            self.get_logger().error('No walker blueprints found!')
            return

        # --- 1. Collect spawn locations (1.5× to absorb collisions) ---
        num_requested = int(self.number_of_walkers * 1.5)
        spawn_points = []
        for _ in range(num_requested):
            loc = self.world.get_random_location_from_navigation()
            if loc is not None:
                sp = carla.Transform()
                sp.location = loc
                spawn_points.append(sp)

        self.get_logger().info(
            f'Collected {len(spawn_points)} candidate spawn points.')

        # --- 2. Batch-spawn walker actors ---
        batch = []
        walker_speed = []
        for spawn_point in spawn_points:
            bp = random.choice(blueprints_walkers)
            if bp.has_attribute('is_invincible'):
                bp.set_attribute('is_invincible', 'false')
            if bp.has_attribute('speed'):
                if random.random() > self.running_factor:
                    walker_speed.append(
                        bp.get_attribute('speed').recommended_values[1])
                else:
                    walker_speed.append(
                        bp.get_attribute('speed').recommended_values[2])
            else:
                walker_speed.append(0.0)
            batch.append(carla.command.SpawnActor(bp, spawn_point))

        results = self.client.apply_batch_sync(batch, True)

        walker_speed_ok = []
        for i, res in enumerate(results):
            if not res.error:
                self.walkers_list.append({'id': res.actor_id})
                walker_speed_ok.append(walker_speed[i])
                if len(self.walkers_list) >= self.number_of_walkers:
                    break

        walker_speed = walker_speed_ok
        self.get_logger().info(
            f'Spawned {len(self.walkers_list)} walker actors.')

        if not self.walkers_list:
            self.get_logger().error('No walkers could be spawned!')
            return

        # Wait ticks for walkers to be registered
        for _ in range(5):
            self.world.wait_for_tick()

        # --- 3. Batch-spawn AI controllers ---
        batch = []
        for w in self.walkers_list:
            batch.append(carla.command.SpawnActor(
                controller_bp, carla.Transform(), w['id']))

        results = self.client.apply_batch_sync(batch, True)
        for i, res in enumerate(results):
            if res.error:
                self.get_logger().warn(f'Controller error: {res.error}')
            else:
                self.walkers_list[i]['con'] = res.actor_id

        # Build [controller, walker, ...] list — only valid pairs
        valid_walkers = []
        valid_speeds = []
        for i, w in enumerate(self.walkers_list):
            if 'con' in w:
                self.all_id.append(w['con'])
                self.all_id.append(w['id'])
                valid_walkers.append(w)
                valid_speeds.append(walker_speed[i])

        self.walkers_list = valid_walkers
        self.walker_speed = valid_speeds

        self.get_logger().info(
            f'{len(self.walkers_list)} walkers with controllers ready.')

        if not self.all_id:
            self.get_logger().error('No walker-controller pairs!')
            return

        # --- 4. Wait for ticks so controllers register ---
        for _ in range(10):
            self.world.wait_for_tick()

        # --- 5. Set crossing factor ---
        self.world.set_pedestrians_cross_factor(self.crossing_factor)

        # --- 6. Start controllers ---
        all_actors = self.world.get_actors(self.all_id)
        for i in range(0, len(self.all_id), 2):
            try:
                all_actors[i].start()
            except RuntimeError as e:
                self.get_logger().warn(f'Controller start error: {e}')

        # Wait ticks after start() before setting targets
        for _ in range(5):
            self.world.wait_for_tick()

        # --- 7. Set destinations and speeds ---
        for i in range(0, len(self.all_id), 2):
            try:
                dest = self.world.get_random_location_from_navigation()
                all_actors[i].go_to_location(dest)
                all_actors[i].set_max_speed(
                    float(self.walker_speed[int(i / 2)]))
            except RuntimeError as e:
                self.get_logger().warn(f'Target error: {e}')

        self.get_logger().info(
            f'All {len(self.walkers_list)} walkers are now moving! '
            f'Press Ctrl+C to stop.')

    # ------------------------------------------------------------------
    # Tick loop (keeps walkers alive)
    # ------------------------------------------------------------------
    def tick(self):
        """Call world.wait_for_tick() — essential to keep AI walkers moving."""
        try:
            self.world.wait_for_tick()
        except Exception:
            pass

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------
    def destroy_pedestrians(self):
        """Stop controllers and destroy all walker + controller actors."""
        try:
            if not self.all_id:
                return
            self.get_logger().info('Stopping AI controllers...')
            actors = self.world.get_actors(self.all_id)
            for i in range(0, len(self.all_id), 2):
                try:
                    actors[i].stop()
                except Exception:
                    pass
            self.get_logger().info(
                f'Destroying {len(self.walkers_list)} walkers + controllers...')
            self.client.apply_batch(
                [carla.command.DestroyActor(x) for x in self.all_id])
            self.all_id.clear()
            self.walkers_list.clear()
            self.get_logger().info('Pedestrians destroyed.')
        except Exception as e:
            self.get_logger().error(f'Cleanup error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = SpawnPedestriansNode()
    try:
        # Main loop: interleave CARLA ticks with ROS 2 spinning.
        # rclpy.spin() alone would starve the CARLA client.
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0)
            node.tick()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_pedestrians()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
