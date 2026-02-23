#!/bin/bash
timeout -s INT 6 ros2 topic hz /carla/ego_vehicle/rgb_front/image > hz_rgb.txt &
timeout -s INT 6 ros2 topic bw /carla/ego_vehicle/rgb_front/image > bw_rgb.txt &
timeout -s INT 6 ros2 topic hz /carla/ego_vehicle/rgb_view/image > hz_view.txt &
timeout -s INT 6 ros2 topic bw /carla/ego_vehicle/rgb_view/image > bw_view.txt &
timeout -s INT 6 ros2 topic hz /carla/ego_vehicle/lidar_top > hz_lidar.txt &
timeout -s INT 6 ros2 topic hz /carla/ego_vehicle/imu > hz_imu.txt &
top -b -n 2 -d 3 -w 150 | head -n 25 > top_head.txt &
top -b -n 2 -d 3 -w 150 | grep -E "python|carla|foxglove" > cpu_usage.txt &
wait

echo "=== RGB FRONT HZ ==="
cat hz_rgb.txt
echo "=== RGB FRONT BW ==="
cat bw_rgb.txt
echo "=== RGB VIEW HZ ==="
cat hz_view.txt
echo "=== RGB VIEW BW ==="
cat bw_view.txt
echo "=== LIDAR HZ ==="
cat hz_lidar.txt
echo "=== IMU HZ ==="
cat hz_imu.txt
echo "=== CPU HEAD ==="
cat top_head.txt
echo "=== CPU USAGE ==="
cat cpu_usage.txt
