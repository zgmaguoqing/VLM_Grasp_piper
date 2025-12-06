#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity

# ================= 修改了这里 =================
# 1. z 改为 0.3
# 2. Pitch 改为 0.6 (低头约35度，否则画面里只有地板)
RGBD_SDF = """<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='external_rgbd_camera'>
    <static>true</static>
    <pose>0.5 0.0 0.3 0 0.6 3.14159</pose>

    <link name='rgbd_link'>
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>1.0</ixx><iyy>1.0</iyy><izz>1.0</izz>
          <ixy>0.0</ixy><ixz>0.0</ixz><iyz>0.0</iyz>
        </inertia>
      </inertial>

      <visual name='visual'>
        <geometry>
          <box>
            <size>0.05 0.05 0.05</size>
          </box>
        </geometry>
        <material>
          <ambient>0.0 0.0 1.0 1</ambient>
        </material>
      </visual>

      <sensor name='rgb_camera' type='camera'>
        <always_on>true</always_on>
        <update_rate>30.0</update_rate>
        <visualize>true</visualize>
        <pose>0 0 0 0 0 0</pose>
        <camera>
          <horizontal_fov>1.047</horizontal_fov>
          <image>
            <width>640</width>
            <height>480</height>
            <format>R8G8B8</format>
          </image>
          <clip>
            <near>0.01</near>
            <far>10.0</far>
          </clip>
        </camera>
        <plugin name='gazebo_ros_rgb_camera' filename='libgazebo_ros_camera.so'>
          <ros>
            <namespace>/sim_cam</namespace>
            <remapping>image_raw:=/sim_cam/rgb_camera/image_raw</remapping>
            <remapping>camera_info:=/sim_cam/rgb_camera/camera_info</remapping>
          </ros>
          <frame_name>rgb_camera_link</frame_name>
        </plugin>
      </sensor>

      <sensor name='my_depth_camera' type='depth'>
        <always_on>true</always_on>
        <update_rate>30.0</update_rate>
        <visualize>true</visualize>
        <pose>0 0 0 0 0 0</pose>
        <camera>
          <horizontal_fov>1.047</horizontal_fov>
          <image>
            <width>640</width>
            <height>480</height>
            <format>R8G8B8</format>
          </image>
          <clip>
            <near>0.01</near>
            <far>10.0</far>
          </clip>
        </camera>
        <plugin name='gazebo_ros_depth_camera' filename='libgazebo_ros_camera.so'>
          <ros>
            <namespace>/sim_cam</namespace>
            <remapping>image_raw:=/sim_cam/my_camera/depth/image_raw</remapping>
            <remapping>camera_info:=/sim_cam/my_camera/depth/camera_info</remapping>
          </ros>
          <frame_name>depth_camera_link</frame_name>
        </plugin>
      </sensor>
    </link>
  </model>
</sdf>
"""

class RGBDCameraSpawner(Node):
    def __init__(self):
        super().__init__('rgbd_camera_spawner')
        self.cli = self.create_client(SpawnEntity, '/spawn_entity')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 Gazebo /spawn_entity 服务中...')
        
        req = SpawnEntity.Request()
        req.name = 'external_rgbd_camera'
        req.xml = RGBD_SDF
        req.reference_frame = "world"

        self.get_logger().info('发送 RGBD 相机模型到 Gazebo...')
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info('✅ RGBD 相机已生成！')
            # 更新日志信息
            self.get_logger().info('   位置: x=0.5, z=0.3 (俯视角度)')
        else:
            self.get_logger().error('❌ RGBD 相机生成失败！')
        
        rclpy.shutdown()

def main():
    rclpy.init()
    RGBDCameraSpawner()

if __name__ == '__main__':
    main()