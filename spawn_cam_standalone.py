import rclpy
from gazebo_msgs.srv import SpawnEntity

def main():
    rclpy.init()
    node = rclpy.create_node('camera_spawner')
    
    client = node.create_client(SpawnEntity, '/spawn_entity')
    while not client.wait_for_service(timeout_sec=1.0):
        print('等待 Gazebo /spawn_entity 服务...')

    camera_sdf = """<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='external_camera'>
    <static>true</static>
    <!-- 相机相对于世界/机械臂的位置，自己可以微调 -->
    <pose>0.85 0.8 0.1 0 0 3.14159</pose>

    <link name='camera_link'>
      <visual name='visual'>
        <geometry>
          <box><size>0.05 0.05 0.05</size></box>
        </geometry>
      </visual>

      <!-- 注意：这里的 type 直接用 camera，插件负责把深度也发出来 -->
      <sensor name='sim_cam' type='camera'>
        <always_on>true</always_on>
        <update_rate>30</update_rate>
        <pose>0 0 0 0 0 0</pose>
        <camera>
          <horizontal_fov>1.047</horizontal_fov>
          <image>
            <width>640</width>
            <height>480</height>
            <format>R8G8B8</format>  <!-- 彩色图格式 -->
          </image>
          <clip>
            <near>0.1</near>
            <far>5.0</far>
          </clip>
        </camera>

        <!-- 关键：用 gazebo_ros_camera 插件，同时发 RGB + Depth -->
        <plugin name='sim_cam_plugin' filename='libgazebo_ros_camera.so'>
          <ros>
            <namespace>sim_cam</namespace>

            <!-- 彩色图 -->
            <remapping>image_raw:=/sim_cam/rgb_camera/image_raw</remapping>
            <remapping>camera_info:=/sim_cam/rgb_camera/camera_info</remapping>

            <!-- 深度图 -->
            <remapping>depth/image_raw:=/sim_cam/my_camera/depth/image_raw</remapping>
            <remapping>depth/camera_info:=/sim_cam/my_camera/depth/camera_info</remapping>
          </ros>

          <!-- 相机名：内部生成的 topic 前缀，用于 depth/image_raw 等 -->
          <camera_name>my_camera</camera_name>
        </plugin>
      </sensor>

    </link>
  </model>
</sdf>
"""

    req = SpawnEntity.Request()
    req.name = 'external_camera'
    req.xml = camera_sdf
    
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    print("✅ 外部 RGB+Depth 摄像机已生成！")
    print("  RGB 话题:   /sim_cam/rgb_camera/image_raw")
    print("  深度话题:   /sim_cam/my_camera/depth/image_raw")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
