import os
import sys
import rclpy
from gazebo_msgs.srv import SpawnEntity

def main():
    rclpy.init()
    node = rclpy.create_node('entity_spawner')

    print("正在等待 /spawn_entity 服务...")
    client = node.create_client(SpawnEntity, '/spawn_entity')
    while not client.wait_for_service(timeout_sec=1.0):
        print('服务未就绪，继续等待...')

    # 定义一个简单的红色方块 (SDF格式)
    # 尺寸 3cm x 3cm x 10cm (便于抓取)
    # 位置 x=0.2, y=0.0 (在机械臂正前方 20cm 处)
    cube_xml = """
    <?xml version='1.0'?>
    <sdf version='1.6'>
      <model name='test_cube'>
        <pose>0.2 0.0 0.05 0 0 0</pose>
        <link name='link'>
          <inertial>
            <mass>0.1</mass>
            <inertia>
              <ixx>0.0001</ixx><ixy>0</ixy><ixz>0</ixz>
              <iyy>0.0001</iyy><iyz>0</iyz><izz>0.0001</izz>
            </inertia>
          </inertial>
          <visual name='visual'>
            <geometry>
              <box>
                <size>0.03 0.03 0.1</size>
              </box>
            </geometry>
            <material>
              <script>
                <uri>file://media/materials/scripts/gazebo.material</uri>
                <name>Gazebo/Red</name>
              </script>
            </material>
          </visual>
          <collision name='collision'>
            <geometry>
              <box>
                <size>0.03 0.03 0.1</size>
              </box>
            </geometry>
          </collision>
        </link>
      </model>
    </sdf>
    """

    request = SpawnEntity.Request()
    request.name = 'test_cube'
    request.xml = cube_xml
    request.robot_namespace = ""
    request.reference_frame = "world"

    print("正在生成红色方块...")
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        print(f"结果: {future.result().status_message}")
    else:
        print("调用失败")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()