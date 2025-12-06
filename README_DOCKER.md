、



## 创建联网隧道

```bash
ssh -N -L 0.0.0.0:18889:127.0.0.1:8889 localhost
```

## 构建 Docker 镜像

### 方法: 使用本地 项目 目录

```bash
cd /data2/Project/Arm/ycliu/VLM_Grasp_Interactive
docker buildx build -f Dockerfile.test -t piper_vlm_local:latest .
```


## 运行容器

### 使用 docker-compose（推荐）

# 启动后进入
```bash
docker compose up -d
docker compose exec piper_vlm bash
docker logs -f piper_vlm_dev
```



## 远程可视化

本地启动XLaunch

由于docker的rootless模式network host并不能真正起作用。即无法直接向宿主机127.0.0.1发送数据。因此需要做一层转发
```bash
ssh -X -N -L 0.0.0.0:6809:127.0.0.1:6808 localhost
```

## 测试相机

在容器内运行以下命令测试相机：

```bash
# 启动 realsense 节点
ros2 launch realsense2_camera rs_launch.py

# 或者使用自定义配置
ros2 launch realsense2_camera rs_launch.py \
    camera_name:=camera \
    enable_depth:=true \
    enable_color:=true \
    enable_infra1:=false \
    enable_infra2:=false
```

## 查看话题

```bash
# 列出所有话题
ros2 topic list

# 查看深度图像
ros2 topic echo /camera/depth/image_rect_raw

# 查看彩色图像
ros2 topic echo /camera/color/image_raw
```


## Rviz可视化

由于docker的rootless模式network host并不能真正起作用。即无法直接向宿主机127.0.0.1发送数据。因此需要做一层转发
ssh -X -N -L 0.0.0.0:6809:127.0.0.1:6808 localhost

####  查看日志
docker compose logs -f rviz
docker compose exec rviz bash


####  显示图像

设置参考坐标系 (Fixed Frame)
左上角 Global Options -> Fixed Frame
map 改为 camera_link 可手动输入

添加 Image 插件
左下角的 Add 按钮
向下滚动找到 Image，点击选中它。

选择要订阅的话题 (Topic)
左侧 Displays 面板中，你会看到新增加了一个 Image 项。
找到 Topic 这一行
选择或输入：/camera/color/image_raw

####  (进阶)显示点云

realsense容器要运行
ros2 launch realsense2_camera rs_launch.py \
    enable_pointcloud:=true \
    align_depth.enable:=true \
    enable_sync:=true



## 注意事项

1. **USB 设备访问**: 容器需要 `--privileged` 标志或适当的设备映射来访问 USB 设备。

2. **librealsense 版本**: 使用 2.54.1 版本是因为从 2.55.1 开始，Intel 停止了对 L515 的支持。

3. **RSUSB 后端**: Dockerfile 使用 `FORCE_RSUSB_BACKEND=true` 构建 librealsense，这样可以避免对内核模块的依赖，更适合 Docker 环境。

4. **ROS2 版本**: Ubuntu 20.04 对应 ROS2 Foxy，这是官方推荐的组合。

5. **权限问题**: 如果遇到权限问题，可以在容器内运行：
   ```bash
   sudo chmod 666 /dev/video*
   ```

## 故障排除

### 相机无法检测

1. 检查 USB 连接：
   ```bash
   lsusb | grep Intel
   ```

2. 检查设备节点：
   ```bash
   ls -l /dev/video*
   ```

3. 检查 udev 规则：
   ```bash
   udevadm control --reload-rules
   udevadm trigger
   ```

### 编译错误

如果遇到编译错误，可以尝试：

1. 清理构建缓存：
   ```bash
   docker build --no-cache -t realsense-l515-ros2:latest .
   ```

2. 检查依赖：
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```

## 参考链接

- [realsense-ros GitHub](https://github.com/IntelRealSense/realsense-ros)
- [librealsense GitHub](https://github.com/IntelRealSense/librealsense)
- [ROS2 Foxy 文档](https://docs.ros.org/en/foxy/)

