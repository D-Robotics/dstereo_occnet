# DStereo Occupancy Network

## 功能介绍

`dstereo_occnet`是地瓜机器人自研的双目占用网络，输入双目图像，输出占用网格，目前算法只支持搭配ZED-2i相机使用。

## 支持平台

| 平台   | 系统                  |
| ------ | --------------------- |
| RDK X5 | Ubuntu 22.04 (Humble) |

## 安装

- 方式1：通过TROS安装

参考：[地瓜机器人-开发者社区](https://developer.d-robotics.cc/rdk_doc/Robot_development/quick_start/install_tros)

- 方式2：通过源码安装

建议使用交叉编译环境进行编译

```bash
git clone https://github.com/D-Robotics/dstereo_occnet.git
bash ./robot_dev_config/build.sh -p X5 -s dstereo_occnet
```

## 运行

### 1. 搭配ZED-2i相机

- 需要配合[`hobot_zed_cam`](https://github.com/D-Robotics/hobot_zed_cam.git)功能包一起使用
- 在RDK X5上执行以下命令:

```bash
# 配置tros.b humble环境
source /opt/tros/humble/setup.bash

# 启动ZED-2i相机和占用网络推理程序
ros2 launch dstereo_occnet zed2i_occ_node.launch.py
```

程序启动后可以通过网页查看ZED-2i发布的双目图像、以及通过rviz2查看占用网格

通过web端查看双目图像，在PC端浏览器输入[http://ip:8000](http://ip:8000)即可查看双目图像，ip为RDK X5板端的ip，例子中为`192.168.128.10`，并且要保证PC和RDK X5能通过网络通讯

![](./docs/ZED-2i-stereo-img.png)

通过rviz2可查看占用网格，需要搭建一个Ubuntu22.04 + ROS2 Humble的上位机，并且上位机和RDK X5板端能够通过ROS2通讯，然后启动rviz2，具体设置如下：

![](./docs/rviz2-occ.png)


- 如需保存结果请加入以下参数:

```bash
# 配置tros.b humble环境
source /opt/tros/humble/setup.bash

# 启动ZED-2i相机和占用网络推理程序
ros2 launch dstereo_occnet zed2i_occ_node.launch.py \
save_occ_flag:=True save_occ_dir:=./occ_result save_freq:=4 save_total:=10
```

### 2. 使用自定义数据离线推理

- 需要准备离线数据上传到RDK X5板端，离线数据的格式如下：
  - 离线目录需要包含左右图片，程序会进行判断，左图需要包含`left`字段，格式为png或者jpg，右图需要包含`right`字段、其它和左图一致
  - 图像分辨率为`640*352`，不支持其它分辨率
  - 左右图需要进行矫正，达到极线对齐的状态
  - 由于目前采用ZED-2i的数据训练模型，所以尽量使离线图像的内参接近ZED-2i，ZED-2i相机参数为: `fx=354.9999, fy=354.9999, cx=322.9469, cy=176.2076, baseline=0.12`

- 在RDK X5上执行以下命令:

```bash
ros2 launch dstereo_occnet offline_infer_web_visual.launch.py \
local_image_dir:=./offline_images save_occ_flag:=True save_occ_dir:=./offline_result
```

## 功能包说明

### 参数

| 名称                | 参数值                                         | 说明                                                            |
| ------------------- | ---------------------------------------------- | --------------------------------------------------------------- |
| stereo_msg_topic    | 默认 /image_combine_raw                        | 订阅的双目图像话题名称                                          |
| camera_info_topic   | 默认 /image_combine_raw/camera_info            | 订阅的相机内参话题名称                                          |
| occ_model_file_path | 默认 X5-OCC-32x64x96x2_constinput_modified.bin | 双目占用网络模型的路径                                          |
| use_local_image     | 默认 False                                     | 是否使用离线推理                                                |
| local_image_dir     | 默认 config                                    | 离线推理存放图片的目录                                          |
| save_occ_flag       | 默认 False                                     | 是否保存推理结果                                                |
| save_occ_dir        | 默认 ./occ_results                             | 保存推理结果的目录                                              |
| save_freq           | 默认 1                                         | 保存频率，例如设置为4代表每隔4帧保存1次，默认每帧推理结果都保存 |
| save_total          | 默认 -1                                        | 保存总数，例如设置为10代表总共保存10帧结果，-1代表一直保存      |
| voxel_size          | 默认 0.02                                      | 每个占用网格的大小，单位m，0.02代表每个占用网格为2*2*2cm        |
| log_level           | 默认 INFO                                      | 日志等级，默认INFO                                              |

### 订阅话题

| 话题名称                       | 消息类型                     | 说明                                                                                                     |
| ------------------------------ | ---------------------------- | -------------------------------------------------------------------------------------------------------- |
| /image_combine_raw             | sensor_msgs::msg::Image      | 订阅双目图像，图像格式为NV12，图像要求上下排列，上面是左图，下面是右图，可以通过stereo_msg_topic参数修改 |
| /image_combine_raw/camera_info | sensor_msgs::msg::CameraInfo | 订阅相机内参，可选，不一定要有该话题，有该话题可以在保存结果时把相机参数一起保存                         |

### 发布话题

| 名称                       | 消息类型                      | 说明                                  |
| -------------------------- | ----------------------------- | ------------------------------------- |
| /dstereo_occnet_node/voxel | sensor_msgs::msg::PointCloud2 | 发布的占用网格数据，可以使用rviz2显示 |