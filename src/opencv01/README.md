# QR Code Decoder Package (opencv01)

这是一个基于 ROS2 的二维码识别功能包，使用 OpenCV 和 pyzbar 库实现实时二维码检测和解码功能，适用于飞行器在飞行过程中连续识别二维码。

## 功能特性

- ✅ 实时从摄像头读取视频流
- ✅ 使用 pyzbar 库进行二维码检测和解码
- ✅ 支持同时检测多个二维码
- ✅ 发布二维码数据、位置坐标和处理后的图像到 ROS2 话题
- ✅ 在图像上绘制二维码边界框和文本信息
- ✅ 支持参数配置（摄像头设备、显示选项等）

## 依赖项

### ROS2 依赖
- `rclpy` - ROS2 Python 客户端库
- `sensor_msgs` - 传感器消息类型
- `std_msgs` - 标准消息类型
- `geometry_msgs` - 几何消息类型
- `cv_bridge` - ROS 图像与 OpenCV 图像转换桥接

### Python 依赖
- `opencv-python` - OpenCV 图像处理库
- `pyzbar` - 二维码/条形码解码库
- `numpy` - 数值计算库

## 安装依赖

```bash
# 安装 ROS2 依赖（如果还没有安装）
sudo apt-get install ros-<your-ros2-distro>-cv-bridge

# 安装 Python 依赖
pip3 install opencv-python pyzbar numpy
```

## 编译

```bash
cd ~/ros2_ws
colcon build --packages-select opencv01
source install/setup.bash
```

## setup.py 配置说明

创建 ROS2 Python 功能包后，通常需要对 `setup.py` 进行以下修改：

### 1. 修改维护者信息（必须）

```python
# 默认生成的（需要修改）
maintainer='ubuntu',
maintainer_email='ubuntu@todo.todo',

# 修改为你的信息
maintainer='your_name',
maintainer_email='your@email.com',
```

### 2. 修改描述信息（必须）

```python
# 默认生成的（TODO）
description='TODO: Package description',

# 修改为实际描述
description='QR code decoder package for ROS2',
```

### 3. 修改许可证（必须）

```python
# 默认生成的（TODO）
license='TODO: License declaration',

# 修改为实际许可证（通常用 Apache-2.0）
license='Apache-2.0',
```

### 4. 添加 Python 第三方库依赖（如果需要）

```python
# 默认只有 setuptools
install_requires=['setuptools'],

# 添加你需要的 Python 库
install_requires=['setuptools', 'opencv-python', 'pyzbar', 'numpy'],
```

**常见依赖示例：**
- `opencv-python` - OpenCV 图像处理
- `numpy` - 数值计算
- `pyzbar` - 二维码解码
- `pyserial` - 串口通信
- `requests` - HTTP 请求

### 5. 添加可执行文件入口点（最重要）

```python
# 默认是空的（必须添加）
entry_points={
    'console_scripts': [
    ],
},

# 添加你的节点
entry_points={
    'console_scripts': [
        'qr_decoder = opencv01.decoder:main',
    ],
},
```

**格式说明：**
```
'可执行文件名 = 包名.模块名:函数名'
```

**示例：**
- `'qr_decoder'` - 运行时的命令名（`ros2 run opencv01 qr_decoder`）
- `opencv01.decoder` - Python 模块路径（`opencv01/decoder.py`）
- `:main` - 要调用的函数（`def main()`）

**多个节点示例：**
```python
entry_points={
    'console_scripts': [
        'node1 = my_package.node1:main',
        'node2 = my_package.node2:main',
    ],
},
```

### 6. 添加 launch 文件（如果需要）

```python
# 需要先导入
from setuptools import find_packages, setup
import os
from glob import glob

# 在 data_files 中添加
data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    # 添加 launch 文件
    (os.path.join('share', package_name, 'launch'),
     glob('launch/*.launch.py')),
],
```

### 依赖管理说明

**重要：** ROS2 Python 包的依赖分为两类：

| 依赖类型 | 声明位置 | 安装方式 | 示例 |
|---------|---------|---------|------|
| **ROS2 系统包** | `package.xml` | `apt install` | `rclpy`, `std_msgs`, `sensor_msgs` |
| **Python 第三方库** | `setup.py` | `pip install` | `opencv-python`, `numpy`, `pyzbar` |

**为什么这样设计？**
- ROS2 系统包（`rclpy`, `std_msgs` 等）由 ROS2 发行版统一管理，通过 apt 安装，在 `package.xml` 中声明
- Python 第三方库（`opencv-python`, `numpy` 等）由 Python 社区维护，通过 pip 安装，在 `setup.py` 中声明

**常见错误：**
- ❌ 错误：在 `setup.py` 中添加 `rclpy`, `std_msgs`（这些应该在 `package.xml` 中）
- ❌ 错误：在 `package.xml` 中添加 `opencv-python`（这些应该在 `setup.py` 中）

### 快速检查清单

创建功能包后，检查并修改：

- [ ] 维护者信息（maintainer, maintainer_email）
- [ ] 描述（description）
- [ ] 许可证（license）
- [ ] Python 依赖（install_requires）
- [ ] 可执行文件入口点（entry_points）**← 最关键**
- [ ] Launch 文件（如果需要，添加 data_files）

## 使用方法

### 基本运行

```bash
ros2 run opencv01 qr_decoder
```

### 带参数运行

```bash
# 指定摄像头设备
ros2 run opencv01 qr_decoder --ros-args -p camera_device:=/dev/video0

# 关闭图像显示窗口
ros2 run opencv01 qr_decoder --ros-args -p show_image:=false

# 组合参数
ros2 run opencv01 qr_decoder --ros-args \
  -p camera_device:=/dev/video0 \
  -p show_image:=true
```

## 参数说明

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `camera_device` | string | `/dev/video0` | 摄像头设备路径 |
| `show_image` | bool | `true` | 是否显示处理后的图像窗口 |

## 话题

### 发布的话题

| 话题名 | 消息类型 | 说明 |
|--------|----------|------|
| `/qr_processed_image` | `sensor_msgs/Image` | 处理后的图像（带二维码标注） |
| `/qr_code_data` | `std_msgs/String` | 检测到的二维码数据内容 |
| `/qr_code_position` | `geometry_msgs/Point` | 二维码中心坐标（x, y, z=0） |

### 订阅的话题

无（直接从摄像头读取）

## 使用示例

### 查看二维码数据

```bash
# 终端1：运行节点
ros2 run opencv01 qr_decoder

# 终端2：查看二维码数据
ros2 topic echo /qr_code_data

# 终端3：查看二维码位置
ros2 topic echo /qr_code_position
```

### 使用 rqt_image_view 查看处理后的图像

```bash
rqt_image_view /qr_processed_image
```

### 查看所有话题

```bash
ros2 topic list
```

## 代码结构

```
opencv01/
├── opencv01/
│   ├── __init__.py
│   └── decoder.py          # 主程序文件
├── package.xml             # 包配置文件
├── setup.py                # Python 包安装配置
├── setup.cfg               # 代码风格配置
├── resource/
│   └── opencv01            # 包标记文件
└── README.md               # 本文件
```

## 工作原理

1. **初始化阶段**：
   - 创建 ROS2 节点
   - 打开摄像头设备
   - 创建发布者（图像、数据、位置）
   - 创建定时器（~30 FPS）

2. **处理循环**：
   - 从摄像头读取一帧图像
   - 转换为 RGB 格式（pyzbar 需要）
   - 使用 pyzbar 检测和解码二维码
   - 对每个检测到的二维码：
     - 提取数据并发布到 `/qr_code_data`
     - 计算中心坐标并发布到 `/qr_code_position`
     - 在图像上绘制边界框和文本
   - 发布处理后的图像到 `/qr_processed_image`
   - 显示图像窗口（可选）

3. **资源清理**：
   - 释放摄像头资源
   - 关闭图像窗口

## 注意事项

- 确保摄像头设备有正确的权限：`sudo chmod 666 /dev/video0`
- 如果摄像头无法打开，程序会尝试使用索引 0（`/dev/video0`）
- 按 ESC 键可以退出程序
- 二维码检测需要良好的光照条件
- 支持同时检测多个二维码

## 故障排除

### 摄像头无法打开

```bash
# 检查摄像头设备
ls -l /dev/video*

# 检查权限
sudo chmod 666 /dev/video0

# 测试摄像头
v4l2-ctl --list-devices
```

### 找不到 pyzbar 模块

```bash
pip3 install pyzbar
# 如果还有问题，可能需要安装系统依赖
sudo apt-get install libzbar0
```

### 图像显示问题

如果不需要显示图像，可以设置 `show_image:=false`，这样可以减少资源消耗。

## 许可证

Apache-2.0

## 维护者

orangepi (2449708401@qq.com)
