# OpenCV_ROS

ROS 2 + OpenCV camera integration project with runtime paths:

1. Linux/WSL camera pipeline using local camera nodes.
2. Windows camera capture pipeline using TCP bridge to ROS topics in WSL.
3. Coordinate-only pipeline using Windows sender + WSL TCP coordinate bridge (no ROS publish).

This repository is prepared for GitHub publication with Docker packaging, runtime scripts, and operational documentation.

## Current Project Status

- ROS 2 runtime scripts are available in `run.bash` (`doctor`, `cv`, `cv-view`, `mock-cv`, `win-bridge`, `win-cv`).
- Coordinate-only runtime is available in `run.bash win-coord` (no ROS topic output).
- Windows sender to WSL bridge pipeline is implemented.
- Click-to-point publishing to `/camera/object_point` is implemented.
- Optional black-object tracking in Windows preview is implemented (toggle key `B`, optional startup flag).
- Docker image build and `doctor` runtime check are verified.

## Key Features

- ROS image publishing on `/camera/image_raw`.
- Point publishing on `/camera/object_point`.
- Latest detected point query via ROS service `/camera/get_object_point` (`opencv_ros2_bridge_interfaces/srv/GetObjectPoint`).
- Windows preview sender with:
  - left-click point publish
  - optional black-object detection
  - preview overlay and target bounding box
- WSL bridge publisher with point conversion (`scale_px_per_meter`).
- WSL coordinate-only bridge with tuple output format `(X y z) (0.0 0.0 0.0)`.
- Mock camera mode for development without physical camera.

## Repository Layout

- `run.bash`: main launcher.
- `windows_camera_ros_sender.py`: Windows capture and TCP sender.
- `windows_stream_bridge_publisher.py`: TCP bridge to ROS Image and PointStamped.
- `windows_coordinate_bridge.py`: TCP bridge that outputs coordinates only (no ROS publish).
- `camera_point_cv_subscriber.py`: OpenCV viewer and point overlay.
- `usb_camera_publisher.py`: local USB camera ROS publisher.
- `mock_camera_publisher.py`: synthetic test camera source.
- `Dockerfile`, `docker-compose.yml`, `docker/entrypoint.sh`: container packaging.
- `DOCKER.md`: Docker usage and troubleshooting.

## Quick Start (Native Runtime)

### 1. Environment check

```bash
source /opt/ros/jazzy/setup.bash
bash run.bash doctor
```

### 2. Coordinate-only mode (no ROS publish)

WSL terminal:

```bash
bash run.bash win-coord
```

Windows sender is auto-launched when possible.
Coordinate output is printed in WSL as multiline blocks:

```text
source: click
frame: base_link
(x y z): (0.0 0.0 0.0)
```

Default axis mapping follows REP103 `base_link` (x forward, y left, z up) with top-view camera assumption.

### 3. Windows ROS bridge mode

WSL terminal A:

```bash
bash run.bash win-bridge
```

Windows terminal:

```bat
cd /d C:\Users\<user>\Downloads\OpenCV_ROS\OpenCV_ROS
python windows_camera_ros_sender.py --host 127.0.0.1 --port 5001 --preview
```

WSL terminal B:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic echo /camera/object_point
```

To pull the latest detected point on demand instead of subscribing to the topic, build the interface package once:

```bash
colcon build --packages-select opencv_ros2_bridge_interfaces
source install/setup.bash
```

Then call the service:

```bash
ros2 service call /camera/get_object_point opencv_ros2_bridge_interfaces/srv/GetObjectPoint
```

The response carries `success`, `message`, `source` (e.g. `click` or `black_detect`), and a `geometry_msgs/PointStamped`.

## Quick Start (Docker Runtime)

### 1. Build image

```bash
docker build -t opencv_ros:jazzy .
```

### 2. Verify runtime

```bash
docker run --rm --network host opencv_ros:jazzy bash run.bash doctor
```

### 3. Start bridge container

```bash
docker run --rm -it --network host opencv_ros:jazzy bash run.bash win-bridge
```

Then run `windows_camera_ros_sender.py` on Windows host.

## Black Object Detection

Enable at startup:

```bash
WIN_DETECT_BLACK=1 bash run.bash win-cv
```

Or pass sender flags directly on Windows:

```bat
python windows_camera_ros_sender.py --host 127.0.0.1 --port 5001 --preview --detect-black --black-detect-hz 8 --black-v-max 55 --min-black-area 600
```

Runtime controls in preview window:

- `q` or `Esc`: quit
- `B`: toggle black-object detection
- left mouse click: publish clicked point

## Known Constraints

- Running inside Linux Docker container cannot reliably auto-launch `cmd.exe` Windows sender.
- `ros2 topic echo --once` may time out if no point was published yet.
- If port `5001` is occupied, use `WIN_BRIDGE_PORT=<port>` on both bridge and sender.

## Related Docs

- `DOCKER.md`

## License

No repository license file is included yet.
Choose and add a license before public release (for example, MIT, Apache-2.0, or GPL-3.0).
