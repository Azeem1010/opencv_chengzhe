# OpenCV_ROS

ROS 2 + OpenCV 攝影機整合專案，提供三種執行路徑：

1. 使用本地攝影機節點的 Linux/WSL 攝影機管線。
2. 透過 TCP 橋接將 Windows 攝影機畫面傳送到 WSL 中 ROS topic 的管線。
3. 僅傳座標的管線：Windows 發送端 + WSL TCP 座標橋接（不發布 ROS topic）。

本儲存庫已為 GitHub 公開做好準備，包含 Docker 打包、執行腳本與操作文件。

## 目前專案狀態

- ROS 2 執行腳本可透過 `run.bash` 使用（`doctor`、`cv`、`cv-view`、`mock-cv`、`win-bridge`、`win-cv`）。
- 僅傳座標的執行模式可透過 `run.bash win-coord` 使用（不輸出 ROS topic）。
- Windows 發送端到 WSL 橋接的管線已實作完成。
- 點擊發布座標到 `/camera/object_point` 已實作完成。
- Windows 預覽視窗中的選用黑色物件追蹤已實作（切換鍵 `B`，可用啟動參數開啟）。
- Docker 映像檔建置與 `doctor` 執行檢查已驗證通過。

## 主要功能

- 在 `/camera/image_raw` 上發布 ROS 影像。
- 在 `/camera/object_point` 上發布座標點。
- 透過 ROS 服務 `/camera/get_object_point` 查詢最新偵測到的座標點（`opencv_ros2_bridge_interfaces/srv/GetObjectPoint`）。
- Windows 預覽發送端，支援：
  - 左鍵點擊發布座標點
  - 選用的黑色物件偵測
  - 預覽疊加層與目標邊界框
- WSL 橋接發布器，含座標點換算（`scale_px_per_meter`）。
- WSL 僅傳座標橋接，輸出元組格式 `(X y z) (0.0 0.0 0.0)`。
- 模擬攝影機模式，可在沒有實體攝影機的情況下進行開發。

## 儲存庫結構

- `run.bash`：主啟動器。
- `windows_camera_ros_sender.py`：Windows 擷取與 TCP 發送端。
- `windows_stream_bridge_publisher.py`：TCP 橋接到 ROS Image 與 PointStamped。
- `windows_coordinate_bridge.py`：僅輸出座標的 TCP 橋接（不發布 ROS topic）。
- `camera_point_cv_subscriber.py`：OpenCV 檢視器與座標點疊加層。
- `usb_camera_publisher.py`：本地 USB 攝影機 ROS 發布器。
- `mock_camera_publisher.py`：合成測試用攝影機來源。
- `Dockerfile`、`docker-compose.yml`、`docker/entrypoint.sh`：容器打包。
- `DOCKER.md`：Docker 使用與疑難排解。

## 快速開始（原生執行）

### 1. 環境檢查

```bash
source /opt/ros/jazzy/setup.bash
bash run.bash doctor
```

### 2. 僅傳座標模式（不發布 ROS topic）

WSL 終端機：

```bash
bash run.bash win-coord
```

可行時會自動啟動 Windows 發送端。
座標輸出會在 WSL 中以多行區塊印出：

```text
source: click
frame: base_link
(x y z): (0.0 0.0 0.0)
```

預設座標軸對應遵循 REP103 的 `base_link`（x 向前、y 向左、z 向上），並假設為俯視攝影機。

### 3. Windows ROS 橋接模式

WSL 終端機 A：

```bash
bash run.bash win-bridge
```

Windows 終端機：

```bat
cd /d C:\Users\<user>\Downloads\OpenCV_ROS\OpenCV_ROS
python windows_camera_ros_sender.py --host 127.0.0.1 --port 5001 --preview
```

WSL 終端機 B：

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic echo /camera/object_point
```

若要按需取得最新偵測到的座標點，而非訂閱 topic，先建置一次介面套件：

```bash
colcon build --packages-select opencv_ros2_bridge_interfaces
source install/setup.bash
```

接著呼叫服務：

```bash
ros2 service call /camera/get_object_point opencv_ros2_bridge_interfaces/srv/GetObjectPoint
```

回應內容包含 `success`、`message`、`source`（例如 `click` 或 `black_detect`），以及一個 `geometry_msgs/PointStamped`。

## 快速開始（Docker 執行）

### 1. 建置映像檔

```bash
docker build -t opencv_ros:jazzy .
```

### 2. 驗證執行環境

```bash
docker run --rm --network host opencv_ros:jazzy bash run.bash doctor
```

### 3. 啟動橋接容器

```bash
docker run --rm -it --network host opencv_ros:jazzy bash run.bash win-bridge
```

接著在 Windows 主機上執行 `windows_camera_ros_sender.py`。

## 黑色物件偵測

啟動時開啟：

```bash
WIN_DETECT_BLACK=1 bash run.bash win-cv
```

或在 Windows 上直接傳入發送端參數：

```bat
python windows_camera_ros_sender.py --host 127.0.0.1 --port 5001 --preview --detect-black --black-detect-hz 8 --black-v-max 55 --min-black-area 600
```

預覽視窗中的執行時控制：

- `q` 或 `Esc`：離開
- `B`：切換黑色物件偵測
- 滑鼠左鍵點擊：發布點擊的座標點

## 已知限制

- 在 Linux Docker 容器內執行時，無法可靠地自動啟動 `cmd.exe` 的 Windows 發送端。
- 若尚未發布過任何座標點，`ros2 topic echo --once` 可能會逾時。
- 若連接埠 `5001` 已被佔用，請在橋接端與發送端都設定 `WIN_BRIDGE_PORT=<port>`。

## 相關文件

- `DOCKER.md`

## 授權

目前尚未包含儲存庫授權檔案。
公開發布前請選擇並加入授權條款（例如 MIT、Apache-2.0 或 GPL-3.0）。
