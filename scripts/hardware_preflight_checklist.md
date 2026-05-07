# Hardware Preflight Checklist — Mobile Robot Real-World Run

**Mục đích:** kiểm tra nhanh nhưng đủ chặt các phần cứng và node ROS 2 quan trọng trước khi chạy robot tự hành trong môi trường thật.

**Môi trường mục tiêu:** Jetson AGX Xavier + X221-AI Carrier Board, Ubuntu 20.04, ROS 2 Foxy, FastRTPS, RPLIDAR S2E qua LAN, BNO055 qua I2C J23, STM32F103RCT6 qua USB-TTL UART.

**Nguồn tham chiếu chính:** `preflight_check.sh`

---

## 1. Thông số kiểm tra mặc định

| Nhóm | Thông số | Giá trị kỳ vọng |
|---|---:|---:|
| LiDAR | IP RPLIDAR S2E | `192.168.11.2` |
| LiDAR | Interface LAN Jetson | `enp2s0` |
| IMU | I2C bus | `/dev/i2c-8` |
| IMU | BNO055 address | `0x28` |
| STM32 wheel control | UART device | `/dev/ttyUSB1` |
| ROS workspace | Workspace | `~/mbrobot_ws` |
| DDS | `ROS_DOMAIN_ID` | `42` |
| DDS | `RMW_IMPLEMENTATION` | `rmw_fastrtps_cpp` |
| `/scan` | Tần số tối thiểu | `7.0 Hz` |
| `/scan_filtered` | Tần số tối thiểu | `7.0 Hz` |
| `/imu/data` | Tần số tối thiểu | `45.0 Hz` |
| `/odom` | Tần số tối thiểu | `80.0 Hz` |
| `/odometry/filtered` | Tần số tối thiểu | `45.0 Hz` |
| Jetson temperature | WARN | `>= 75°C` |
| Jetson temperature | FAIL | `>= 85°C` |
| RAM available | WARN | `<= 1500 MB` |
| RAM available | FAIL | `<= 800 MB` |
| Disk free `/` | WARN | `<= 5000 MB` |

---

## 2. Cách chạy kiểm tra

### 2.1 Kiểm tra chuẩn

```bash
bash preflight_check.sh
```

Dùng khi các node ROS chính đã được launch và robot chuẩn bị chạy tự hành.

### 2.2 Kiểm tra không cần ROS

```bash
bash preflight_check.sh --no-ros
```

Dùng để kiểm tra nhanh OS, cổng thiết bị, IMU, LiDAR trước khi chạy launch file.

### 2.3 Kiểm tra kèm motor test

```bash
bash preflight_check.sh --motor-test
```

> ⚠️ **Cảnh báo:** robot có thể di chuyển. Chỉ chạy khi robot đặt trên sàn phẳng, phía trước và phía sau trống ít nhất 50 cm.

### 2.4 Lưu log kiểm tra

```bash
mkdir -p ~/robot_logs
bash preflight_check.sh 2>&1 | tee ~/robot_logs/preflight_$(date +%Y%m%d_%H%M%S).log
```

---

## 3. Quy tắc GO / NO-GO

| Kết quả | Quyết định | Ý nghĩa |
|---|---|---|
| `FAIL = 0` và `WARN <= 2` | **GO** | Có thể chạy tự hành. |
| `FAIL = 0` và `WARN > 2` | **GO WITH CAUTION** | Có thể chạy nhưng nên xử lý cảnh báo trước demo thực tế. |
| `FAIL > 0` | **NO-GO** | Không chạy tự hành. Phải debug các lỗi nghiêm trọng trước. |

**Nguyên tắc an toàn:** nếu lỗi liên quan đến `/cmd_vel`, `/odom`, e-stop, TF `odom -> base_footprint`, LiDAR `/scan`, hoặc IMU `/imu/data`, phải xem là **NO-GO** dù script chỉ báo WARN.

---

# 4. Checklist chi tiết

## LEVEL 0 — Kiểm tra an toàn cơ khí trước khi cấp lệnh

| Check | Cách kiểm tra | PASS | Kết quả |
|---|---|---|---|
| Robot đặt ổn định trên sàn | Quan sát trực tiếp | Không nghiêng, không kẹt bánh tự do phía trước | ☐ PASS ☐ WARN ☐ FAIL |
| Vùng xung quanh robot | Kiểm tra bán kính tối thiểu 0.5 m | Không có người, dây, vật cản sát bánh | ☐ PASS ☐ WARN ☐ FAIL |
| Nút/cơ chế dừng khẩn cấp | Kiểm tra thao tác dừng thủ công | Có thể dừng robot ngay khi bất thường | ☐ PASS ☐ WARN ☐ FAIL |
| Bánh trái/phải hoverboard | Quan sát cơ khí | Không lỏng, không cạ khung, không kẹt | ☐ PASS ☐ WARN ☐ FAIL |
| Bánh tự do phía trước | Xoay tay nhẹ | Xoay tự do, không rung mạnh, không lệch trục | ☐ PASS ☐ WARN ☐ FAIL |
| Pin/nguồn cấp | Quan sát và đo nếu cần | Điện áp ổn định, đầu nối chắc, không nóng bất thường | ☐ PASS ☐ WARN ☐ FAIL |
| Dây LAN LiDAR | Kiểm tra đầu nối | Cắm chắc, không căng dây khi robot quay | ☐ PASS ☐ WARN ☐ FAIL |
| Dây USB-TTL STM32 | Kiểm tra đầu nối | Không lỏng, không bị kéo khi robot di chuyển | ☐ PASS ☐ WARN ☐ FAIL |
| Dây IMU BNO055 tại J23 | Kiểm tra đầu nối | SDA/SCL/3.3V/GND đúng, không đảo dây | ☐ PASS ☐ WARN ☐ FAIL |

---

## LEVEL 1 — Tài nguyên hệ thống Jetson

### 4.1 Nhiệt độ CPU

**Lệnh kiểm tra thủ công:**

```bash
cat /sys/devices/virtual/thermal/thermal_zone0/temp
```

**Diễn giải:** giá trị trả về thường là milli-degree Celsius. Ví dụ `65000` tương ứng `65°C`.

| Điều kiện | Đánh giá | Hành động |
|---|---|---|
| `< 75°C` | PASS | Có thể tiếp tục. |
| `75°C ~ < 85°C` | WARN | Theo dõi sát; kiểm tra quạt/tản nhiệt. |
| `>= 85°C` | FAIL | Dừng kiểm tra, đợi Jetson nguội trước khi chạy. |

Kết quả: `________ °C` → ☐ PASS ☐ WARN ☐ FAIL

### 4.2 RAM khả dụng

**Lệnh:**

```bash
free -m
```

| Điều kiện | Đánh giá | Hành động |
|---|---|---|
| `> 3000 MB` | PASS | Tốt. |
| `1500 ~ 3000 MB` | WARN | Tắt browser, RViz thừa, terminal/log không cần thiết. |
| `<= 800 MB` | FAIL | Không đủ tài nguyên để chạy ổn định. |

Kết quả: `________ MB` → ☐ PASS ☐ WARN ☐ FAIL

### 4.3 Dung lượng disk

**Lệnh:**

```bash
df -m /
```

| Điều kiện | Đánh giá | Hành động |
|---|---|---|
| `> 5000 MB` | PASS | Đủ cho log và runtime. |
| `<= 5000 MB` | WARN | Dọn log cũ, bag file, build cache nếu cần. |

Kết quả: `________ MB` → ☐ PASS ☐ WARN ☐ FAIL

---

## LEVEL 1 — Device ports và cảm biến mức OS

### 4.4 UART STM32 qua USB-TTL

**Kỳ vọng:** `/dev/ttyUSB1`

**Lệnh:**

```bash
ls -l /dev/ttyUSB*
test -r /dev/ttyUSB1 && test -w /dev/ttyUSB1 && echo OK
```

| Điều kiện | Đánh giá | Hành động |
|---|---|---|
| `/dev/ttyUSB1` tồn tại và có quyền đọc/ghi | PASS | Có thể chạy wheel odometry node. |
| Có `/dev/ttyUSB*` khác nhưng không phải `/dev/ttyUSB1` | WARN | Kiểm tra lại mapping port hoặc sửa `wheel_odom_params.yaml`. |
| Không có `/dev/ttyUSB*` | FAIL | Kiểm tra USB-TTL, J33, dây GND/TX/RX, nguồn STM32. |
| Có port nhưng thiếu quyền | WARN/FAIL | Thêm user vào group `dialout`, logout/login lại. |

**Lệnh sửa quyền thường dùng:**

```bash
sudo usermod -aG dialout $USER
# Sau đó logout/login hoặc reboot Jetson
```

Kết quả: `________________` → ☐ PASS ☐ WARN ☐ FAIL

### 4.5 I2C bus cho BNO055

**Kỳ vọng:** `/dev/i2c-8`

**Lệnh:**

```bash
ls -l /dev/i2c-8
```

| Điều kiện | Đánh giá | Hành động |
|---|---|---|
| `/dev/i2c-8` tồn tại | PASS | Có thể kiểm tra BNO055. |
| Không tồn tại | FAIL | Kiểm tra cấu hình Jetson/X221-AI, J23 wiring, device tree nếu có chỉnh sửa. |

Kết quả: `________________` → ☐ PASS ☐ WARN ☐ FAIL

### 4.6 BNO055 detect trên I2C

**Kỳ vọng:** address `0x28`

**Lệnh:**

```bash
sudo apt install -y i2c-tools
sudo i2cdetect -y 8
```

| Điều kiện | Đánh giá | Hành động |
|---|---|---|
| Thấy `28` trên bus 8 | PASS | IMU detect tốt. |
| Không thấy `28` | FAIL | Kiểm tra SDA, SCL, 3.3V, GND, ADR pin, bus number. |
| Không có `i2cdetect` | WARN | Cài `i2c-tools`, sau đó kiểm tra lại. |

Kết quả: `0x____` → ☐ PASS ☐ WARN ☐ FAIL

### 4.7 LiDAR RPLIDAR S2E qua LAN

**Kỳ vọng:** ping được `192.168.11.2`, packet loss 0%.

**Lệnh:**

```bash
ping -c 3 -W 2 192.168.11.2
```

Nếu không ping được, thử cấu hình lại interface:

```bash
sudo ip addr add 192.168.11.1/24 dev enp2s0 2>/dev/null || true
sudo ip link set enp2s0 up
ping -c 3 -W 2 192.168.11.2
```

| Điều kiện | Đánh giá | Hành động |
|---|---|---|
| Ping được, 0% packet loss | PASS | LiDAR network OK. |
| Ping được nhưng có packet loss | WARN | Kiểm tra dây LAN, đầu RJ45, nguồn LiDAR. |
| Không ping được sau reconnect | FAIL | Kiểm tra IP LiDAR, IP Jetson, cable, port LAN, nguồn LiDAR. |

Kết quả: `RTT avg = ______ ms, packet loss = ______ %` → ☐ PASS ☐ WARN ☐ FAIL

---

## LEVEL 2 — ROS topics sau khi launch robot thật

> Yêu cầu: `nav_launch.py` hoặc launch tương đương đã chạy ổn định. Đợi ít nhất 10–20 giây sau khi launch trước khi đo tần số topic.

### 4.8 Kiểm tra danh sách topic

```bash
ros2 topic list
```

Các topic tối thiểu phải có:

```text
/scan
/scan_filtered
/imu/data
/odom
/odometry/filtered
/cmd_vel
/robot/state
/robot/emergency_stop
```

Kết quả: ☐ PASS ☐ WARN ☐ FAIL

### 4.9 Kiểm tra tần số topic

| Topic | Lệnh | PASS nếu |
|---|---|---:|
| `/scan` | `timeout 7 ros2 topic hz /scan --window 5` | `>= 7.0 Hz` |
| `/scan_filtered` | `timeout 7 ros2 topic hz /scan_filtered --window 5` | `>= 7.0 Hz` |
| `/imu/data` | `timeout 7 ros2 topic hz /imu/data --window 5` | `>= 45.0 Hz` |
| `/odom` | `timeout 7 ros2 topic hz /odom --window 5` | `>= 80.0 Hz` |
| `/odometry/filtered` | `timeout 7 ros2 topic hz /odometry/filtered --window 5` | `>= 45.0 Hz` |

| Topic | Tần số đo được | Kết quả |
|---|---:|---|
| `/scan` | `______ Hz` | ☐ PASS ☐ WARN ☐ FAIL |
| `/scan_filtered` | `______ Hz` | ☐ PASS ☐ WARN ☐ FAIL |
| `/imu/data` | `______ Hz` | ☐ PASS ☐ WARN ☐ FAIL |
| `/odom` | `______ Hz` | ☐ PASS ☐ WARN ☐ FAIL |
| `/odometry/filtered` | `______ Hz` | ☐ PASS ☐ WARN ☐ FAIL |

**NO-GO nếu:** topic không tồn tại, không có data, hoặc tần số thấp hơn ngưỡng trong nhiều lần đo liên tiếp.

---

## LEVEL 2 — TF tree

### 4.10 TF `odom -> base_footprint`

```bash
timeout 4 ros2 run tf2_ros tf2_echo odom base_footprint
```

PASS nếu có dòng `Translation` và giá trị cập nhật liên tục.

Kết quả: ☐ PASS ☐ WARN ☐ FAIL

### 4.11 TF `base_footprint -> laser_frame`

```bash
timeout 4 ros2 run tf2_ros tf2_echo base_footprint laser_frame
```

PASS nếu có static/dynamic transform hợp lệ từ thân robot đến LiDAR.

Kết quả: ☐ PASS ☐ WARN ☐ FAIL

### 4.12 TF `map -> odom`

```bash
timeout 4 ros2 run tf2_ros tf2_echo map odom
```

PASS nếu AMCL hoặc SLAM publish được transform `map -> odom`.

Kết quả: ☐ PASS ☐ WARN ☐ FAIL

**NO-GO nếu thiếu:**

- `odom -> base_footprint`: EKF/wheel odometry chưa hoạt động đúng.
- `base_footprint -> laser_frame`: URDF/static TF sai, LiDAR không khớp footprint.
- `map -> odom`: AMCL/SLAM chưa định vị được robot trong map.

---

## LEVEL 2 — Nav2 lifecycle nodes

### 4.13 Kiểm tra node Nav2

```bash
ros2 node list | grep -E "(amcl|controller_server|planner_server|bt_navigator|map_server)"
```

Kỳ vọng có ít nhất 4/5 node sau:

```text
/amcl
/controller_server
/planner_server
/bt_navigator
/map_server
```

| Số node tìm thấy | Đánh giá | Hành động |
|---:|---|---|
| `>= 4` | PASS | Nav2 bringup cơ bản đã chạy. |
| `< 4` | WARN/FAIL | Kiểm tra `nav_launch.py`, map file, params file, lifecycle manager. |

Kết quả: `____ / 5` → ☐ PASS ☐ WARN ☐ FAIL

---

## LEVEL 2 — Emergency stop

### 4.14 Kích hoạt e-stop

```bash
ros2 topic pub --once /robot/emergency_stop std_msgs/msg/Bool "{data: true}"
sleep 2
ros2 topic echo /robot/state --once
```

PASS theo script nếu state trả về:

```text
EMERGENCY_STOP
```

Kết quả state: `________________` → ☐ PASS ☐ WARN ☐ FAIL

### 4.15 Reset e-stop

```bash
ros2 topic pub --once /robot/emergency_stop std_msgs/msg/Bool "{data: false}"
sleep 2
ros2 topic echo /robot/state --once
```

PASS theo script nếu state trả về:

```text
IDLE
```

Kết quả state: `________________` → ☐ PASS ☐ WARN ☐ FAIL

> Ghi chú triển khai: nếu phiên bản `navigator.py` hiện tại dùng state `STOPPED` thay cho `EMERGENCY_STOP`, cần thống nhất lại logic kiểm tra. Về an toàn, điều kiện quan trọng nhất là robot phải ngừng xuất vận tốc nguy hiểm xuống `/cmd_vel` và không tiếp tục goal khi e-stop đang active.

---

## LEVEL 3 — Motor test có kiểm soát

> Chỉ thực hiện khi robot đã PASS Level 1 và Level 2. Test này làm robot tiến khoảng 10 cm rồi lùi khoảng 10 cm.

### 4.16 Điều kiện an toàn trước motor test

| Check | PASS nếu | Kết quả |
|---|---|---|
| Vùng trước robot | Trống ít nhất 50 cm | ☐ PASS ☐ FAIL |
| Vùng sau robot | Trống ít nhất 50 cm | ☐ PASS ☐ FAIL |
| Người vận hành | Đứng cạnh robot, sẵn sàng ngắt nguồn/e-stop | ☐ PASS ☐ FAIL |
| Sàn | Phẳng, không trơn, không có dây vướng | ☐ PASS ☐ FAIL |
| Pin/nguồn | Đủ tải, không tụt áp khi motor chạy | ☐ PASS ☐ FAIL |

### 4.17 Lệnh motor test tự động

```bash
bash preflight_check.sh --motor-test
```

### 4.18 Kiểm tra thủ công sau motor test

| Check | PASS nếu | Kết quả |
|---|---|---|
| Robot tiến | Tiến khoảng 10 cm khi `linear.x = 0.1 m/s` | ☐ PASS ☐ WARN ☐ FAIL |
| Robot lùi | Lùi gần về vị trí ban đầu khi `linear.x = -0.1 m/s` | ☐ PASS ☐ WARN ☐ FAIL |
| Bánh trái/phải | Hai bánh quay đều, không lệch mạnh | ☐ PASS ☐ WARN ☐ FAIL |
| Chiều odometry | `/odom.pose.pose.position.x` tăng khi tiến, giảm khi lùi | ☐ PASS ☐ WARN ☐ FAIL |
| Âm thanh motor | Không có tiếng rít, kẹt, giật bất thường | ☐ PASS ☐ WARN ☐ FAIL |
| Dừng sau test | Robot dừng hoàn toàn sau lệnh Twist rỗng | ☐ PASS ☐ WARN ☐ FAIL |

**NO-GO nếu:** robot chạy ngược chiều, một bánh không quay, `/odom` sai chiều, robot không dừng, hoặc có dao động mạnh.

---

# 5. Bảng debug nhanh khi FAIL

| Mục FAIL | Nguyên nhân thường gặp | Cách xử lý ưu tiên |
|---|---|---|
| Không thấy `/dev/ttyUSB1` | USB-TTL đổi port, lỏng dây, chưa cấp nguồn STM32 | `ls /dev/ttyUSB*`, cắm lại USB, kiểm tra TX/RX/GND, sửa `serial_port` nếu cần |
| UART thiếu quyền | User chưa thuộc group `dialout` | `sudo usermod -aG dialout $USER`, reboot/logout-login |
| Không thấy `/dev/i2c-8` | Sai bus, J23/I2C chưa enable | Kiểm tra mapping I2C trên Jetson/X221-AI, kiểm tra device tree nếu đã chỉnh |
| Không detect BNO055 `0x28` | Sai dây SDA/SCL, thiếu nguồn, ADR sai | Đo 3.3V/GND, kiểm tra SDA/SCL, thử địa chỉ `0x29` nếu ADR kéo VCC |
| LiDAR không ping | Sai IP, interface chưa up, dây LAN lỗi | Set IP `192.168.11.1/24` cho `enp2s0`, kiểm tra cable/RJ45/nguồn LiDAR |
| `/scan` không có data | LiDAR driver chưa chạy, sai UDP IP/port | Kiểm tra `sllidar_node`, IP `192.168.11.2`, UDP port `8089` |
| `/scan_filtered` không có data | Laser filter chưa start hoặc thiếu TF `base_footprint` | Kiểm tra `scan_to_scan_filter_chain`, TF `base_footprint -> laser_frame` |
| `/imu/data` thấp hoặc mất | BNO055 driver lỗi, I2C nhiễu | Kiểm tra bus 8, địa chỉ 0x28, dây ngắn/chắc, nguồn sạch |
| `/odom` thấp hoặc mất | STM32 không gửi packet, parse UART lỗi | Kiểm tra firmware, baud 115200, frame format, USB-TTL, GND chung |
| `/odometry/filtered` mất | EKF không nhận `/odom` hoặc `/imu/data` | Kiểm tra `ekf_filter_node`, topic input, frame ID, timestamp |
| Thiếu `odom -> base_footprint` | EKF không publish TF | Kiểm tra `publish_tf: true` trong EKF, `publish_tf: false` ở wheel odom để tránh trùng TF |
| Thiếu `map -> odom` | AMCL/SLAM chưa chạy hoặc chưa set initial pose | Kiểm tra map, `/scan_filtered`, RViz initial pose, lifecycle AMCL |
| E-stop không đổi state | Topic/state machine không khớp | Kiểm tra `/robot/emergency_stop`, `/robot/state`, logic navigator/safety node |
| Motor chạy ngược | Đảo chiều wheel hoặc quy ước trái/phải sai | Kiểm tra mapping omega_L/omega_R, sign command, dây motor/hall |
| Robot không dừng | `/cmd_vel` không về zero hoặc STM32 giữ lệnh cũ | Gửi Twist zero, thêm watchdog timeout ở STM32/Jetson bridge |

---

# 6. Mẫu ghi kết quả trước khi chạy tự hành

```text
Date/time              : ______________________________
Operator               : ______________________________
Location / floor        : ______________________________
Robot battery/voltage   : ______________________________
ROS launch file         : ______________________________
Map file                : ______________________________
Checkpoint file         : ______________________________

LEVEL 0 Safety          : PASS / WARN / FAIL
LEVEL 1 System          : PASS / WARN / FAIL
LEVEL 1 Device ports    : PASS / WARN / FAIL
LEVEL 2 ROS topics      : PASS / WARN / FAIL
LEVEL 2 TF tree         : PASS / WARN / FAIL
LEVEL 2 Nav2 nodes      : PASS / WARN / FAIL
LEVEL 2 E-stop          : PASS / WARN / FAIL
LEVEL 3 Motor test      : PASS / WARN / FAIL / SKIPPED

Total PASS              : ______
Total WARN              : ______
Total FAIL              : ______

Final decision          : GO / GO WITH CAUTION / NO-GO
Notes                   : ______________________________
                         ______________________________
                         ______________________________
```

---

# 7. Lệnh chạy tự hành sau khi PASS

```bash
cd ~/mbrobot_ws
source install/setup.bash
ros2 launch mobile_robot nav_launch.py
```

Chạy theo tầng/map cụ thể:

```bash
ros2 launch mobile_robot nav_launch.py floor:=e6
```

---

# 8. Checklist rút gọn dùng tại hiện trường

1. ☐ Robot cơ khí ổn định, vùng xung quanh trống.
2. ☐ Jetson không quá nóng, RAM còn đủ, disk còn đủ.
3. ☐ `/dev/ttyUSB1` tồn tại và đọc/ghi được.
4. ☐ `/dev/i2c-8` tồn tại.
5. ☐ `i2cdetect -y 8` thấy BNO055 tại `0x28`.
6. ☐ Ping LiDAR `192.168.11.2` thành công, packet loss 0%.
7. ☐ `/scan`, `/scan_filtered`, `/imu/data`, `/odom`, `/odometry/filtered` có data đúng tần số.
8. ☐ TF `odom -> base_footprint`, `base_footprint -> laser_frame`, `map -> odom` đầy đủ.
9. ☐ Nav2 node chính đã active.
10. ☐ E-stop kích hoạt và reset đúng.
11. ☐ Motor test chỉ chạy khi khu vực an toàn và người vận hành sẵn sàng dừng robot.
12. ☐ Không có FAIL trước khi gửi goal tự hành.
