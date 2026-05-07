#!/bin/bash
# ============================================================
# preflight_check.sh — Mobile Robot Hardware Preflight Check
# Môi trường: Jetson AGX Xavier / Ubuntu 20.04 / ROS 2 Foxy
#
# Cách dùng:
#   bash preflight_check.sh              # kiểm tra chuẩn
#   bash preflight_check.sh --motor-test # kèm test motor (robot CÓ THỂ di chuyển!)
#   bash preflight_check.sh --no-ros     # chỉ kiểm tra OS/device, không cần ROS node
#
# Lưu kết quả:
#   bash preflight_check.sh 2>&1 | tee ~/robot_logs/preflight_$(date +%Y%m%d_%H%M%S).log
# ============================================================

# ── Cấu hình — chỉnh theo thực tế ────────────────────────────────────────────
LIDAR_IP="192.168.11.2"
I2C_BUS=8
IMU_ADDR="28"
UART_DEVICE="/dev/ttyUSB1"
ROS_WS="$HOME/mbrobot_ws"
EXPECTED_DOMAIN_ID="42"
EXPECTED_RMW="rmw_fastrtps_cpp"

# Ngưỡng tần số (Hz)
HZ_SCAN=7.0
HZ_IMU=45.0
HZ_ODOM=80.0
HZ_EKF=45.0

# Ngưỡng tài nguyên
MAX_TEMP_WARN=75    # °C
MAX_TEMP_FAIL=85    # °C
MIN_RAM_WARN=1500   # MB
MIN_RAM_FAIL=800    # MB
# ─────────────────────────────────────────────────────────────────────────────

# ── Màu terminal ──────────────────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
BLUE='\033[0;34m'; CYAN='\033[0;36m'; BOLD='\033[1m'; RESET='\033[0m'

# ── Counters ──────────────────────────────────────────────────────────────────
PASS=0; WARN=0; FAIL=0
FAILED_ITEMS=()

# ── Helper functions ──────────────────────────────────────────────────────────
pass()  { echo -e "  ${GREEN}✔  PASS${RESET}  $1"; ((PASS++)); }
warn()  { echo -e "  ${YELLOW}⚠  WARN${RESET}  $1"; ((WARN++)); }
fail()  {
  echo -e "  ${RED}✘  FAIL${RESET}  $1"
  ((FAIL++))
  FAILED_ITEMS+=("$1")
}
info()    { echo -e "  ${BLUE}ℹ  INFO${RESET}  $1"; }
section() { echo -e "\n${CYAN}${BOLD}══════════════════════════════════════════${RESET}"
            echo -e "${CYAN}${BOLD}  $1${RESET}"
            echo -e "${CYAN}${BOLD}══════════════════════════════════════════${RESET}"; }

# ── Parse flags ───────────────────────────────────────────────────────────────
MOTOR_TEST=false
NO_ROS=false
for arg in "$@"; do
  case "$arg" in
    --motor-test) MOTOR_TEST=true ;;
    --no-ros)     NO_ROS=true ;;
  esac
done

echo ""
echo -e "${BOLD}╔══════════════════════════════════════════════╗${RESET}"
echo -e "${BOLD}║   MOBILE ROBOT — HARDWARE PREFLIGHT CHECK   ║${RESET}"
echo -e "${BOLD}║   $(date '+%Y-%m-%d %H:%M:%S')                     ║${RESET}"
echo -e "${BOLD}╚══════════════════════════════════════════════╝${RESET}"
[ "$MOTOR_TEST" = true ] && echo -e "${RED}${BOLD}  [!] MOTOR TEST MODE — robot có thể di chuyển!${RESET}"
[ "$NO_ROS" = true ]     && info "NO-ROS mode: bỏ qua kiểm tra ROS topics/TF"

# ════════════════════════════════════════════════════════════════════════════
section "LEVEL 1 — TÀI NGUYÊN HỆ THỐNG"

# Nhiệt độ CPU
TEMP_RAW=$(cat /sys/devices/virtual/thermal/thermal_zone0/temp 2>/dev/null || echo 0)
TEMP=$(echo "scale=1; $TEMP_RAW / 1000" | bc 2>/dev/null || echo "N/A")
if [ "$TEMP_RAW" -lt $((MAX_TEMP_WARN * 1000)) ] 2>/dev/null; then
  pass "Jetson CPU temp: ${TEMP}°C (< ${MAX_TEMP_WARN}°C)"
elif [ "$TEMP_RAW" -lt $((MAX_TEMP_FAIL * 1000)) ] 2>/dev/null; then
  warn "Jetson CPU temp HIGH: ${TEMP}°C — monitor closely"
else
  fail "Jetson CPU temp CRITICAL: ${TEMP}°C — đợi nguội trước khi chạy"
fi

# RAM
FREE_RAM=$(free -m 2>/dev/null | awk '/^Mem:/{print $7}')
if [ "${FREE_RAM:-0}" -gt 3000 ]; then
  pass "RAM available: ${FREE_RAM} MB"
elif [ "${FREE_RAM:-0}" -gt "$MIN_RAM_WARN" ]; then
  warn "RAM available thấp: ${FREE_RAM} MB — tắt browser/ứng dụng thừa"
else
  fail "RAM available CRITICAL: ${FREE_RAM} MB — không đủ tài nguyên"
fi

# Disk
FREE_DISK=$(df -m / 2>/dev/null | awk 'NR==2{print $4}')
if [ "${FREE_DISK:-0}" -gt 5000 ]; then
  pass "Disk free: ${FREE_DISK} MB"
else
  warn "Disk free thấp: ${FREE_DISK} MB — có thể ảnh hưởng logging"
fi

# ════════════════════════════════════════════════════════════════════════════
section "LEVEL 1 — DEVICE PORTS"

# UART STM32
if [ -e "$UART_DEVICE" ]; then
  # Kiểm tra quyền truy cập
  if [ -r "$UART_DEVICE" ] && [ -w "$UART_DEVICE" ]; then
    pass "UART $UART_DEVICE tồn tại và có quyền đọc/ghi"
  else
    warn "UART $UART_DEVICE tồn tại nhưng thiếu quyền — thêm user vào group dialout"
  fi
else
  # Thử tìm device USB-TTL khác
  ALT=$(ls /dev/ttyUSB* 2>/dev/null | head -1)
  if [ -n "$ALT" ]; then
    warn "UART $UART_DEVICE không tìm thấy, có thể dùng: $ALT"
  else
    fail "Không tìm thấy UART device — kiểm tra cáp USB-TTL (J33)"
  fi
fi

# I2C Bus
if [ -e "/dev/i2c-$I2C_BUS" ]; then
  pass "I2C bus /dev/i2c-$I2C_BUS tồn tại"
else
  fail "I2C bus /dev/i2c-$I2C_BUS không tìm thấy — kiểm tra J23 wiring"
fi

# IMU BNO055 qua i2cdetect
if command -v i2cdetect &>/dev/null; then
  if i2cdetect -y "$I2C_BUS" 2>/dev/null | grep -q " $IMU_ADDR "; then
    pass "IMU BNO055 detected tại I2C-$I2C_BUS addr=0x$IMU_ADDR"
  else
    fail "IMU BNO055 KHÔNG detect được — kiểm tra SDA/SCL/power/bus number"
  fi
else
  warn "i2cdetect không có — không kiểm tra được IMU ở level OS. Cài: sudo apt install i2c-tools"
fi

# LiDAR ping — auto-reconnect nếu fail
LIDAR_IF="enp2s0"
info "Ping LiDAR tại $LIDAR_IP..."
if PING_OUT=$(ping -c 3 -W 2 "$LIDAR_IP" 2>&1); then
  RTT=$(echo "$PING_OUT" | grep -oP 'rtt.*= \K[0-9.]+(?=/)' || echo "?")
  LOSS=$(echo "$PING_OUT" | grep -oP '[0-9]+(?=% packet loss)' || echo "0")
  if [ "${LOSS:-0}" -eq 0 ]; then
    pass "LiDAR $LIDAR_IP: reachable, RTT avg = ${RTT}ms, 0% loss"
  else
    warn "LiDAR $LIDAR_IP: ${LOSS}% packet loss — kiểm tra cáp LAN"
  fi
else
  warn "LiDAR $LIDAR_IP: không ping được — đang thử kết nối lại interface $LIDAR_IF..."
  sudo ip addr add 192.168.11.1/24 dev "$LIDAR_IF" 2>/dev/null || true
  sudo ip link set "$LIDAR_IF" up 2>/dev/null || true
  sleep 2
  if PING_OUT2=$(ping -c 3 -W 2 "$LIDAR_IP" 2>&1); then
    RTT2=$(echo "$PING_OUT2" | grep -oP 'rtt.*= \K[0-9.]+(?=/)' || echo "?")
    pass "LiDAR $LIDAR_IP: reachable sau reconnect $LIDAR_IF, RTT = ${RTT2}ms"
  else
    fail "LiDAR $LIDAR_IP: UNREACHABLE ngay cả sau khi reconnect $LIDAR_IF — kiểm tra cáp LAN"
  fi
fi

# ════════════════════════════════════════════════════════════════════════════
if [ "$NO_ROS" = false ]; then

section "LEVEL 2 — ROS TOPICS (yêu cầu nodes đang chạy)"

info "Kiểm tra topics — đảm bảo launch file đã chạy và ổn định..."

# ── Hàm kiểm tra tần số topic ───────────────────────────────────────────────
check_hz() {
  local topic=$1
  local min_hz=$2
  local label=$3

  # Kiểm tra topic có tồn tại không
  if ! ros2 topic list 2>/dev/null | grep -q "^$topic$"; then
    fail "$label: topic '$topic' không tồn tại (node chưa start?)"
    return
  fi

  # Đo tần số trong 6 giây
  HZ_VAL=$(timeout 7 ros2 topic hz "$topic" --window 5 2>/dev/null \
    | grep -oP 'average rate: \K[0-9.]+' | tail -1)

  if [ -z "$HZ_VAL" ]; then
    fail "$label: KHÔNG có data từ '$topic'"
    return
  fi

  if awk "BEGIN{exit !($HZ_VAL >= $min_hz)}"; then
    pass "$label: ${HZ_VAL} Hz ≥ ${min_hz} Hz  [$topic]"
  else
    fail "$label: ${HZ_VAL} Hz < ${min_hz} Hz (quá chậm) [$topic]"
  fi
}

check_hz "/scan"               $HZ_SCAN  "LiDAR /scan"
check_hz "/scan_filtered"      $HZ_SCAN  "Laser filter /scan_filtered"
check_hz "/imu/data"           $HZ_IMU   "IMU /imu/data"
check_hz "/odom"               $HZ_ODOM  "Wheel odometry /odom"
check_hz "/odometry/filtered"  $HZ_EKF   "EKF /odometry/filtered"

# ─────────────────────────────────────────────────────────────────────────────
section "LEVEL 2 — TF TREE"

check_tf() {
  local parent=$1 child=$2
  if timeout 4 ros2 run tf2_ros tf2_echo "$parent" "$child" 2>/dev/null \
      | grep -q "Translation"; then
    pass "TF: $parent → $child"
  else
    fail "TF: $parent → $child  MISSING (check EKF/SLAM/URDF)"
  fi
}

check_tf "odom"           "base_footprint"
check_tf "base_footprint" "laser_frame"
check_tf "map"            "odom"

# ─────────────────────────────────────────────────────────────────────────────
section "LEVEL 2 — NAV2 LIFECYCLE NODES"

NAV2_NODES=$(ros2 node list 2>/dev/null | grep -cE "(amcl|controller_server|planner_server|bt_navigator|map_server)" || echo 0)
if [ "$NAV2_NODES" -ge 4 ]; then
  pass "Nav2 nodes active: ${NAV2_NODES} node(s) found"
else
  warn "Nav2 nodes: chỉ tìm thấy ${NAV2_NODES}/5 — kiểm tra nav_launch.py"
fi

# ─────────────────────────────────────────────────────────────────────────────
section "LEVEL 2 — EMERGENCY STOP TEST"

info "Kích hoạt e-stop..."
ros2 topic pub --once /robot/emergency_stop std_msgs/msg/Bool "{data: true}" &>/dev/null
sleep 2

E_STATE=$(timeout 3 ros2 topic echo /robot/state --once 2>/dev/null \
  | grep "data:" | awk '{print $2}' || echo "")

if echo "$E_STATE" | grep -q "EMERGENCY_STOP"; then
  pass "E-Stop kích hoạt: state = EMERGENCY_STOP"
else
  fail "E-Stop kích hoạt THẤT BẠI: state = '${E_STATE:-không đọc được}'"
fi

info "Reset e-stop..."
ros2 topic pub --once /robot/emergency_stop std_msgs/msg/Bool "{data: false}" &>/dev/null
sleep 2

E_STATE2=$(timeout 3 ros2 topic echo /robot/state --once 2>/dev/null \
  | grep "data:" | awk '{print $2}' || echo "")

if echo "$E_STATE2" | grep -q "IDLE"; then
  pass "E-Stop reset: state = IDLE"
else
  warn "E-Stop reset: state = '${E_STATE2:-không đọc được}' (mong đợi IDLE)"
fi

fi  # end of NO_ROS check

# ════════════════════════════════════════════════════════════════════════════
if [ "$MOTOR_TEST" = true ]; then
  section "LEVEL 3 — MOTOR TEST (Trên sàn)  ⚠️"
  echo ""
  echo -e "${RED}${BOLD}  ⚠️  CẢNH BÁO: Robot SẼ di chuyển trên sàn!${RESET}"
  echo -e "${YELLOW}  Robot sẽ tiến ~10cm rồi lùi ~10cm về chỗ cũ (v = 0.1 m/s).${RESET}"
  echo -e "${YELLOW}  Đảm bảo vùng 50cm phía trước và sau robot hoàn toàn trống.${RESET}"
  echo ""
  read -rp "  Nhấn ENTER để tiến hành, Ctrl+C để hủy..."
  echo ""

  read -rp "  Xác nhận vùng di chuyển trống? [y/N] " CONFIRM
  if [[ "$CONFIRM" != "y" && "$CONFIRM" != "Y" ]]; then
    info "Motor test bị hủy bởi user."
  else
    # ── Tiến ~10cm: 0.1 m/s × 1.0s (STM32 firmware tự ramp acc) ─────────
    info "Tiến: linear.x = 0.1 m/s trong 1.0s (≈10cm)..."
    ros2 topic pub --rate 20 /cmd_vel geometry_msgs/msg/Twist \
      "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" &>/dev/null &
    CMD_PID=$!
    sleep 1.0
    kill "$CMD_PID" 2>/dev/null; wait "$CMD_PID" 2>/dev/null

    # Dừng — chờ ổn định
    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}" &>/dev/null
    sleep 0.5

    ODOM_X=$(timeout 2 ros2 topic echo /odom --field pose.pose.position.x --once 2>/dev/null \
      | grep -oP '[0-9.-]+' | head -1 || echo "N/A")
    info "/odom position.x sau tiến: ${ODOM_X} m"

    # ── Lùi ~10cm: -0.1 m/s × 1.0s ──────────────────────────────────────
    info "Lùi: linear.x = -0.1 m/s trong 1.0s (≈10cm về chỗ cũ)..."
    ros2 topic pub --rate 20 /cmd_vel geometry_msgs/msg/Twist \
      "{linear: {x: -0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" &>/dev/null &
    CMD_PID=$!
    sleep 1.0
    kill "$CMD_PID" 2>/dev/null; wait "$CMD_PID" 2>/dev/null

    # Dừng hoàn toàn
    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}" &>/dev/null
    sleep 0.3

    ODOM_X2=$(timeout 2 ros2 topic echo /odom --field pose.pose.position.x --once 2>/dev/null \
      | grep -oP '[0-9.-]+' | head -1 || echo "N/A")
    info "/odom position.x sau lùi: ${ODOM_X2} m"

    pass "Motor test hoàn thành — kiểm tra thủ công:"
    info "  - Robot tiến ~10cm rồi lùi về chỗ cũ?"
    info "  - Cả 2 bánh quay đều, không lệch?"
    info "  - /odom phản hồi đúng chiều (x tăng khi tiến, giảm khi lùi)?"
    info "  - Không có tiếng kêu bất thường?"
  fi
fi

# ════════════════════════════════════════════════════════════════════════════
section "KẾT QUẢ TỔNG HỢP"

echo ""
echo -e "  ${GREEN}PASS: ${PASS}${RESET}  |  ${YELLOW}WARN: ${WARN}${RESET}  |  ${RED}FAIL: ${FAIL}${RESET}"
echo ""

if [ "${#FAILED_ITEMS[@]}" -gt 0 ]; then
  echo -e "${RED}  Các mục FAIL:${RESET}"
  for item in "${FAILED_ITEMS[@]}"; do
    echo -e "  ${RED}  ✘ $item${RESET}"
  done
  echo ""
fi

if [ "$FAIL" -eq 0 ] && [ "$WARN" -le 2 ]; then
  echo -e "${GREEN}${BOLD}  ╔══════════════════════════════════════╗${RESET}"
  echo -e "${GREEN}${BOLD}  ║  ✅  GO — SẴN SÀNG CHẠY TỰ HÀNH   ║${RESET}"
  echo -e "${GREEN}${BOLD}  ╚══════════════════════════════════════╝${RESET}"
  echo ""
  echo "  Lệnh chạy tự hành:"
  echo "    ros2 launch mobile_robot nav_launch.py"
  echo "    ros2 launch mobile_robot nav_launch.py floor:=e6"
  exit 0
elif [ "$FAIL" -eq 0 ]; then
  echo -e "${YELLOW}${BOLD}  ╔══════════════════════════════════════╗${RESET}"
  echo -e "${YELLOW}${BOLD}  ║  ⚠   GO WITH CAUTION — $WARN cảnh báo  ║${RESET}"
  echo -e "${YELLOW}${BOLD}  ╚══════════════════════════════════════╝${RESET}"
  echo ""
  echo "  Xem xét giải quyết WARN trước khi chạy demo thực tế."
  exit 0
else
  echo -e "${RED}${BOLD}  ╔══════════════════════════════════════╗${RESET}"
  echo -e "${RED}${BOLD}  ║  🚫  NO-GO — $FAIL lỗi nghiêm trọng       ║${RESET}"
  echo -e "${RED}${BOLD}  ╚══════════════════════════════════════╝${RESET}"
  echo ""
  echo "  Debug các mục FAIL trước khi chạy tự hành!"
  exit 1
fi
