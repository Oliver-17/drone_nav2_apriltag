#!/usr/bin/env bash
# =============================================================================
#  start_arena_sitl.sh — 在自製的 nav2_arena 世界裡啟動 PX4 SITL
#
#  跟 drone_control/scripts/start_3_px4.sh 的差別只有「用哪個世界」：
#  那支跑 PX4 內建的 default 世界，這支跑本套件的 gz/worlds/nav2_arena.sdf。
#  其餘（NVIDIA offload、GZ_IP、預檢參數）完全沿用，因為那些坑跟世界無關。
#
#  用法：
#      ./start_arena_sitl.sh              # 三台，有視窗
#      DRONES=1 ./start_arena_sitl.sh     # 只開一台（純看場景時比較快）
#      HEADLESS=1 ./start_arena_sitl.sh   # 無視窗
#
#  停止：
#      pkill -x px4 ; pkill -f "gz sim"
#      （第二行一定要用 -f：gz 是 Ruby 包裝腳本，程序名是 ruby，-x 抓不到）
# =============================================================================
set -e

PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
BUILD_DIR="$PX4_DIR/build/px4_sitl_default"
# 這支腳本在 <pkg>/scripts/ 底下，往上一層就是套件根目錄
PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DRONES="${DRONES:-3}"

if [ ! -x "$BUILD_DIR/bin/px4" ]; then
    echo "找不到 $BUILD_DIR/bin/px4，請先執行： cd $PX4_DIR && make px4_sitl_default"
    exit 1
fi

# --- 強制 Gazebo 走 NVIDIA 獨顯 -----------------------------------------------
# 雙顯卡 + PRIME on-demand 的預設是走內顯，三機同場畫不動 → 物理步進被拖慢
# → lockstep 下 PX4 收不到 IMU → Accel TIMEOUT → EKF 劣化 → 預檢失敗。
if command -v nvidia-smi >/dev/null 2>&1; then
    export __NV_PRIME_RENDER_OFFLOAD=1
    export __GLX_VENDOR_LIBRARY_NAME=nvidia
    export __VK_LAYER_NV_optimus=NVIDIA_only
    echo "已啟用 NVIDIA offload"
else
    echo "找不到 nvidia-smi，維持預設顯示卡"
fi

# --- 把 gz-transport 綁在回環位址（2026-08-31 實測的關鍵修正）----------------
# 不設的話 gz-transport 綁到所有網路介面，IMU 傳遞出現抖動 → Accel TIMEOUT
# → EKF 劣化 → 起飛後失效保護 RTL → 翻覆墜毀。詳見 start_3_px4.sh 的長註解。
export GZ_IP=127.0.0.1

# --- Gazebo 資源路徑 ---------------------------------------------------------
# 先 source PX4 的 gz_env.sh 取得 PX4_GZ_MODELS（x500 在那裡）、外掛路徑與
# server config，再把「世界」改指到本套件。順序不能反 ——
# gz_env.sh 是無條件覆寫 PX4_GZ_WORLDS 的，先設會被蓋掉。
# shellcheck disable=SC1091
source "$BUILD_DIR/rootfs/gz_env.sh"

# px4-rc.gzsim:51 組出來的路徑是 "${PX4_GZ_WORLDS}/${PX4_GZ_WORLD}.sdf"
export PX4_GZ_WORLDS="$PKG_DIR/gz/worlds"
export PX4_GZ_WORLD=nav2_arena

# AprilTag 模型在本套件裡，要讓 Gazebo 展開 <uri>model://apriltag_36h11</uri>。
# 注意 PX4_GZ_MODELS 不能改 —— px4-rc.gzsim:137 是
# file://${PX4_GZ_MODELS}/${MODEL_NAME}/model.sdf，寫死單一目錄不是搜尋路徑，
# 改掉的話 x500 就找不到了。自製模型只能靠 GZ_SIM_RESOURCE_PATH 這條路進來。
export GZ_SIM_RESOURCE_PATH="$PKG_DIR/gz/models:$GZ_SIM_RESOURCE_PATH"

echo "世界： $PX4_GZ_WORLDS/$PX4_GZ_WORLD.sdf"

NAMES=("MAV1" "MAV2" "MAV3")
# ENU：第一個是東、第二個是北。擺在拓樸圖節點 0 (-2, 0) 附近，
# T3 照節點順序飛的時候起點才對得上。三台沿南北排開，間隔 3 公尺。
# 節點 0 的座標定義在 scripts/gen_arena.py，那邊改了這裡也要跟著改。
POSES=("-2,0"  "-2,3"  "-2,-3")

# --- 小工具：輪詢等待某個條件成立 -------------------------------------------
wait_for() {
    local desc="$1" timeout="$2"; shift 2
    local waited=0
    while ! "$@" >/dev/null 2>&1; do
        sleep 1
        waited=$((waited + 1))
        if [ "$waited" -ge "$timeout" ]; then
            echo "  ✗ 逾時（${timeout}s）：$desc"
            return 1
        fi
    done
    echo "  ✓ $desc（耗時 ${waited}s）"
    return 0
}

world_is_up()       { gz topic -l 2>/dev/null | grep -qE "/world/nav2_arena/clock"; }
instance_is_ready() { grep -q "uxrce_dds_client.*vehicle_local_position" "$1"; }

# 「就緒」的判斷是 uxrce_dds_client 有沒有把 vehicle_local_position 註冊出去，
# 而那要等 XRCE Agent 接上才會發生。Agent 沒開的話 PX4 本身跑得好好的，
# 只是這個檢查會白等 90 秒 —— 純粹要看場景時很浪費，所以先問清楚。
if ! pgrep -x MicroXRCEAgent >/dev/null 2>&1; then
    echo
    echo "  ⚠ 沒偵測到 MicroXRCEAgent。"
    echo "    只想看場景 → 不用理它，等下的「已連上 XRCE Agent」會逾時，那是正常的。"
    echo "    要用 ROS 2 控制 → 先另開一個終端跑： MicroXRCEAgent udp4 -p 8888"
    echo
    WAIT_AGENT=15      # 沒 Agent 就不要白等 90 秒
else
    WAIT_AGENT=90
fi

# --- 補上 v1.17 SITL 的預檢參數 ----------------------------------------------
# 機型檔 4001_gz_x500 會 set-default NAV_DLL_ACT 2，沒開 QGC 就 ARM 不起來。
fix_preflight_params() {
    local i="$1"
    local param="$BUILD_DIR/bin/px4-param"
    "$param" --instance "$i" set NAV_DLL_ACT 0          >/dev/null 2>&1 || return 1
    "$param" --instance "$i" set CBRK_SUPPLY_CHK 894281 >/dev/null 2>&1 || return 1
    "$param" --instance "$i" save                       >/dev/null 2>&1 || return 1
    return 0
}

# --- 清理舊程序 --------------------------------------------------------------
echo "清掉可能殘留的舊程序…"
pkill -x px4 || true
pkill -f "gz sim" || true      # 注意是 -f，不是 -x
sleep 2

# --- 依序啟動 ----------------------------------------------------------------
for i in $(seq 0 $((DRONES - 1))); do
    NAME="${NAMES[$i]}"
    POSE="${POSES[$i]}"
    WORK_DIR="$BUILD_DIR/instance_$i"

    mkdir -p "$WORK_DIR"
    rm -f "$WORK_DIR/out.log"

    echo "啟動 $NAME  (instance $i, MAV_SYS_ID $((i+1)), 位置 E,N = $POSE)"

    (
        cd "$WORK_DIR"
        PX4_UXRCE_DDS_NS="$NAME" \
        PX4_SYS_AUTOSTART=4001 \
        PX4_GZ_MODEL=x500 \
        PX4_GZ_MODEL_POSE="$POSE" \
        HEADLESS="${HEADLESS:-}" \
        "$BUILD_DIR/bin/px4" -i "$i" -d "$BUILD_DIR/etc" \
            > "$WORK_DIR/out.log" 2>&1 &
    )

    # 第一台負責建立世界，後面的會偵測到世界已存在而直接加入
    # （px4-rc.gzsim:35 用 `gz topic -l | grep /world/*/clock` 判斷）。
    if [ "$i" -eq 0 ]; then
        wait_for "Gazebo 世界 nav2_arena 已建立" 90 world_is_up
    fi

    wait_for "$NAME 已連上 XRCE Agent" "$WAIT_AGENT" instance_is_ready "$WORK_DIR/out.log" || true

    if fix_preflight_params "$i"; then
        echo "  ✓ $NAME 預檢參數已設定（NAV_DLL_ACT=0, CBRK_SUPPLY_CHK）"
    else
        echo "  ⚠ $NAME 預檢參數設定失敗 —— ARM 可能會被擋"
    fi
done

echo
echo "=================================================="
echo " ${DRONES} 台就緒，世界： nav2_arena"
echo "=================================================="
echo "log 位置："
for i in $(seq 0 $((DRONES - 1))); do echo "  ${NAMES[$i]}: $BUILD_DIR/instance_$i/out.log"; done
echo
echo "停止： pkill -x px4 ; pkill -f 'gz sim'"
