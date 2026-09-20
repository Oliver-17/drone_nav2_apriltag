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
#      PX4_GZ_WORLD=empty_nav2 DRONES=1 ./start_arena_sitl.sh  # 空白世界
#      START_NODE=1 DRONES=1 ./start_arena_sitl.sh  # 第一台生在拓樸節點 1
#      START_POSE=0,5 DRONES=1 ./start_arena_sitl.sh # 直接指定第一台 ENU 起始位置
#      HEADLESS=1 ./start_arena_sitl.sh   # 無視窗
#      GZ_RAM_LIMIT=6G ./start_arena_sitl.sh  # 用 cgroup 限制 PX4/Gazebo 記憶體
#      GZ_RAM_LIMIT=off ./start_arena_sitl.sh # 不限制記憶體
#
#  停止：
#      pkill -x px4 ; pkill -f "gz sim"
#      （第二行一定要用 -f：gz 是 Ruby 包裝腳本，程序名是 ruby，-x 抓不到）
# =============================================================================
set -e

PX4_DIR="${PX4_DIR:-/home/zhg/ncrl_mqtt/PX4-Autopilot}" #這裡只是針對我的路徑
# PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}" 通用應該是這個
BUILD_DIR="$PX4_DIR/build/px4_sitl_default"
# 這支腳本在 <pkg>/scripts/ 底下，往上一層就是套件根目錄
PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DRONES="${DRONES:-3}"
GZ_RAM_LIMIT="${GZ_RAM_LIMIT:-8G}"
case "$GZ_RAM_LIMIT" in
    off|OFF|none|NONE|0) GZ_RAM_LIMIT="" ;;
esac

if [ ! -x "$BUILD_DIR/bin/px4" ]; then
    echo "找不到 $BUILD_DIR/bin/px4，請先執行： cd $PX4_DIR && make px4_sitl_default"
    exit 1
fi

# === 自動相容記憶體限制 (cgroup v2 -> cgroup v1 -> ulimit) ===================
if [ -n "$GZ_RAM_LIMIT" ]; then
    # 將 8G / 6G 轉成 KB 單位給 ulimit 使用
    case "$GZ_RAM_LIMIT" in
        *G|*g) RAM_KB=$((${GZ_RAM_LIMIT%[Gg]} * 1024 * 1024)) ;;
        *M|*m) RAM_KB=$((${GZ_RAM_LIMIT%[Mm]} * 1024)) ;;
        *)     RAM_KB="" ;;
    esac

    SET_OK=0

    # 1. 嘗試 cgroup v2
    if [ -d "/sys/fs/cgroup" ] && mkdir -p "/sys/fs/cgroup/gz_limit" 2>/dev/null; then
        if echo "$GZ_RAM_LIMIT" > "/sys/fs/cgroup/gz_limit/memory.max" 2>/dev/null; then
            echo $$ > "/sys/fs/cgroup/gz_limit/cgroup.procs" 2>/dev/null || true
            echo "✓ 已成功透過 cgroup v2 設置記憶體上限：$GZ_RAM_LIMIT"
            SET_OK=1
        fi
    fi

    # 2. 嘗試 cgroup v1 (若環境屬於舊版 cgroup)
    if [ "$SET_OK" -eq 0 ] && [ -d "/sys/fs/cgroup/memory" ] && mkdir -p "/sys/fs/cgroup/memory/gz_limit" 2>/dev/null; then
        if [ -n "$RAM_KB" ] && echo "$((RAM_KB * 1024))" > "/sys/fs/cgroup/memory/gz_limit/memory.limit_in_bytes" 2>/dev/null; then
            echo $$ > "/sys/fs/cgroup/memory/gz_limit/tasks" 2>/dev/null || true
            echo "✓ 已成功透過 cgroup v1 設置記憶體上限：$GZ_RAM_LIMIT"
            SET_OK=1
        fi
    fi

    # 3. 若容器限制寫入 /sys/fs/cgroup，自動降級使用原生 ulimit 限制
    if [ "$SET_OK" -eq 0 ] && [ -n "$RAM_KB" ]; then
        if ulimit -v "$RAM_KB" 2>/dev/null; then
            echo "✓ 已透過 ulimit 限制進程虛擬記憶體上限：$GZ_RAM_LIMIT ($RAM_KB KB)"
            SET_OK=1
        fi
    fi

    if [ "$SET_OK" -eq 0 ]; then
        echo "⚠ 容器環境限制無法更改記憶體上限，改用未限制模式"
    fi
else
    echo "記憶體限制：off"
fi
# =============================================================================

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
# 先 source PX4 的 gz_env.sh 取得 PX4 的模型目錄（x500 本體與 lidar mesh 在那裡）、外掛路徑與
# server config，再把「世界」改指到本套件。順序不能反 ——
# gz_env.sh 是無條件覆寫 PX4_GZ_WORLDS 的，先設會被蓋掉。
# shellcheck disable=SC1091
source "$BUILD_DIR/rootfs/gz_env.sh"

# px4-rc.gzsim:51 組出來的路徑是 "${PX4_GZ_WORLDS}/${PX4_GZ_WORLD}.sdf"
export PX4_GZ_WORLDS="$PKG_DIR/gz/worlds"
PX4_GZ_WORLD="${PX4_GZ_WORLD:-${WORLD:-nav2_arena}}"
export PX4_GZ_WORLD

# 本套件的模型（apriltag_36h11、x500_nav2）要讓 Gazebo 能展開 model:// URI。
export GZ_SIM_RESOURCE_PATH="$PKG_DIR/gz/models:$GZ_SIM_RESOURCE_PATH"

# 機體模型走的是另一條路：px4-rc.gzsim:137 組出來的是
# file://${PX4_GZ_MODELS}/${MODEL_NAME}/model.sdf ——
# 寫死單一目錄，不吃 GZ_SIM_RESOURCE_PATH，所以自製機體必須把 PX4_GZ_MODELS
# 整個指過來。（先前這裡的註解寫「PX4_GZ_MODELS 不能改」是錯的：
# 上面 gz_env.sh 那行 GZ_SIM_RESOURCE_PATH=...:$PX4_GZ_MODELS:... 在 source
# 當下就把 PX4 的模型目錄烤進搜尋路徑了，之後改 PX4_GZ_MODELS 也不影響
# x500_nav2 內部那句 <uri>model://x500</uri> 的解析。）
export PX4_GZ_MODELS="$PKG_DIR/gz/models"

echo "世界： $PX4_GZ_WORLDS/$PX4_GZ_WORLD.sdf"
echo "機體： $PX4_GZ_MODELS/${SIM_MODEL:-x500_depth_nav2}/model.sdf"

NAMES=("MAV1" "MAV2" "MAV3")
# ENU：第一個是東、第二個是北。預設第一台擺在拓樸圖節點 0 (-2, 0)。
# 若要避免一開始正對牆，可用 START_NODE=1 讓 MAV1 從節點 1 (0, 5) 開始。
# fly_nodes 也要同步帶 start_node:=1，PX4 local origin 才會和 graph 起點一致。
POSES=("-2,0"  "-2,3"  "-2,-3")
START_NODE="${START_NODE:-0}"
START_POSE="${START_POSE:-}"
case "$START_NODE" in
    0) NODE_START_POSE="-2,0" ;;
    1) NODE_START_POSE="0,5" ;;
    2) NODE_START_POSE="6.5,5" ;;
    3) NODE_START_POSE="13,5" ;;
    4) NODE_START_POSE="1,-6.5" ;;
    5) NODE_START_POSE="6.5,-6.5" ;;
    6) NODE_START_POSE="13,-6.5" ;;
    7) NODE_START_POSE="15,0" ;;
    8) NODE_START_POSE="20,9" ;;
    9) NODE_START_POSE="25,14" ;;
    10) NODE_START_POSE="27,16" ;;
    *)
        echo "未知 START_NODE=$START_NODE，改用節點 0。可用 START_POSE=E,N 直接指定。"
        NODE_START_POSE="-2,0"
        START_NODE=0
        ;;
esac
if [ -z "$START_POSE" ]; then
    if [ "$PX4_GZ_WORLD" = "empty_nav2" ]; then
        START_POSE="0,0"
    else
        START_POSE="$NODE_START_POSE"
    fi
fi
POSES[0]="$START_POSE"

# 機體模型：x500_nav2 = x500 + 前視相機 + 下視相機 + 2D 光達，
# 定義在 gz/models/x500_nav2/。飛行物理與 x500 完全相同，所以機型仍用 4001。
# 為什麼是 PX4_SIM_MODEL 而不是 PX4_GZ_MODEL：後者自 v1.15 起已廢棄，
# 整個 PX4 原始碼都不再讀它（只剩 docs/en/sim_gazebo_gz/index.md:267 的說明），
# 之前寫 PX4_GZ_MODEL=x500 其實沒有生效，會出 x500 純粹是因為
# airframes/4001_gz_x500 裡的預設值 PX4_SIM_MODEL=${PX4_SIM_MODEL:=x500}。
SIM_MODEL="${SIM_MODEL:-x500_depth_nav2}"

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

world_is_up()       { gz topic -l 2>/dev/null | grep -qE "/world/${PX4_GZ_WORLD}/clock"; }
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
    "" --instance "" set SYS_HAS_BARO 0        >/dev/null 2>&1 || return 1
    "" --instance "" set SYS_HAS_MAG 0         >/dev/null 2>&1 || return 1
    

    "$param" --instance "$i" save                       >/dev/null 2>&1 || return 1
    return 0
}

# --- 以 cgroup 限制 Gazebo/PX4 可用記憶體 -----------------------------------
# PX4 的第一個 instance 會負責啟動 Gazebo；用 systemd scope 包住 px4，可讓
# Gazebo 子程序也落在同一個 MemoryMax 限制裡。root shell 用 system scope，
# 一般使用者用 user scope；若系統不支援則退回原本直接啟動方式。
launch_px4_instance() {
    local i="$1"
    local name="$2"
    local pose="$3"
    local work_dir="$4"

    local -a env_args=(
        "PX4_UXRCE_DDS_NS=$name"
        "PX4_SYS_AUTOSTART=4001"
        "PX4_SIM_MODEL=$SIM_MODEL"
        "PX4_GZ_MODEL_POSE=$pose"
        "HEADLESS=${HEADLESS:-}"
    )
    local -a px4_cmd=("$BUILD_DIR/bin/px4" -i "$i" -d "$BUILD_DIR/etc")

    cd "$work_dir"
    env "${env_args[@]}" "${px4_cmd[@]}"
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

    if [ "$i" -eq 0 ]; then
        echo "啟動 $NAME  (instance $i, MAV_SYS_ID $((i+1)), START_NODE=$START_NODE, 位置 E,N = $POSE)"
    else
        echo "啟動 $NAME  (instance $i, MAV_SYS_ID $((i+1)), 位置 E,N = $POSE)"
    fi

    launch_px4_instance "$i" "$NAME" "$POSE" "$WORK_DIR" \
        > "$WORK_DIR/out.log" 2>&1 &

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
