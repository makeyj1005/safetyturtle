#!/bin/bash
# start_all.sh — 순찰 로봇 전체를 한 번에 띄운다 (로봇 노드 + 노트북 노드 + 대시보드).
#
# [노트북에서 실행]
#   ~/vibe/ex1/tools/start_all.sh            # 전부 켜기
#   ~/vibe/ex1/tools/start_all.sh --no-drive # 주행 조작 없이 (센서·감지만)
#   ~/vibe/ex1/tools/start_all.sh --no-rear  # 후면 CSI 노드 없이
#   ~/vibe/ex1/tools/start_all.sh --stop     # 전부 끄기
#
# [왜 스크립트가 필요한가 — 2026-09-03]
# 세션마다 명령 10개를 손으로 치고 있었다(bringup, gpio, speaker, lcd, webcam,
# mux, fire, restricted, helmet, extinguisher, dashboard). 순서와 옵션을 하나만
# 틀려도 조용히 안 되는 게 있어서(예: helmet_node 의 manage_camera 를 안 끄면
# 컨테이너에서 ssh 를 시도해 실패) 한곳에 모았다.
#
# 웹페이지의 "전체 시동" 버튼과 다른 점: 이 스크립트는 **프로세스를 띄우고**,
# 웹 버튼은 이미 떠 있는 노드의 **감지를 켠다**. 대시보드는 컨테이너 안에서 돌아
# 도커·ssh 를 건드릴 수 없어서 이 둘을 나눴다.
set -u

ROBOT="${ROBOT:-rpi@192.168.0.73}"
EX1="$HOME/vibe/ex1"
MAP="${MAP:-/home/rpi/vibe/ex1/maps/venue2_map.yaml}"

# --- 웹캠(전면 USB) 화질 설정 ---
# 원본 기본값은 640x480 / 3fps / jpeg50 이었다 — 무선이 병목이라 Nav2 주행 중에
# /scan 이 밀리지 않게 낮게 잡은 값이다(HANDOFF 제약 5).
# 지금은 Nav2 를 안 쓰는 시연 구성이고 CPU·GPU 에 여유가 많아서(2026-09-04 실측:
# GPU 2%, 12코어 중 load 2.2) 올려 쓴다.
#
# [무엇이 실제로 좋아지는가 — 헛된 기대를 막기 위해]
#   fps ↑     판단이 빨라진다. helmet_node 는 표본 N장을 모아 결론내므로
#             fps 가 높으면 같은 표본을 더 짧은 시간에 모은다(judge_sec 과 짝).
#   해상도 ↑  안전모 **색 판정**은 좋아진다(머리 영역 픽셀이 늘어난다).
#             하지만 **사람 검출은 거의 안 좋아진다** — MobileNet-SSD 가 내부에서
#             300x300 으로 줄여 처리하기 때문이다.
#   화각      소프트웨어로 넓힐 수 없다(렌즈 성질). 다만 저해상도에서 센서를
#             크롭하는 카메라라면 해상도를 올릴 때 넓어질 수 있다 — 실측 필요.
#
# ⚠️ 무선이 이 프로젝트의 병목이다. 올린 뒤 반드시 확인할 것:
#      ros2 topic hz /scan        (라이다가 밀리면 안전기능·Nav2 가 무너진다)
#      ros2 topic hz /webcam/image_raw/compressed
#    /scan 이 흔들리면 fps 나 해상도를 되돌린다.
# ⚠️ Nav2 자율주행을 켤 때는 원본값(3fps / 640x480 / jpeg50)으로 되돌릴 것.
WEBCAM_FPS="${WEBCAM_FPS:-15.0}"
WEBCAM_W="${WEBCAM_W:-1280}"
WEBCAM_H="${WEBCAM_H:-720}"
WEBCAM_JPEG="${WEBCAM_JPEG:-70}"

# --- CSI(후면) 화질 ---
# 압력계 판정은 해상도가 결정적이다: 70cm 거리에서 640x480 이면 게이지가 19px 로
# 바늘을 볼 수 없고, 1640x1232 면 약 48px 이 되어 판정이 가능하다.
# 게다가 gauge.judge 는 캘리브레이션 해상도와 다르면 아예 판정을 거부한다.
# 1640x1232 는 CMA 메모리 상한이기도 하다(그 위는 "Cannot allocate memory").
CSI_W="${CSI_W:-1640}"
CSI_H="${CSI_H:-1232}"
CSI_FPS="${CSI_FPS:-5}"
SSH_OPTS=(-o ConnectTimeout=8 -o BatchMode=yes -o StrictHostKeyChecking=accept-new)

# 컨테이너 공통 옵션.
# --ipc=host 가 반드시 필요하다: Fast-RTPS 는 같은 기계 안에서 공유메모리(/dev/shm)로
# 통신하는데, --network host 는 네트워크 네임스페이스만 공유하고 IPC 는 컨테이너마다
# 격리된 채로 남는다. 이걸 빼면 "발행은 되는데 구독 쪽에 하나도 안 온다"가 된다.
DOCKER_COMMON=(--rm --network host --ipc=host -e ROS_DOMAIN_ID=3
               -v "$EX1:/root/vibe/ex1")
GPU_OPTS=(--device=/dev/kfd --device=/dev/dri --group-add 990 --group-add video
          -e HSA_OVERRIDE_GFX_VERSION=11.0.0)

CONTAINERS=(cmd_vel_mux dashboard fire_node restricted_node helmet_node
            extinguisher_node helmet_rear_node extinguisher_inspect_node)
ROBOT_NODES=(gpio_io_node speaker_node lcd_node webcam_node)

say() { printf '\n\033[1m== %s\033[0m\n' "$*"; }
ok()  { printf '  \033[32m✓\033[0m %s\n' "$*"; }
bad() { printf '  \033[31m✗\033[0m %s\n' "$*"; }

# ---------------------------------------------------------------- 끄기
if [ "${1:-}" = "--stop" ]; then
    say "노트북 컨테이너 정지"
    for c in "${CONTAINERS[@]}"; do
        docker stop "$c" >/dev/null 2>&1 && ok "$c" || true
    done
    say "로봇 노드 정지"
    # pkill -f 금지 규칙(HANDOFF2 3.3)을 지키려고 [문]자 트릭을 쓴다 —
    # 그냥 pkill -f 하면 이 ssh 명령 자신까지 패턴에 걸려 죽는다.
    for n in "${ROBOT_NODES[@]}"; do
        ssh "${SSH_OPTS[@]}" "$ROBOT" "pkill -f '[${n:0:1}]${n:1}\.py'" >/dev/null 2>&1
        ok "$n"
    done
    echo
    echo "bringup 은 남겨둔다(재기동 비용이 크고, 껐다 바로 켜면 OpenCR 이"
    echo "'stack smashing' 으로 죽는다). 정말 끄려면:"
    echo "  ssh $ROBOT \"pkill -9 -f '[r]obot.launch.py'\""
    exit 0
fi

WITH_DRIVE=1
WITH_REAR=1
for a in "$@"; do
    case "$a" in
        --no-drive) WITH_DRIVE=0 ;;
        --no-rear)  WITH_REAR=0 ;;   # CSI 미연결 상태에서 헛돌지 않게
    esac
done

# ---------------------------------------------------------------- 사전 점검
say "로봇 접속 확인"
if ! ssh "${SSH_OPTS[@]}" "$ROBOT" "echo ok" >/dev/null 2>&1; then
    bad "로봇($ROBOT)에 접속할 수 없다"
    echo "     - 로봇 전원이 켜져 있는지 (부팅에 1분쯤 걸린다)"
    echo "     - 노트북이 team1 와이파이에 붙어 있는지"
    echo "     - IP 가 바뀌었는지 (공유기 192.168.0.1 의 DHCP 목록에서 ros18 확인)"
    exit 1
fi
ok "$ROBOT"

# ---------------------------------------------------------------- 로봇: bringup
say "로봇 bringup (모터·라이다·IMU)"
if ssh "${SSH_OPTS[@]}" "$ROBOT" "pgrep -f '[r]obot.launch.py' >/dev/null"; then
    ok "이미 실행 중 — 그대로 쓴다"
else
    # TURTLEBOT3_MODEL·LDS_MODEL 을 명시해야 한다. 비대화형 ssh 는 .bashrc 를
    # 안 읽어서, 로봇 계정에 이미 있는 값이 안 넘어온다.
    ssh "${SSH_OPTS[@]}" "$ROBOT" '
        export ROS_DOMAIN_ID=3 TURTLEBOT3_MODEL=burger LDS_MODEL=LDS-01
        source /opt/ros/humble/setup.bash
        rm -f ~/bringup.log
        ( setsid nohup ros2 launch turtlebot3_bringup robot.launch.py \
              > ~/bringup.log 2>&1 < /dev/null & )' >/dev/null 2>&1
    printf '  기다리는 중'
    for _ in $(seq 1 20); do
        if ssh "${SSH_OPTS[@]}" "$ROBOT" \
             "grep -q 'diff_drive_controller.*Run!' ~/bringup.log" 2>/dev/null; then
            echo; ok "기동 완료"; break
        fi
        if ssh "${SSH_OPTS[@]}" "$ROBOT" \
             "grep -qE 'stack smashing|died' ~/bringup.log" 2>/dev/null; then
            echo; bad "bringup 이 죽었다 (OpenCR 이 안정될 시간이 필요하다)"
            echo "     잠시 뒤 다시 실행할 것 — ssh $ROBOT 'tail -20 ~/bringup.log'"
            exit 1
        fi
        printf '.'; sleep 2
    done
fi

# ---------------------------------------------------------------- 로봇: 센서·스피커·LCD·웹캠
say "로봇 노드 (센서·스피커·LCD·웹캠)"
for n in "${ROBOT_NODES[@]}"; do
    if ssh "${SSH_OPTS[@]}" "$ROBOT" "pgrep -f '[${n:0:1}]${n:1}\.py' >/dev/null"; then
        ok "$n (이미 실행 중)"
    else
        # 웹캠만 화질 인자를 넘긴다(위 주석 참고). 나머지는 인자가 없다.
        args=""
        [ "$n" = "webcam_node" ] && args="--ros-args -p fps:=$WEBCAM_FPS \
            -p width:=$WEBCAM_W -p height:=$WEBCAM_H -p jpeg_quality:=$WEBCAM_JPEG"
        ssh "${SSH_OPTS[@]}" "$ROBOT" "
            export ROS_DOMAIN_ID=3
            source /opt/ros/humble/setup.bash
            ( setsid nohup python3 -u ~/launch/$n.py $args > ~/$n.log 2>&1 < /dev/null & )" \
            >/dev/null 2>&1
        ok "$n${args:+ (${WEBCAM_W}x${WEBCAM_H} @${WEBCAM_FPS}fps jpeg${WEBCAM_JPEG})}"
    fi
done

# ---------------------------------------------------------------- 노트북 컨테이너
run_node() {   # run_node <컨테이너이름> <gpu|cpu> <ros2 명령...>
    local name="$1" kind="$2"; shift 2
    if docker ps --format '{{.Names}}' | grep -qx "$name"; then
        ok "$name (이미 실행 중)"
        return
    fi
    docker rm -f "$name" >/dev/null 2>&1 || true
    local -a extra=()
    local image="patrol-ros2:humble"
    if [ "$kind" = "gpu" ]; then
        extra=("${GPU_OPTS[@]}")
        image="patrol-ros2:gpu"
    fi
    docker run -d "${DOCKER_COMMON[@]}" "${extra[@]}" --name "$name" "$image" \
        bash -c "source /opt/ros/humble/setup.bash
                 source /root/vibe/ex1/ros2_ws/install/setup.bash
                 $*" >/dev/null && ok "$name" || bad "$name 시작 실패"
}

say "노트북 노드"
if [ "$WITH_DRIVE" = "1" ]; then
    run_node cmd_vel_mux cpu "ros2 run patrol_core cmd_vel_mux"
else
    echo "  (--no-drive: 주행 중재 노드는 띄우지 않는다 — 웹에서 조작해도 안 움직인다)"
fi

run_node dashboard cpu "python3 -u /root/vibe/ex1/tools/dashboard_server.py"

run_node fire_node cpu \
    "ros2 run patrol_core fire_node --ros-args -p use_nav:=false -p sound:=false"

# 사람 검출은 GPU(YOLO)가 훨씬 빠르다 — CPU MobileNet-SSD 는 46% 를 먹었다.
run_node restricted_node gpu \
    "ros2 run patrol_core restricted_node --ros-args -p detector:=yolo -p sound:=false"

# manage_camera:=false 가 중요하다: true 면 helmet_node 가 ssh 로 로봇 웹캠을
# 띄우려 하는데, 컨테이너에 ssh 키가 없어 실패한다. 웹캠은 위에서 이미 띄웠다.
run_node helmet_node cpu \
    "ros2 run patrol_core helmet_node --ros-args -p method:=color \
     -p manage_camera:=false -p sound:=false -p hold:=false"

run_node extinguisher_node cpu "ros2 run patrol_core extinguisher_expiry_node"

# --- 후면(CSI) 카메라 + 안전모 감시 + 소화기 점검 ---
# [왜 후면도 보는가] 앞에서만 안전모를 쓰고 로봇이 지나가면 벗는 경우를 잡으려고.
# 앞 인스턴스와 이름·상태토픽·사진접두어·DB이름을 모두 다르게 준다(절대 규칙 7).
if [ "$WITH_REAR" = "1" ]; then
    say "로봇 CSI 카메라 (후면)"
    ssh "${SSH_OPTS[@]}" "$ROBOT" \
        "~/launch/csi_camera.sh $CSI_W $CSI_H $CSI_FPS" 2>&1 | sed 's/^/  /'

    run_node helmet_rear_node cpu \
        "ros2 run patrol_core helmet_node --ros-args -r __node:=helmet_node_rear \
         -p topic:=/csi/image_raw/compressed -p method:=color \
         -p manage_camera:=false -p sound:=false -p hold:=false \
         -p status_topic:=/helmet_rear/status -p evidence_prefix:=helmet_rear \
         -p db_node_name:=helmet_node_rear"

    run_node extinguisher_inspect_node cpu \
        "ros2 run patrol_core extinguisher_inspect_node --ros-args \
         -p topic:=/csi/image_raw/compressed"
else
    echo "  (--no-rear: 후면 CSI 노드는 띄우지 않는다)"
fi

# ---------------------------------------------------------------- 확인
say "확인"
sleep 6
if command -v curl >/dev/null && curl -sf http://localhost:8080/api/status >/dev/null 2>&1; then
    ok "대시보드: http://localhost:8080"
else
    bad "대시보드가 아직 응답하지 않는다 (docker logs dashboard 로 확인)"
fi

batt=$(docker run --rm "${DOCKER_COMMON[@]}" patrol-ros2:humble bash -c \
       "source /opt/ros/humble/setup.bash
        timeout 6 ros2 topic echo /battery_state --once 2>/dev/null \
          | grep -m1 voltage | awk '{print \$2}'" 2>/dev/null)
if [ -n "$batt" ]; then
    # 11.3V 아래면 주행 금지(11.0V 에서 노드가 죽는다) — HANDOFF2 5.0
    if awk "BEGIN{exit !($batt < 11.3)}"; then
        bad "배터리 ${batt}V — 11.3V 미만이라 주행 금지. 충전할 것"
    else
        ok "배터리 ${batt}V"
    fi
else
    bad "배터리를 읽지 못했다 (bringup 확인)"
fi

echo
echo "전부 켜졌다. 웹페이지에서 '전체 시동' 을 누르면 감지가 시작된다."
echo "끄기: $0 --stop"
