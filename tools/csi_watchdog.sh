#!/bin/bash
# csi_watchdog.sh — CSI 카메라가 멎으면 알아서 다시 띄운다. 로봇에서 실행.
#
# [왜 필요한가 — 2026-09-06]
# camera_ros 가 프로세스는 살아 있는데 발행만 멈추는 일이 하루에 두 번 있었다.
# 로그에는 아무 오류도 안 남는다. 그래서 "죽었나?" 하고 pgrep 해보면 멀쩡히
# 보이고, 정작 화면만 몇 분째 정지해 있다.
# 소화기 점검은 이 카메라로 하므로, 멎은 줄 모르고 순찰을 돌면 QR 을 못 읽어
# 점검을 통째로 건너뛴다(실제로 그렇게 시연 한 번을 날렸다).
#
# 프로세스 존재 여부로는 못 잡으니 **토픽이 실제로 흐르는지**로 판단한다.
#
# [실행]
#   ~/launch/csi_watchdog.sh --daemon
#   ~/launch/csi_watchdog.sh --stop
set -u

W=1640
H=1232
FPS=5
CHECK_SEC=20        # 몇 초마다 볼지
LOG="$HOME/csi_watchdog.log"

if [ "${1:-}" = "--stop" ]; then
    pkill -f '[c]si_watchdog.sh' && echo "감시 정지" || echo "실행 중이 아니었다"
    exit 0
fi

if [ "${1:-}" = "--daemon" ]; then
    # 뒤에서 도는 쪽은 인자 없이 실행된다 — 줄 끝($)까지 맞춰야 자기 자신이
    # 안 걸린다(restart_bringup.sh 주석과 같은 이유).
    if pgrep -f '[c]si_watchdog\.sh$' >/dev/null; then
        echo "이미 실행 중"
        exit 0
    fi
    ( setsid nohup "$0" > "$LOG" 2>&1 < /dev/null & )
    sleep 1
    pgrep -f '[c]si_watchdog\.sh' >/dev/null \
        && echo "CSI 감시 시작 (로그: $LOG)" || echo "시작 실패"
    exit 0
fi

export ROS_DOMAIN_ID=3
source /opt/ros/humble/setup.bash

log() { printf '[%s] %s\n' "$(date '+%H:%M:%S')" "$*"; }

restart_csi() {
    log "CSI 재시작"
    # ros2 run 껍데기와 실제 실행파일을 모두 죽인다. 하나만 죽이면 좀비가 남아
    # 다음 시작이 "이미 실행 중" 으로 막힌다.
    pkill -9 -f '[c]amera_ros' 2>/dev/null
    pkill -9 -f '[c]amera_node' 2>/dev/null
    sleep 3
    "$HOME/launch/csi_camera.sh" "$W" "$H" "$FPS" >> "$LOG" 2>&1
    sleep 8
}

log "CSI 감시 시작 — ${CHECK_SEC}초마다 토픽 확인"
fails=0
while true; do
    # 5초 안에 한 장이라도 오면 살아있는 것으로 본다.
    # 프로세스 유무가 아니라 **토픽이 흐르는지**를 본다는 게 핵심이다.
    if timeout 6 ros2 topic echo /csi/image_raw/compressed --once \
            > /dev/null 2>&1; then
        if [ "$fails" -gt 0 ]; then
            log "정상 복귀"
        fi
        fails=0
    else
        fails=$((fails + 1))
        log "프레임 없음 (${fails}회째)"
        # 한 번 놓친 것으로는 재시작하지 않는다 — 무선이 잠깐 흔들려도
        # 실패로 잡히는데, 그때마다 껐다 켜면 오히려 더 자주 끊긴다.
        if [ "$fails" -ge 2 ]; then
            restart_csi
            fails=0
        fi
    fi
    sleep "$CHECK_SEC"
done
