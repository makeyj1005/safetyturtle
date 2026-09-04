#!/bin/bash
# supervisor.sh — 웹페이지의 "전체 시동 / 전체 정지" 요청을 실제로 실행하는 감시 프로세스.
#
# [왜 이게 필요한가]
# 대시보드(dashboard_server.py)는 도커 컨테이너 안에서 돈다. 컨테이너 안에는
# docker 명령도, 로봇으로 가는 ssh 키도 없다. 그래서 대시보드는 프로세스를
# 직접 띄우거나 끌 수 없다 — 감지 기능을 켜고 끄는 토픽만 낼 수 있었다.
#
# 이 스크립트는 **호스트에서** 돌면서 공유 폴더의 요청 파일 하나만 지켜본다.
# 대시보드가 그 파일에 한 줄 쓰면, 여기서 start_all.sh 를 대신 실행한다.
# (도커 소켓을 컨테이너에 넣어주는 방법도 있지만, 그러면 웹페이지가 호스트의
#  모든 컨테이너를 다룰 권한을 갖게 된다. 파일 한 개로 정해진 두 동작만
#  받는 편이 훨씬 좁고 안전하다.)
#
# [실행]
#   ~/vibe/ex1/tools/supervisor.sh          # 앞에서 실행(로그가 보인다)
#   ~/vibe/ex1/tools/supervisor.sh --daemon # 뒤에서 실행
#   ~/vibe/ex1/tools/supervisor.sh --stop
#
# start_all.sh 가 자동으로 이걸 띄우므로 보통은 따로 실행할 필요가 없다.
set -u

EX1="$HOME/vibe/ex1"
CTL="$EX1/logs/control"
REQ="$CTL/request"          # 대시보드가 쓴다: restart | stop
STATUS="$CTL/status"        # 우리가 쓴다: 대시보드가 읽어 화면에 보여준다
LOG="$CTL/supervisor.log"

mkdir -p "$CTL"

if [ "${1:-}" = "--stop" ]; then
    pkill -f '[s]upervisor.sh' && echo "감시 프로세스 정지" || echo "실행 중이 아니었다"
    exit 0
fi

if [ "${1:-}" = "--daemon" ]; then
    # 뒤에서 도는 쪽은 인자 없이 실행된다("$0" 만). 그래서 줄 끝($)까지 맞춰
    # 찾아야 한다 — '[s]upervisor.sh' 만 쓰면 지금 실행 중인 이 `--daemon`
    # 명령 자신이 걸려서 항상 "이미 실행 중"이 되어버린다.
    if pgrep -f '[s]upervisor\.sh$' >/dev/null; then
        echo "이미 실행 중"
        exit 0
    fi
    ( setsid nohup "$0" > "$LOG" 2>&1 < /dev/null & )
    sleep 1
    pgrep -f '[s]upervisor.sh' >/dev/null \
        && echo "감시 프로세스 시작 (로그: $LOG)" \
        || echo "시작 실패 — $LOG 를 볼 것"
    exit 0
fi

# 대시보드가 읽을 상태를 한 줄로 남긴다. 형식: <상태>|<사람이 읽을 문구>
#   상태: idle | running | done | error
set_status() { printf '%s|%s\n' "$1" "$2" > "$STATUS"; }

log() { printf '[%s] %s\n' "$(date '+%H:%M:%S')" "$*"; }

set_status idle "대기 중"
log "감시 시작 — 요청 파일: $REQ"

while true; do
    if [ -f "$REQ" ]; then
        action="$(tr -d '[:space:]' < "$REQ")"
        # 먼저 지운다. 실행이 오래 걸리는 동안 같은 요청을 두 번 잡으면
        # start_all.sh 가 겹쳐 돌아 컨테이너 이름이 충돌한다.
        rm -f "$REQ"

        case "$action" in
        restart)
            log "전체 재시작 요청"
            set_status running "정지 중... (1/2)"
            # --keep-dash: 대시보드는 남긴다. 이걸 끄면 지금 보고 있는
            # 웹페이지가 죽어서 진행 상황을 볼 수 없게 된다.
            "$EX1/tools/start_all.sh" --stop --keep-dash >>"$LOG" 2>&1
            set_status running "시작 중... (2/2) 40초쯤 걸린다"
            if "$EX1/tools/start_all.sh" >>"$LOG" 2>&1; then
                log "재시작 완료"
                set_status done "재시작 완료"
            else
                log "재시작 실패 — 로그 확인"
                set_status error "재시작 실패 (logs/control/supervisor.log 확인)"
            fi
            ;;
        stop)
            log "전체 정지 요청"
            set_status running "정지 중..."
            if "$EX1/tools/start_all.sh" --stop --keep-dash >>"$LOG" 2>&1; then
                log "정지 완료"
                set_status done "전체 정지 완료 (대시보드만 남았다)"
            else
                set_status error "정지 실패 (logs/control/supervisor.log 확인)"
            fi
            ;;
        "")
            ;;   # 빈 파일 — 무시
        *)
            log "모르는 요청: $action"
            set_status error "모르는 요청: $action"
            ;;
        esac
    fi
    sleep 2
done
