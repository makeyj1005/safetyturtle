#!/bin/bash
# 시동.sh — 순찰 로봇 전체를 켠다. 이것 하나만 실행하면 된다.
#
# [쓰는 법]
#   ~/vibe/ex1/시동.sh          전부 켜기
#   ~/vibe/ex1/시동.sh 정지     전부 끄기
#   ~/vibe/ex1/시동.sh 상태     지금 상태만 보기
#
# 로봇 전원을 켜고 **1분쯤 기다렸다가** 실행할 것.
# 부팅이 덜 됐으면 로봇 노드를 못 올린다.
set -u

EX1="$HOME/vibe/ex1"
ROBOT="rpi@192.168.0.73"
SSH_OPTS=(-o ConnectTimeout=8 -o BatchMode=yes -o StrictHostKeyChecking=accept-new)

hr()  { printf '\n\033[1m%s\033[0m\n' "════ $* ════"; }
ok()  { printf '  \033[32m✓\033[0m %s\n' "$*"; }
bad() { printf '  \033[31m✗\033[0m %s\n' "$*"; }

상태보기() {
    hr "상태"
    if ping -c 1 -W 2 "${ROBOT#*@}" >/dev/null 2>&1; then
        ok "로봇 연결됨"
    else
        bad "로봇이 응답하지 않는다 — 전원을 확인할 것"
    fi

    local n
    n=$(docker ps --format '{{.Names}}' 2>/dev/null | wc -l)
    echo "  노트북 컨테이너 ${n}개: $(docker ps --format '{{.Names}}' 2>/dev/null | sort | tr '\n' ' ')"

    if pgrep -f '[s]upervisor\.sh$' >/dev/null; then
        ok "웹 재시작 버튼 감시 실행 중"
    else
        bad "감시 프로세스 꺼짐 — 웹의 '전체 시동' 버튼이 동작하지 않는다"
    fi

    local code
    code=$(curl -s -o /dev/null -w '%{http_code}' --max-time 5 http://localhost:8080/ 2>/dev/null)
    if [ "$code" = "200" ]; then
        ok "웹페이지 http://localhost:8080  (폰: http://$(hostname -I | awk '{print $1}'):8080)"
        curl -s --max-time 5 http://localhost:8080/api/status 2>/dev/null | python3 -c "
import json,sys
try:
    d=json.load(sys.stdin)
except Exception:
    sys.exit()
b=d.get('battery') or {}
f=lambda v: '–' if v is None else round(v,1)
print(f\"  전면 카메라 {f(d.get('camera_age_sec'))}초 전 / 후면 {f(d.get('camera_rear_age_sec'))}초 전\")
v=b.get('voltage')
print(f\"  전원 {v:.2f}V\" if v else '  전원 정보 없음')
" 2>/dev/null
    else
        bad "웹페이지가 응답하지 않는다 (HTTP ${code:-없음})"
    fi
}

case "${1:-시동}" in
정지|stop|--stop)
    hr "전부 끄기"
    "$EX1/tools/start_all.sh" --stop
    # 감시 프로세스도 같이 내린다. 안 내리면 다음에 켤 때 두 개가 돈다.
    pkill -f '[s]upervisor\.sh$' 2>/dev/null && ok "감시 프로세스 정지" || true
    ;;
상태|status)
    상태보기
    ;;
*)
    hr "순찰 로봇 시동"
    if ! ping -c 1 -W 2 "${ROBOT#*@}" >/dev/null 2>&1; then
        bad "로봇이 응답하지 않는다"
        echo "     - 로봇 전원이 켜져 있는지 확인 (부팅에 1분쯤 걸린다)"
        echo "     - 노트북이 team1 와이파이에 붙어 있는지 확인"
        exit 1
    fi
    ok "로봇 연결 확인"

    # 웹의 '전체 시동/정지' 버튼이 동작하려면 이 감시 프로세스가 있어야 한다.
    # 컨테이너가 아니라 호스트에서 도는 프로세스라, 컨테이너를 다 내려도
    # 남아 있어야 정상이다. 어쩌다 죽으면 버튼이 조용히 먹통이 된다.
    "$EX1/tools/supervisor.sh" --daemon

    "$EX1/tools/start_all.sh"
    상태보기
    echo
    echo "  문제가 있으면: $0 상태"
    ;;
esac
