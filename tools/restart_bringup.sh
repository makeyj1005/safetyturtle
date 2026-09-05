#!/bin/bash
# restart_bringup.sh — 로봇에서 bringup 을 안전하게 껐다 켠다.
#
# [왜 스크립트로 따로 두는가]
# ssh 로 한 줄씩 보내면 두 가지 함정에 걸린다.
#
#  ① pkill 자기 자신 죽이기
#     `ssh 로봇 'pkill -f "[r]obot.launch.py" ... ros2 launch ... robot.launch.py'`
#     처럼 한 명령 안에 죽일 패턴과 켤 명령을 같이 넣으면, ssh 세션의 셸
#     cmdline 에 "robot.launch.py" 가 그대로 들어 있어서 pkill 이 **자기 자신을**
#     죽인다. 대괄호 트릭([r]obot)은 패턴 문자열만 가려줄 뿐, 뒤에 있는 실제
#     launch 명령까지 가려주지 못한다.
#
#  ② ssh 가 끊기면 작업이 중간에 멈춤
#     turtlebot3_node 를 죽이면 OpenCR 이 리셋되면서 순간적으로 전류를 끌어가고,
#     그때 파이의 와이파이(SDIO)가 흔들려 ssh 가 툭 끊긴다(2026-09-05 실측,
#     3번 반복). 그러면 "죽이기"만 되고 "켜기"는 실행되지 않아 좀비만 남는다.
#
# 그래서 이 스크립트를 로봇에 두고 **분리(setsid)** 실행한다. ssh 가 끊겨도
# 끝까지 돈다. 파일 안에 있으므로 ssh cmdline 에 패턴이 노출되지도 않는다.
#
# [사용]
#   ssh rpi@<IP> 'setsid nohup ~/launch/restart_bringup.sh > ~/restart.log 2>&1 &'
# set -u 를 쓰지 않는다 — ROS 의 setup.bash 는 정의되지 않은 변수를 참조해서
# (AMENT_TRACE_SETUP_FILES 등) set -u 상태에서 source 하면 그 자리에서 죽는다.
# 2026-09-05 에 이걸로 스크립트가 아무 일도 안 하고 끝났다.

export TURTLEBOT3_MODEL=burger
export LDS_MODEL=LDS-01
export ROS_DOMAIN_ID=3
source /opt/ros/humble/setup.bash

log() { printf '[%s] %s\n' "$(date '+%H:%M:%S')" "$*"; }

log "기존 프로세스 정리"
# 대괄호 트릭 — 이 스크립트 파일 안에서는 자기 자신이 걸릴 일이 없지만,
# 습관을 유지한다(ps 출력에 grep 자신이 섞이는 것도 막는다).
pkill -f '[r]obot\.launch\.py' 2>/dev/null
pkill -9 -f '[t]urtlebot3_ros' 2>/dev/null
pkill -9 -f '[h]lds_laser' 2>/dev/null

# OpenCR 이 완전히 놓여날 시간. 짧으면 다음 실행이 세그폴트(-11)로 죽는다.
# 실측: 6초는 부족했고 15초면 됐다.
log "OpenCR 안정 대기 15초"
sleep 15

log "bringup 시작"
setsid nohup ros2 launch turtlebot3_bringup robot.launch.py \
    > "$HOME/bringup.log" 2>&1 < /dev/null &

sleep 20

if pgrep -f '[t]urtlebot3_ros' >/dev/null; then
    log "turtlebot3_node 살아있음 (OK)"
else
    log "turtlebot3_node 없음 — bringup.log 확인 필요"
fi

if grep -q "process has died" "$HOME/bringup.log" 2>/dev/null; then
    log "경고: 죽은 프로세스가 있다"
    grep "process has died" "$HOME/bringup.log" | tail -2
fi

log "완료"
