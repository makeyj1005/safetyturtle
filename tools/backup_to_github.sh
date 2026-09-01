#!/bin/bash
# backup_to_github.sh — events.sqlite 등 변경분을 GitHub(ex1-code 브랜치)에 주기적으로 백업.
# crontab 에 등록해 이 노트북이 켜져있는 동안 자동 실행되게 한다.
set -e
cd "$HOME/vibe/ex1"
git add logs/events.sqlite maps/ 2>/dev/null || true
if ! git diff --cached --quiet; then
    git commit -m "자동 백업 $(date '+%Y-%m-%d %H:%M')" >/dev/null
    git push origin ex1-code >/dev/null 2>&1
    echo "$(date): 백업 완료" >> "$HOME/vibe/ex1/logs/backup.log"
else
    echo "$(date): 변경 없음" >> "$HOME/vibe/ex1/logs/backup.log"
fi
