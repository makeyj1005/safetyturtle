#!/usr/bin/env python3
"""make_voice_lines.py — edge-tts로 알림 음성 mp3를 미리 만든다.

무료(계정·API키 불필요)이면서 gTTS보다 자연스럽고, espeak-ng보다 훨씬 낫다
(2026-09-02 실측 비교 후 결정 — ko-KR-InJoonNeural 선택, ElevenLabs는 무료 플랜이
API 자체를 막아서 못 씀). 문구를 바꾸면 이 스크립트를 다시 돌려서 mp3만 새로 만들면
된다 — 노드 코드는 파일 경로만 참조하므로 안 바뀐다.

  pip3 install edge-tts
  python3 ~/vibe/ex1/tools/make_voice_lines.py
"""
import asyncio
import os

import edge_tts

OUT_DIR = os.path.join(os.path.expanduser("~"), "vibe", "ex1", "sounds")
VOICE = "ko-KR-InJoonNeural"

# rate: 경고성 문구는 조금 빠르게(+12%) 해서 긴박감을 준다. 평상시 안내는 기본 속도.
LINES = {
    "helmet_bad": (
        "안전모 미착용이 확인되었습니다. 즉시 안전모를 착용하시고, "
        "미보유 시 작업을 중지해 주십시오.", "+12%"),
    "gauge_ok": ("소화기 압력계 범위가 정상입니다", "+0%"),
    "fire_alarm": (
        "화재가 감지되었습니다. 화재가 감지되었습니다. 침착하게 대피해 주십시오. "
        "지금부터 비상구로 안내하겠습니다. 자세를 낮추고 젖은 천으로 코와 입을 막아 주십시오.",
        "+12%"),
    "intrusion": (
        "경고. 무단 출입이 감지되었습니다. 즉시 구역 밖으로 이동해 주십시오. "
        "현재 상황이 녹화되고 있습니다.", "+12%"),
}


async def make_one(name, text, rate):
    path = os.path.join(OUT_DIR, f"{name}.mp3")
    communicate = edge_tts.Communicate(text, VOICE, rate=rate)
    await communicate.save(path)
    print(f"생성됨: {path}")


async def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    for name, (text, rate) in LINES.items():
        await make_one(name, text, rate)


if __name__ == "__main__":
    asyncio.run(main())
