#!/usr/bin/env python3
"""shot_grab.py — 로봇에서 고해상도 프레임을 로컬 저장시킨 뒤 파일로 가져온다.

ROS 의존이 없는 순수 모듈이다(subprocess, tarfile 만 쓴다). inspect_node 와
tools/grab_shot.py 가 같은 방법으로 사진을 얻도록 여기 한 곳에 둔다.

[왜 스트리밍이 아니라 파일인가 — 2026-07-31 실측]
1640x1232 jpeg85 프레임은 120~180KB 다. 무선 DDS 로 보내면
  best_effort  20초에 0장   (조각 하나 잃으면 프레임 전체를 버린다)
  reliable     20초에 13장  (0.65fps, 최대 5.5초 공백)
로봇 안에서는 2.6fps 로 멀쩡하다. 점검은 정지 상태에서 몇 장만 필요하므로
로봇에서 로컬 저장 후 가져오는 편이 빠르고 확실하다.

[왜 scp 가 아니라 tar 스트림인가]
scp 로 `host:/tmp/grab/*.jpg` 를 받는 방식은 원격 글로브 확장에 의존한다.
파일이 없거나 확장이 안 되면 통째로 실패하고, 이유가 stderr 한 줄로만 남아
원인을 알기 어려웠다(tools/grab_shot.py 의 기존 버그). 그래서
  ssh 한 번  →  stdout = tar 스트림(바이너리),  stderr = 진행 로그(글자)
로 나눠 받는다. 연결 한 번이고, 글로브가 없고, 몇 장을 저장했는지도 같이 온다.

[카메라를 이 함수가 켜고 끄는 이유]
주행 중 카메라 스트림이 켜져 있으면 무선이 포화돼 /scan·/odom 이 밀리고 Nav2 가
목표를 거절한다(HANDOFF "함정 12", ping 1000ms 실측). 그래서 camera 인자를 주면
사진을 찍는 그 순간에만 로봇에서 카메라를 띄우고 바로 내린다.
"""
import io
import os
import subprocess
import tarfile
import time

# ssh 연결을 재사용한다(ControlMaster). 무선이 나쁠 때 접속 handshake 가 여러 번
# 왕복해 한 장에 30초 넘게 걸렸다 — 첫 연결만 맺고 그 뒤로는 같은 통로를 쓴다.
# ControlPersist 로 마지막 사용 후 그 시간만큼 통로를 살려둔다(조준은 몇 초 간격).
SSH_OPTS = ["-o", "ConnectTimeout=15", "-o", "BatchMode=yes",
            "-o", "ControlMaster=auto", "-o", "ControlPath=/tmp/ssh_grab_%r@%h",
            "-o", "ControlPersist=120"]

REMOTE_DIR = "/tmp/grab"
REMOTE_CAM_LAUNCH = "~/launch/robot_camera.launch.py"
# 카메라를 못 껐을 때 로봇이 보내는 표시. 호출한 쪽이 이걸 보고 경고한다.
CAM_LEFT = "camera STILL RUNNING"

# 로봇에서 실행되는 코드. 로컬 구독이라 유실이 없다.
# 진행 로그는 stderr 로 낸다 — stdout 은 tar 스트림 몫이다.
REMOTE_CODE = r'''
import os, sys, time, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

n_want = int(sys.argv[1]); topic = sys.argv[2]; out = sys.argv[3]
limit = float(sys.argv[4])
os.makedirs(out, exist_ok=True)
for f in os.listdir(out):
    try:
        os.remove(os.path.join(out, f))
    except OSError:
        pass

rclpy.init()
node = Node("grab_local")
got = []
def cb(m):
    if len(got) < n_want:
        p = os.path.join(out, "f%02d.jpg" % len(got))
        open(p, "wb").write(bytes(m.data))
        got.append(p)
# 발행자가 reliable 이어도 best_effort 구독은 호환된다. 로컬이라 유실이 없다.
node.create_subscription(CompressedImage, topic, cb, qos_profile_sensor_data)
t0 = time.time()
while len(got) < n_want and time.time() - t0 < limit:
    rclpy.spin_once(node, timeout_sec=0.1)
print("saved %d in %.1fs" % (len(got), time.time() - t0))
rclpy.shutdown()
'''


def camera_args(width, height, fps=3.0, jpeg_quality=85, index=1, reliability="reliable"):
    """robot_camera.launch.py 에 넘길 인자. 해상도는 캘리브레이션 값을 그대로 준다.

    ROI 가 픽셀 좌표라 캘리브레이션과 해상도가 다르면 gauge.judge 가 판정불가를 낸다.
    """
    return {
        "camera": str(int(index)),
        "width": str(int(width)),
        "height": str(int(height)),
        "fps": str(float(fps)),
        "jpeg_quality": str(int(jpeg_quality)),
        "reliability": str(reliability),
    }


def build_remote(n, topic, wait_sec, camera=None, remote_dir=REMOTE_DIR):
    """로봇에서 돌릴 셸 스크립트를 만든다. stdout=tar, stderr=로그."""
    lines = [
        "source /opt/ros/humble/setup.bash",
        f"mkdir -p {remote_dir}",
    ]
    if camera:
        opts = " ".join(f"{k}:={v}" for k, v in camera.items())
        lines += [
            f"ros2 launch {REMOTE_CAM_LAUNCH} {opts} >/tmp/grab_cam.log 2>&1 &",
            "CAM=$!",
            "echo \"camera launched pid $CAM\" 1>&2",
        ]
    # 파이썬의 stdout 을 stderr 로 돌린다. stdout 은 tar 만 쓴다.
    lines += [
        f"python3 - {n} {topic} {remote_dir} {wait_sec:.1f} <<'PYEOF' 1>&2",
        REMOTE_CODE,
        "PYEOF",
    ]
    if camera:
        # ⚠️ `ros2 launch` 를 죽여도 camera_node 자식은 살아남는다 (2026-08-01 실측:
        # kill -INT 뒤 -9 로 launch 를 없앴더니 camera_node 가 PPID=1 로 고아가 되어
        # 계속 스트리밍했다). 카메라가 남으면 주행 중 무선이 포화돼 Nav2 가 목표를
        # 거절한다 — 이 함수가 카메라를 켜고 끄는 이유가 통째로 무너진다.
        # 그래서 launch 가 아니라 **camera_node 를 이름으로** 확인하고 끝까지 없앤다.
        # pkill 패턴에 [c] 를 쓰는 이유: ssh 가 이 스크립트 전체를 셸 명령줄로 넘기므로
        # 패턴을 그대로 쓰면 pkill -f 가 자기 셸까지 죽인다.
        lines += [
            "kill -INT $CAM 2>/dev/null",
            "for i in 1 2 3 4 5 6 7 8; do "
            "pgrep -f '[c]amera_ros/camera_node' >/dev/null || break; sleep 0.5; done",
            # 아직 살아있으면 노드에 직접 SIGINT, 그래도 남으면 SIGKILL
            "pkill -INT -f '[c]amera_ros/camera_node' 2>/dev/null",
            "for i in 1 2 3 4 5 6; do "
            "pgrep -f '[c]amera_ros/camera_node' >/dev/null || break; sleep 0.5; done",
            "pkill -KILL -f '[c]amera_ros/camera_node' 2>/dev/null",
            "kill -9 $CAM 2>/dev/null",
            "wait $CAM 2>/dev/null",
            # 정말 없어졌는지 확인해서 보고한다. 조용히 남으면 다음 주행이 망가진다.
            "sleep 0.5",
            "if pgrep -f '[c]amera_ros/camera_node' >/dev/null; then "
            f"echo '{CAM_LEFT}' 1>&2; else echo 'camera stopped' 1>&2; fi",
        ]
    lines += [f"tar cf - -C {remote_dir} ."]
    return "\n".join(lines)


def ssh_cmd(host, cmd, timeout=40):
    if "@" not in host:
        host = f"rpi@{host}"
    return subprocess.run(["ssh", *SSH_OPTS, host, cmd],
                          capture_output=True, text=True, timeout=timeout)


def stop_camera(host, tries=3):
    """카메라를 확실히 없앤다. 남으면 다음 실행이 장치를 못 잡는다.

    매달린(hung) 카메라는 SIGTERM 으로 안 죽는다(실측: futex 에서 멈춘 채 25분간
    /dev/media0 을 붙들었다). SIGTERM -> SIGKILL 순서로 확인하며 없앤다.
    """
    for _ in range(tries):
        try:
            r = ssh_cmd(host, "pkill -f '[c]amera_ros/camera_node'; sleep 2; "
                              "pkill -9 -f '[c]amera_ros/camera_node'; sleep 1; "
                              "pgrep -f '[c]amera_ros/camera_node' >/dev/null "
                              "&& echo LEFT || echo GONE", timeout=45)
            if "GONE" in r.stdout:
                return True
        except subprocess.SubprocessError:
            pass
        time.sleep(2)
    return False


def start_camera(host, width, height, fps=3.0, jpeg_quality=85, index=0):
    """카메라를 켜두고 그대로 둔다(끄는 건 stop_camera).

    한 장 찍을 때마다 켰다 끄면 30~40초가 걸려 정렬 되먹임을 돌릴 수 없다.
    켜둔 채로는 한 장에 7~9초다(실측). 먼저 남아있는 카메라를 없앤다 — 장치를
    하나만 잡을 수 있어서 묵은 게 있으면 새 카메라가 그냥 죽는다.
    """
    if not stop_camera(host):
        return False
    opts = " ".join(f"{k}:={v}" for k, v in
                    camera_args(width, height, fps=fps, jpeg_quality=jpeg_quality,
                                index=index).items())
    cmd = (f"source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=3; "
           f"setsid nohup ros2 launch {REMOTE_CAM_LAUNCH} {opts} "
           f">/tmp/inspect_cam.log 2>&1 < /dev/null & echo started")
    try:
        return "started" in ssh_cmd(host, cmd).stdout
    except subprocess.SubprocessError:
        return False


def grab(host, out_dir, n=3, topic="/camera/image_raw/compressed", domain="3",
         camera=None, wait_sec=15.0, prefix=None, timeout=None):
    """로봇에서 프레임 n 장을 받아 out_dir 에 저장한다.

    host    : rpi@192.168.0.67 (또는 IP 만)
    camera  : None 이면 카메라가 이미 떠 있다고 보고 구독만 한다.
              dict(camera_args(...)) 를 주면 찍는 동안만 로봇에서 카메라를 띄운다.
    반환    : (파일경로 목록, 로그 문자열). 실패하면 목록이 빈다.
    """
    if "@" not in host:
        host = f"rpi@{host}"
    # 카메라를 띄우는 경우 기동 시간을 더해 여유를 준다.
    # 실측(2026-08-01, Pi 3): 카메라를 켜고 3장 받고 끄기까지 **27~43초**. 대부분이
    # ros2 launch 기동 시간이다. 여기에 무선이 나빠지면(손실 16%, RTT 300ms, ssh
    # 접속만 5초) 그대로 넘긴다 — 45초로 잡았다가 실제로 시간초과로 한 회차를 날렸다.
    # 넉넉히 잡아도 성공하면 그만큼 일찍 끝나므로 손해가 없다.
    # 카메라가 이미 켜져 있어도(camera=None) ssh 왕복만 30초를 넘길 때가 있다
    # (2026-08-01 실측: 무선이 나쁜 구간에서 35초 상한에 걸려 정렬을 통째로 놓쳤다).
    # 넉넉히 잡아도 성공하면 그만큼 일찍 끝나므로 손해가 없다.
    if timeout is None:
        timeout = wait_sec + (75.0 if camera else 45.0)
    remote = f"export ROS_DOMAIN_ID={domain}\n" + build_remote(n, topic, wait_sec, camera)

    try:
        r = subprocess.run(["ssh", *SSH_OPTS, host, remote],
                           capture_output=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        return [], (f"ssh 시간초과({timeout:.0f}초) — 무선이 느리거나 카메라 기동이 "
                    f"늦다. `ping {host.split('@')[-1]}` 로 왕복시간을 볼 것 "
                    f"(정상 30ms, 300ms 넘으면 포화) (host={host})")
    except OSError as e:
        return [], f"ssh 실행 실패: {e}"

    log = r.stderr.decode("utf-8", "replace").strip()
    if r.returncode != 0 and not r.stdout:
        return [], f"원격 실행 실패(rc={r.returncode}): {log[-400:]}"

    os.makedirs(out_dir, exist_ok=True)
    stamp = prefix or time.strftime("%H%M%S")
    files = []
    try:
        with tarfile.open(fileobj=io.BytesIO(r.stdout), mode="r|") as tf:
            for m in tf:
                if not m.isfile() or not m.name.endswith(".jpg"):
                    continue
                src = tf.extractfile(m)
                if src is None:
                    continue
                p = os.path.join(out_dir, f"{stamp}_{os.path.basename(m.name)}")
                with open(p, "wb") as f:
                    f.write(src.read())
                files.append(p)
    except tarfile.TarError as e:
        return [], f"tar 스트림을 풀 수 없다: {e} / 로그: {log[-300:]}"

    if CAM_LEFT in log:
        # 조용히 넘어가면 안 된다. 남은 카메라가 다음 주행에서 Nav2 를 무너뜨린다.
        log = ("⚠️ 로봇에 카메라가 그대로 남아있다 — 주행 전에 끌 것 "
               "(ssh rpi@호스트 \"pkill -f camera_node\")\n" + log)
    if not files:
        return [], f"프레임을 받지 못했다 — 카메라가 떠 있는지 확인. 로그: {log[-300:]}"
    return sorted(files), log


def local_grab(photo_dir, out_dir=None, n=3):
    """로봇 없이 시험할 때: 이미 있는 사진을 그대로 쓴다(복사하지 않는다).

    inspect_node 의 photo_dir 파라미터가 이걸 쓴다. 판정·기록·부저·CSV 흐름을
    로봇 없이 그대로 돌려볼 수 있다.
    """
    if not os.path.isdir(photo_dir):
        return [], f"photo_dir 이 없다: {photo_dir}"
    files = [os.path.join(photo_dir, f) for f in sorted(os.listdir(photo_dir))
             if f.lower().endswith((".jpg", ".jpeg", ".png"))]
    if not files:
        return [], f"photo_dir 에 사진이 없다: {photo_dir}"
    return files[:n], f"local {len(files[:n])} from {photo_dir}"
