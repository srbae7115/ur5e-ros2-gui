#!/usr/bin/env python3
"""UR5e MoveJ / MoveL GUI (ROS 2 Humble + ur_robot_driver)

- 상태 읽기: /joint_states, /tcp_pose_broadcaster/pose
- 이동 명령: URScript movej / movel 을 /urscript_interface/script_command 로 전송
- 주의: URScript 전송 시 드라이버의 External Control 프로그램이 중단됨.
        MoveIt/ROS 컨트롤러를 다시 쓰려면 [ROS 제어 복구] 버튼을 누를 것.

실행: source /opt/ros/humble/setup.bash && python3 ~/ur5e_move_gui.py
(ur_control.launch.py 드라이버가 먼저 실행 중이어야 함)
"""
import json
import math
import os
import queue
import socket
import subprocess
import threading
import time
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox, simpledialog

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
from ur_msgs.srv import SetSpeedSliderFraction

JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
          "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
JOINT_LABELS = ["Base", "Shoulder", "Elbow", "Wrist1", "Wrist2", "Wrist3"]
POSE_LABELS = ["X [mm]", "Y [mm]", "Z [mm]", "RX [rad]", "RY [rad]", "RZ [rad]"]

ROBOT_IP = "192.168.0.9"

# 드라이버 시작 시점에 기록한 원위치 [rad] (티칭 도구에서 'home'을 저장하면 그 값이 우선)
HOME_JOINTS = [-1.5623, -1.5682, -1.5762, -1.5672, 1.5724, -0.0027]


def home_joints():
    p = load_points().get("home")
    return p["joints"] if p else HOME_JOINTS


def rotvec_to_mat(r):
    r = np.asarray(r, dtype=float)
    th = np.linalg.norm(r)
    if th < 1e-12:
        return np.eye(3)
    k = r / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * K @ K


def rot_angle(Ra, Rb):
    """두 회전행렬 사이 각도 [rad]"""
    c = (np.trace(Ra.T @ Rb) - 1) / 2
    return math.acos(max(-1.0, min(1.0, c)))


def quat_to_rotvec(x, y, z, w):
    if w < 0:
        x, y, z, w = -x, -y, -z, -w
    w = min(1.0, max(-1.0, w))
    angle = 2.0 * math.acos(w)
    s = math.sqrt(max(0.0, 1.0 - w * w))
    if s < 1e-9:
        return [0.0, 0.0, 0.0]
    return [x / s * angle, y / s * angle, z / s * angle]


class URNode(Node):
    def __init__(self):
        super().__init__("ur5e_move_gui")
        self.joints = None      # rad, JOINTS 순서
        self.vel = [0.0] * 6    # rad/s
        self.fd_until = 0.0     # Freedrive 종료 예정 시각
        self.tcp = None         # [x,y,z(m), rx,ry,rz(rad)]
        self.prog_running = None
        self.create_subscription(JointState, "/joint_states", self._js_cb, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, "/tcp_pose_broadcaster/pose", self._tcp_cb, qos_profile_sensor_data)
        self.create_subscription(Bool, "/io_and_status_controller/robot_program_running", self._prog_cb,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.script_pub = self.create_publisher(String, "/urscript_interface/script_command", 10)
        self.speed_cli = self.create_client(SetSpeedSliderFraction, "/io_and_status_controller/set_speed_slider")
        self.resend_cli = self.create_client(Trigger, "/io_and_status_controller/resend_robot_program")

    def _js_cb(self, msg):
        d = dict(zip(msg.name, msg.position))
        if all(j in d for j in JOINTS):
            self.joints = [d[j] for j in JOINTS]
        if len(msg.velocity) == len(msg.name):
            self.vel = list(msg.velocity)

    def _tcp_cb(self, msg):
        p, q = msg.pose.position, msg.pose.orientation
        self.tcp = [p.x, p.y, p.z] + quat_to_rotvec(q.x, q.y, q.z, q.w)

    def _prog_cb(self, msg):
        self.prog_running = msg.data

    def send_script(self, body, fd_seconds=0):
        # 새 스크립트는 실행 중인 스크립트(Freedrive 포함)를 대체함
        self.fd_until = time.time() + fd_seconds if fd_seconds else 0.0
        script = "def gui_cmd():\n" + "".join(f"  {line}\n" for line in body) + "end\n"
        self.script_pub.publish(String(data=script))
        return script

    def set_speed(self, frac):
        if not self.speed_cli.service_is_ready():
            return False
        self.speed_cli.call_async(SetSpeedSliderFraction.Request(speed_slider_fraction=frac))
        return True

    def resend(self):
        if not self.resend_cli.service_is_ready():
            return False
        self.resend_cli.call_async(Trigger.Request())
        return True


class Gripper:
    """Robotiq 2F-85 - 로봇의 Robotiq URCap 소켓(63352) 사용. 팔 제어와 독립적."""
    OBJ_TXT = {0: "이동 중", 1: "물체 감지(열다가)", 2: "물체 감지(잡음)", 3: "목표 도달(물체 없음)"}

    def __init__(self, host, port=63352):
        self.host, self.port = host, port
        self.sock = None
        self.lock = threading.Lock()
        self.state = {}  # ACT, STA, POS, OBJ, FLT
        threading.Thread(target=self._poll, daemon=True).start()

    def _cmd(self, c):
        with self.lock:
            try:
                if self.sock is None:
                    self.sock = socket.create_connection((self.host, self.port), timeout=1.0)
                self.sock.sendall((c + "\n").encode())
                return self.sock.recv(1024).decode().strip()
            except OSError:
                if self.sock:
                    self.sock.close()
                self.sock = None
                return None

    def _get(self, var):
        r = self._cmd(f"GET {var}")
        try:
            return int(r.split()[1])
        except (AttributeError, IndexError, ValueError):
            return None

    def _poll(self):
        while True:
            st = {k: self._get(k) for k in ("ACT", "STA", "POS", "OBJ", "FLT")}
            st["connected"] = st["POS"] is not None
            self.state = st
            time.sleep(0.2)

    def activate(self):
        def run():
            self._cmd("SET ACT 0"); time.sleep(0.1)
            self._cmd("SET ACT 1")
            for _ in range(60):
                if self._get("STA") == 3:
                    break
                time.sleep(0.1)
            self._cmd("SET GTO 1")
        threading.Thread(target=run, daemon=True).start()

    def move(self, pos, speed, force):
        p, s, f = (int(max(0, min(255, x))) for x in (pos, speed, force))
        threading.Thread(target=self._cmd, args=(f"SET POS {p} SPE {s} FOR {f} GTO 1",), daemon=True).start()


class Dashboard:
    """UR Dashboard 서버(29999): 전원/브레이크/보호정지 해제 + 상태 폴링"""

    def __init__(self, host, port=29999):
        self.host, self.port = host, port
        self.sock = None
        self.lock = threading.Lock()
        self.mode = None      # POWER_OFF, IDLE, RUNNING ...
        self.safety = None    # NORMAL, PROTECTIVE_STOP, ...
        threading.Thread(target=self._poll, daemon=True).start()

    def cmd(self, c):
        with self.lock:
            try:
                if self.sock is None:
                    self.sock = socket.create_connection((self.host, self.port), timeout=2.0)
                    self.sock.recv(1024)  # 환영 메시지
                self.sock.sendall((c + "\n").encode())
                return self.sock.recv(1024).decode().strip()
            except OSError:
                if self.sock:
                    self.sock.close()
                self.sock = None
                return None

    def _poll(self):
        while True:
            r = self.cmd("robotmode")
            self.mode = r.split(":")[-1].strip() if r else None
            r = self.cmd("safetystatus")
            self.safety = r.split(":")[-1].strip() if r else None
            time.sleep(0.5)

    def wait_mode(self, target, timeout):
        t = time.time()
        while time.time() - t < timeout:
            if self.mode == target:
                return True
            time.sleep(0.2)
        return False


class ToolTip:
    """마우스를 올리면 0.45초 뒤 설명 말풍선 표시"""

    def __init__(self, w, text):
        self.w, self.text, self.tip, self.after_id = w, text, None, None
        w.bind("<Enter>", self._schedule, add="+")
        w.bind("<Leave>", self._hide, add="+")
        w.bind("<ButtonPress>", self._hide, add="+")

    def _schedule(self, _e):
        self._cancel()
        self.after_id = self.w.after(450, self._show)

    def _cancel(self):
        if self.after_id:
            self.w.after_cancel(self.after_id)
            self.after_id = None

    def _show(self):
        x = self.w.winfo_rootx() + 10
        y = self.w.winfo_rooty() + self.w.winfo_height() + 4
        self.tip = tk.Toplevel(self.w)
        self.tip.wm_overrideredirect(True)
        self.tip.geometry(f"+{x}+{y}")
        tk.Label(self.tip, text=self.text, justify="left", bg="#fffbe6", fg="#222", relief="solid", bd=1,
                 wraplength=420, padx=8, pady=5, font=("", 10)).pack()

    def _hide(self, _e=None):
        self._cancel()
        if self.tip:
            self.tip.destroy()
            self.tip = None


# 위젯 글자 → 말풍선 설명
TIPS = {
    "서보:": "로봇 팔 전원/브레이크 상태.\n초록 RUNNING = 움직일 수 있음 / 주황 IDLE = 전원만 켜짐(브레이크 잠김) / 회색 = 전원 꺼짐",
    "안전:": "안전 상태. NORMAL이 아니면(보호정지, 비상정지 등) 로봇이 움직이지 않습니다.",
    "서보 ON": "전원 ON → 브레이크 해제 → ROS 제어 프로그램 재전송까지 자동으로 진행.\n브레이크 풀릴 때 '딸깍' 소리와 함께 살짝 움직이는 건 정상입니다.",
    "서보 OFF": "로봇 전원을 끄고 브레이크를 잠급니다. (확인 창이 뜸)\n그리퍼는 전원이 따로라 잡고 있던 물체는 계속 잡고 있습니다.",
    "보호정지 해제": "충돌 감지 등으로 보호정지(PROTECTIVE_STOP)가 걸렸을 때 해제.\n비상정지 버튼으로 멈춘 경우는 버튼을 먼저 돌려서 풀고 [서보 ON]을 누르세요.",
    "현재값 불러오기": "로봇의 현재 값을 입력칸에 채웁니다. 여기서 조금만 고쳐서 실행하면 안전합니다.",
    "원위치 값": "원위치 자세를 입력칸에 채우기만 합니다. (움직이지 않음 → [MoveJ 실행]으로 이동)",
    "⌂ 원위치로 이동": "확인 창 후 원위치 자세로 바로 MoveJ 이동합니다.\n(속도는 MoveJ 칸의 v, a 값 사용)",
    "MoveJ 실행": "movej: 관절 공간 이동. 각 관절이 동시에 출발/도착하며 끝단 경로는 곡선입니다.\n가장 안정적인 큰 이동 방식. 충돌 검사는 하지 않습니다.",
    "MoveL 실행": "movel: 끝단(TCP)이 직선으로 이동합니다.\n자세가 특이점 근처면 실패하거나 관절이 빠르게 돌 수 있으니 짧은 거리부터 시험하세요.",
    "0으로": "입력칸을 모두 0으로. 상대 이동할 때 먼저 누르고 움직일 축만 입력하세요.",
    "절대": "입력한 값 자체가 목표입니다.",
    "상대(증분)": "현재 관절 각도에 입력값을 더한 곳으로 이동. 예) Base에 10 → 베이스만 10° 회전",
    "상대(base)": "현재 TCP에서 로봇 바닥(base) 좌표축 방향으로 입력값만큼 이동/회전.",
    "상대(tool)": "현재 TCP에서 툴 좌표축 방향으로 이동/회전. 예) Z에 10 → 툴이 바라보는 방향으로 10mm",
    "Base": "조그 기준을 로봇 바닥 좌표계로. 회전은 TCP 위치를 그대로 두고 base 축 기준으로 돕니다.",
    "Tool": "조그 기준을 툴 좌표계로. Z+는 그리퍼가 바라보는 방향입니다.",
    "이동 스텝 [mm]": "X/Y/Z 버튼 1회당 이동 거리 (최대 100mm). 처음엔 1~5mm 권장.",
    "회전 스텝 [°]": "Rx/Ry/Rz 버튼 1회당 회전 각도 (최대 45°).",
    "속도 [mm/s]": "조그 직선 이동 속도.",
    "회전속도 [°/s]": "조그 회전 속도.",
    "속도 v [°/s]": "MoveJ 관절 최대 속도.",
    "가속 a [°/s²]": "MoveJ 관절 가속도. 작을수록 부드럽게 출발/정지.",
    "속도 v [mm/s]": "MoveL 끝단 직선 속도.",
    "가속 a [mm/s²]": "MoveL 끝단 가속도.",
    "RX [rad]": "회전벡터(axis-angle) 표현, 단위 rad. 펜던트 '이동' 탭의 RX/RY/RZ와 같은 값입니다.",
    "RY [rad]": "회전벡터(axis-angle) 표현, 단위 rad.", "RZ [rad]": "회전벡터(axis-angle) 표현, 단위 rad.",
    "활성화": "그리퍼 초기화. 전원 켠 뒤 처음 한 번 필요합니다.\n한 번 완전히 닫혔다 열리므로 손가락 사이를 비워두세요.",
    "열기": "그리퍼 완전히 열기 (POS 0, 약 85mm).",
    "닫기": "그리퍼 완전히 닫기 (POS 255). 물체가 있으면 설정한 힘으로 잡고 멈춥니다.",
    "위치(0=열림~255=닫힘)": "슬라이더로 원하는 벌림 정도를 정하고 [이동]. 약 230이 완전히 닫힘입니다.",
    "이동": "슬라이더 위치로 그리퍼 이동.",
    "속도": "그리퍼 손가락 속도 0~255 (약 20~150 mm/s).",
    "잡는 힘": "파지력 0~255 (약 20~235 N). 다음 [닫기]/[이동]부터 적용됩니다.\n종이컵·과일 등은 0~30, 일반 물체 50~100, 무거운 금속은 150 이상.",
    "약하게": "힘 0 (≈20N, 최소)", "보통": "힘 100 (≈105N)", "세게": "힘 255 (≈235N, 최대) - 깨질 수 있는 물체 주의",
    "■ 정지": "소프트웨어 정지(stopj). 현재 동작을 감속 정지합니다.\n위급 시에는 반드시 펜던트의 비상정지 버튼을 누르세요!",
    "ROS 제어 복구 (MoveIt용)": "이 GUI로 movej/movel을 보내면 드라이버 제어 프로그램이 멈춥니다.\nMoveIt이나 ROS 토픽으로 다시 제어하려면 누르세요. (GUI만 쓸 땐 불필요)",
    "속도 슬라이더": "펜던트 속도 슬라이더와 같습니다. 모든 동작 속도에 곱해집니다. [적용]을 눌러야 반영.",
    "적용": "속도 슬라이더 값을 로봇에 적용.",
    "ⓘ 정보/펌웨어": "로봇 모델, 시리얼, PolyScope 버전, 그리퍼 펌웨어, ROS 드라이버 버전 확인.",
    "❓ 도움말": "전체 사용 설명서 (단축키 F1).",
    "▶ Pick & Place": "Pick & Place 실행 창 열기 (티칭한 home / pick / place 사용).",
    "▶ Pick & Place 실행": "그리퍼 열기 → home → pick 접근 → 하강 → 잡기(파지 확인) → 상승\n"
                         "→ place 접근 → 하강 → 놓기 → 상승 → home.  실행 전 확인 창이 뜹니다.",
    "반복 횟수": "사이클 반복 횟수. 왕복이 아니므로 2회 이상이면 매번 pick에 물체를 다시 놓아야 합니다.\n"
              "(물체가 없으면 '물체를 잡지 못함'으로 안전하게 멈춤)",
    "MoveJ 속도 [°/s]": "home ↔ 접근점 사이 큰 이동 속도.",
    "MoveL 속도 [mm/s]": "물체를 잡은 뒤/놓은 뒤 직선 상승 속도.",
    "하강 속도 [mm/s]": "접근점 → pick/place 직선 하강 속도. 정밀하게 놓으려면 낮게.",
    "그리퍼 힘 (0~255)": "파지력 (약 20~235N). 무른 물체는 낮게.",
    "그리퍼 속도 (0~255)": "손가락 닫힘/열림 속도.",
    "📍 티칭 도구":"Freedrive로 손으로 위치를 잡고 이름 붙여 저장 / 저장 위치로 이동.\n저장 파일: ~/ur5e_points.json",
    "✋ Freedrive ON": "로봇을 손으로 끌어서 움직일 수 있게 합니다. 120초 후 자동 해제.\n"
                      "양손으로 천천히 움직이세요. 펜던트 설치의 페이로드(2F-85 약 0.9kg)가\n틀리면 손을 놓을 때 로봇이 처지거나 떠오릅니다.",
    "Freedrive OFF": "Freedrive 해제. 로봇이 현재 위치에 고정됩니다.\n(이동 명령을 보내도 자동 해제됨)",
    "이름": "저장할 위치 이름. 같은 이름이면 덮어쓸지 묻습니다. Enter로도 저장.",
    "💾 저장": "현재 로봇 위치(관절값 + TCP 자세 + 그리퍼 위치)를 이 이름으로 저장.",
    "pick": "현재 위치를 'pick'(물건 잡는 위치)으로 저장. Pick&Place 프로그램이 사용합니다.",
    "place": "현재 위치를 'place'(물건 놓는 위치)로 저장. Pick&Place 프로그램이 사용합니다.",
    "home": "현재 위치를 'home'으로 저장. 메인 창 [원위치] 버튼도 이 값을 사용합니다.",
    "▶ 이 위치로 이동": "선택한 위치로 MoveJ 이동 (확인 창). 속도는 메인 창 MoveJ의 v, a.",
    "▲ 접근점으로 이동": "선택한 위치에서 툴 Z 반대 방향(그리퍼가 아래를 보면 위쪽)으로\n접근 높이만큼 떨어진 곳으로 이동. 물체 근처는 이걸로 먼저 가세요.",
    "접근 높이 [mm]": "접근점 거리 (0~300mm). Pick&Place의 접근/후퇴 높이와 같은 개념.",
    "MoveJ 칸에 불러오기": "선택한 위치의 관절값을 메인 창 MoveJ 칸에 채웁니다. (목록 더블클릭과 동일)",
    "이름 변경": "선택한 위치의 이름 변경.",
    "🗑 삭제": "선택한 위치 삭제 (확인 창).",
}
for _ax, _desc in [("X", "base/tool X축"), ("Y", "Y축"), ("Z", "Z축 (Base 기준 + = 위쪽)"),
                   ("Rx", "X축 기준 회전"), ("Ry", "Y축 기준 회전"), ("Rz", "Z축 기준 회전")]:
    TIPS[f"{_ax} +"] = f"{_desc} + 방향으로 1스텝 이동"
    TIPS[f"{_ax} −"] = f"{_desc} − 방향으로 1스텝 이동"

HELP_TEXT = """\
UR5e 제어 GUI 사용 설명서
════════════════════════════════════════

■ 시작 순서
  1) 드라이버 실행
     ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.0.9 headless_mode:=true
  2) 이 GUI 실행:  python3 ~/ur5e_move_gui.py
  3) 서보가 OFF면 [서보 ON]
  4) 그리퍼를 쓰려면 [활성화] 한 번

■ 현재 상태
  관절 각도(°)와 TCP 위치(mm) / 회전벡터(rad)를 0.1초마다 표시합니다.
  값은 펜던트 '이동' 탭에 나오는 것과 같은 base 좌표계 기준입니다.

■ 서보 (전원)
  [서보 ON]       전원 ON → 브레이크 해제 → ROS 제어 재연결 (자동)
  [서보 OFF]      전원 OFF, 브레이크 잠김
  [보호정지 해제]  충돌 등으로 멈췄을 때. 비상정지는 버튼부터 풀어야 함.

■ MoveJ (관절 이동)
  관절 6개 각도를 입력해서 이동. 큰 자세 변경에 적합.
  · 절대: 입력값이 목표 각도
  · 상대: 현재 각도 + 입력값
  [현재값 불러오기] 후 일부만 고쳐서 실행하는 습관을 들이세요.

■ MoveL (직선 이동)
  TCP가 직선으로 이동. 물체 접근/후퇴에 적합.
  · 절대: X,Y,Z(mm) / RX,RY,RZ(rad) 목표 자세
  · 상대(base): 바닥 좌표축 기준 증분
  · 상대(tool): 툴 좌표축 기준 증분  (예: Z=+30 → 그리퍼 앞쪽으로 30mm)

■ 조그 버튼
  버튼 1회 = 설정한 스텝만큼 직선 이동/회전.
  Base 기준: Z+ = 위.  Tool 기준: Z+ = 그리퍼 앞쪽.

■ 그리퍼 (Robotiq 2F-85)
  POS 0 = 완전히 열림(약 85mm), 약 230 = 완전히 닫힘.
  물체를 잡으면 상태에 '물체 감지(잡음)'이 표시됩니다.
  팔 제어와 독립적이라 언제든 사용 가능합니다.

■ 티칭 도구  ([📍 티칭 도구] 버튼)
  1) [✋ Freedrive ON] → 로봇을 손으로 끌어 대략 위치를 잡음 (120초 자동 해제)
  2) [Freedrive OFF] → 메인 창 조그로 mm 단위 미세 조정
  3) 이름 입력 후 [💾 저장], 또는 빠른 저장 [pick] [place] [home]
  4) 목록에서 선택 → [▲ 접근점으로 이동] / [▶ 이 위치로 이동] 으로 확인
  · 저장 파일: ~/ur5e_points.json  (관절값 rad, TCP m / rad, 그리퍼 POS)
  · 'home'을 저장하면 메인 창 [원위치]가 그 값을 사용합니다.
  · Pick&Place 티칭 팁: pick/place는 '그리퍼를 닫으면 바로 잡히는 높이'로.
    접근점(위쪽)은 프로그램이 자동 계산합니다.

■ 하단
  [■ 정지]          소프트웨어 감속 정지
  [ROS 제어 복구]    MoveIt / ROS 토픽 제어를 다시 쓰고 싶을 때
  속도 슬라이더       전체 속도 비율 (펜던트 슬라이더와 동일)

■ 안전 수칙
  · 처음 쓰는 동작은 속도 10~30%, 작은 스텝으로.
  · 이 GUI는 충돌 검사를 하지 않습니다. 경로를 눈으로 확인하세요.
  · [■ 정지]는 소프트웨어 정지입니다. 위급하면 펜던트 비상정지!

■ 기술 정보
  팔 이동  : URScript → /urscript_interface/script_command (ROS)
  상태     : /joint_states, /tcp_pose_broadcaster/pose (ROS)
  서보     : Dashboard 서버 TCP 29999
  그리퍼   : Robotiq URCap 소켓 TCP 63352

  버튼에 마우스를 올려두면 각 기능 설명이 나옵니다.
"""


def fmt(vals):
    return "[" + ", ".join(f"{v:.5f}" for v in vals) + "]"


class App:
    def __init__(self, root, node):
        self.root, self.node = root, node
        root.title("UR5e MoveJ / MoveL")
        root.geometry("+80+80")
        style = ttk.Style()
        style.configure("Go.TButton", font=("", 11, "bold"))
        style.configure("Stop.TButton", font=("", 12, "bold"), foreground="red")

        # ---------- 현재 상태 ----------
        st = ttk.LabelFrame(root, text="현재 상태", padding=8)
        st.grid(row=0, column=0, columnspan=2, sticky="ew", padx=8, pady=6)
        self.cur_j, self.cur_p = [], []
        for i in range(6):
            ttk.Label(st, text=JOINT_LABELS[i] + " [°]").grid(row=0, column=i, padx=6)
            v = tk.StringVar(value="-"); self.cur_j.append(v)
            ttk.Label(st, textvariable=v, width=10, anchor="center", relief="sunken").grid(row=1, column=i, padx=6)
            ttk.Label(st, text=POSE_LABELS[i]).grid(row=2, column=i, padx=6, pady=(6, 0))
            v = tk.StringVar(value="-"); self.cur_p.append(v)
            ttk.Label(st, textvariable=v, width=10, anchor="center", relief="sunken").grid(row=3, column=i, padx=6)
        self.status = tk.StringVar(value="연결 대기 중...")
        ttk.Label(st, textvariable=self.status, width=70, anchor="w").grid(row=4, column=0, columnspan=6, sticky="w", pady=(8, 0))

        pw = ttk.Frame(st); pw.grid(row=5, column=0, columnspan=6, sticky="w", pady=(8, 0))
        ttk.Label(pw, text="서보:").pack(side="left")
        self.servo_lbl = tk.Label(pw, text="-", width=18, font=("", 11, "bold"), relief="ridge")
        self.servo_lbl.pack(side="left", padx=4)
        ttk.Label(pw, text="안전:").pack(side="left", padx=(8, 0))
        self.safety_lbl = tk.Label(pw, text="-", width=18, font=("", 11, "bold"), relief="ridge")
        self.safety_lbl.pack(side="left", padx=4)
        ttk.Button(pw, text="서보 ON", style="Go.TButton", command=self.servo_on).pack(side="left", padx=(12, 2))
        ttk.Button(pw, text="서보 OFF", command=self.servo_off).pack(side="left", padx=2)
        ttk.Button(pw, text="보호정지 해제", command=self.unlock_pstop).pack(side="left", padx=2)

        # ---------- MoveJ ----------
        fj = ttk.LabelFrame(root, text="MoveJ  (관절 공간)", padding=8)
        fj.grid(row=1, column=0, sticky="nsew", padx=8, pady=6)
        self.j_ent = self._entries(fj, [l + " [°]" for l in JOINT_LABELS])
        self.j_mode = tk.StringVar(value="abs")
        mf = ttk.Frame(fj); mf.grid(row=6, column=0, columnspan=2, sticky="w", pady=4)
        ttk.Radiobutton(mf, text="절대", variable=self.j_mode, value="abs").pack(side="left")
        ttk.Radiobutton(mf, text="상대(증분)", variable=self.j_mode, value="rel").pack(side="left")
        self.j_v = self._param(fj, 7, "속도 v [°/s]", "30")
        self.j_a = self._param(fj, 8, "가속 a [°/s²]", "60")
        bf = ttk.Frame(fj); bf.grid(row=9, column=0, columnspan=2, pady=(6, 0), sticky="ew")
        ttk.Button(bf, text="현재값 불러오기", command=self.load_joints).pack(side="left", padx=2)
        ttk.Button(bf, text="원위치 값", command=self.load_home).pack(side="left", padx=2)
        ttk.Button(bf, text="⌂ 원위치로 이동", command=self.go_home).pack(side="left", padx=2)
        ttk.Button(bf, text="MoveJ 실행", style="Go.TButton", command=self.do_movej).pack(side="right", padx=2)

        # ---------- MoveL ----------
        fl = ttk.LabelFrame(root, text="MoveL  (직선, base 좌표계 / TCP)", padding=8)
        fl.grid(row=1, column=1, sticky="nsew", padx=8, pady=6)
        self.l_ent = self._entries(fl, POSE_LABELS)
        self.l_mode = tk.StringVar(value="abs")
        mf = ttk.Frame(fl); mf.grid(row=6, column=0, columnspan=2, sticky="w", pady=4)
        ttk.Radiobutton(mf, text="절대", variable=self.l_mode, value="abs").pack(side="left")
        ttk.Radiobutton(mf, text="상대(base)", variable=self.l_mode, value="rel_base").pack(side="left")
        ttk.Radiobutton(mf, text="상대(tool)", variable=self.l_mode, value="rel_tool").pack(side="left")
        self.l_v = self._param(fl, 7, "속도 v [mm/s]", "100")
        self.l_a = self._param(fl, 8, "가속 a [mm/s²]", "300")
        bf = ttk.Frame(fl); bf.grid(row=9, column=0, columnspan=2, pady=(6, 0), sticky="ew")
        ttk.Button(bf, text="현재값 불러오기", command=self.load_pose).pack(side="left", padx=2)
        ttk.Button(bf, text="0으로", command=lambda: self._fill(self.l_ent, [0] * 6, "{:.0f}")).pack(side="left", padx=2)
        ttk.Button(bf, text="MoveL 실행", style="Go.TButton", command=self.do_movel).pack(side="right", padx=2)

        # ---------- 조그 버튼 ----------
        fg = ttk.LabelFrame(root, text="조그  (버튼 1회 = 1스텝 직선 이동)", padding=8)
        fg.grid(row=2, column=0, columnspan=2, sticky="ew", padx=8, pady=6)
        opt = ttk.Frame(fg); opt.grid(row=0, column=0, columnspan=6, sticky="w", pady=(0, 6))
        ttk.Label(opt, text="기준:").pack(side="left")
        self.jog_frame = tk.StringVar(value="base")
        ttk.Radiobutton(opt, text="Base", variable=self.jog_frame, value="base").pack(side="left")
        ttk.Radiobutton(opt, text="Tool", variable=self.jog_frame, value="tool").pack(side="left")
        ttk.Label(opt, text="    이동 스텝 [mm]").pack(side="left")
        self.jog_mm = ttk.Combobox(opt, values=["0.5", "1", "5", "10", "20", "50"], width=5)
        self.jog_mm.set("5"); self.jog_mm.pack(side="left", padx=2)
        ttk.Label(opt, text="  회전 스텝 [°]").pack(side="left")
        self.jog_deg = ttk.Combobox(opt, values=["0.5", "1", "2", "5", "10", "15"], width=5)
        self.jog_deg.set("2"); self.jog_deg.pack(side="left", padx=2)
        ttk.Label(opt, text="  속도 [mm/s]").pack(side="left")
        self.jog_v = ttk.Entry(opt, width=6, justify="right"); self.jog_v.insert(0, "50"); self.jog_v.pack(side="left", padx=2)
        ttk.Label(opt, text="  회전속도 [°/s]").pack(side="left")
        self.jog_w = ttk.Entry(opt, width=6, justify="right"); self.jog_w.insert(0, "15"); self.jog_w.pack(side="left", padx=2)
        for col, name in enumerate(["X", "Y", "Z", "Rx", "Ry", "Rz"]):
            ttk.Button(fg, text=f"{name} +", width=8,
                       command=lambda i=col: self.do_jog(i, +1)).grid(row=1, column=col, padx=4, pady=2, ipady=4)
            ttk.Button(fg, text=f"{name} −", width=8,
                       command=lambda i=col: self.do_jog(i, -1)).grid(row=2, column=col, padx=4, pady=2, ipady=4)

        # ---------- 그리퍼 ----------
        fr = ttk.LabelFrame(root, text="그리퍼  (Robotiq 2F-85)", padding=8)
        fr.grid(row=3, column=0, columnspan=2, sticky="ew", padx=8, pady=6)
        ttk.Button(fr, text="활성화", command=self.grip_activate).grid(row=0, column=0, padx=3)
        ttk.Button(fr, text="열기", width=8, style="Go.TButton",
                   command=lambda: self.grip_move(0)).grid(row=0, column=1, padx=3, ipady=3)
        ttk.Button(fr, text="닫기", width=8, style="Go.TButton",
                   command=lambda: self.grip_move(255)).grid(row=0, column=2, padx=3, ipady=3)
        ttk.Label(fr, text="  위치(0=열림~255=닫힘)").grid(row=0, column=3)
        self.g_pos = tk.IntVar(value=0)
        ttk.Scale(fr, from_=0, to=255, variable=self.g_pos, length=160,
                  command=lambda v: self.g_pos_lbl.set(str(int(float(v))))).grid(row=0, column=4)
        self.g_pos_lbl = tk.StringVar(value="0")
        ttk.Label(fr, textvariable=self.g_pos_lbl, width=4).grid(row=0, column=5)
        ttk.Button(fr, text="이동", command=lambda: self.grip_move(self.g_pos.get())).grid(row=0, column=6, padx=3)
        sl = ttk.Frame(fr); sl.grid(row=1, column=0, columnspan=11, sticky="w", pady=(6, 0))
        ttk.Label(sl, text="속도").pack(side="left")
        self.g_spe = tk.IntVar(value=128)
        self.g_spe_lbl = tk.StringVar()
        ttk.Scale(sl, from_=0, to=255, variable=self.g_spe, length=160,
                  command=lambda _v: self._grip_lbls()).pack(side="left", padx=4)
        ttk.Label(sl, textvariable=self.g_spe_lbl, width=16).pack(side="left")
        ttk.Label(sl, text="    잡는 힘").pack(side="left")
        self.g_for = tk.IntVar(value=50)
        self.g_for_lbl = tk.StringVar()
        ttk.Scale(sl, from_=0, to=255, variable=self.g_for, length=220,
                  command=lambda _v: self._grip_lbls()).pack(side="left", padx=4)
        ttk.Label(sl, textvariable=self.g_for_lbl, width=18).pack(side="left")
        for name, v in (("약하게", 0), ("보통", 100), ("세게", 255)):
            ttk.Button(sl, text=name, width=6,
                       command=lambda v=v: (self.g_for.set(v), self._grip_lbls())).pack(side="left", padx=1)
        self._grip_lbls()
        self.g_status = tk.StringVar(value="-")
        ttk.Label(fr, textvariable=self.g_status, width=95, anchor="w").grid(row=2, column=0, columnspan=11, sticky="w", pady=(6, 0))

        # ---------- 동작 진행 상태 ----------
        fp = ttk.LabelFrame(root, text="동작 진행", padding=8)
        fp.grid(row=4, column=0, columnspan=2, sticky="ew", padx=8, pady=6)
        self.mv_state = tk.Label(fp, text="대기", width=16, font=("", 11, "bold"), fg="white", bg="#757575", relief="ridge")
        self.mv_state.pack(side="left")
        self.mv_prog = tk.DoubleVar(value=0)
        ttk.Progressbar(fp, variable=self.mv_prog, maximum=100, length=320).pack(side="left", padx=8)
        self.mv_pct = tk.StringVar(value="0%")
        ttk.Label(fp, textvariable=self.mv_pct, width=5, font=("", 11, "bold")).pack(side="left")
        self.mv_info = tk.StringVar(value="")
        # 글자 길이가 바뀌어도 창 크기가 변하지 않도록 폭 고정 (넘치면 잘림)
        ttk.Label(fp, textvariable=self.mv_info, width=70, anchor="w").pack(side="left", padx=8, fill="x", expand=True)
        self.motion = None

        # ---------- 하단 제어 ----------
        fc = ttk.Frame(root, padding=(8, 0)); fc.grid(row=5, column=0, columnspan=2, sticky="ew")
        ttk.Button(fc, text="■ 정지", style="Stop.TButton", command=self.do_stop).pack(side="left", ipadx=10, ipady=4)
        ttk.Button(fc, text="ROS 제어 복구 (MoveIt용)", command=self.do_resend).pack(side="left", padx=10)
        ttk.Label(fc, text="속도 슬라이더").pack(side="left", padx=(20, 4))
        self.speed = tk.IntVar(value=30)
        ttk.Scale(fc, from_=1, to=100, variable=self.speed, length=180,
                  command=lambda v: self.speed_lbl.set(f"{int(float(v))}%")).pack(side="left")
        self.speed_lbl = tk.StringVar(value="30%")
        ttk.Label(fc, textvariable=self.speed_lbl, width=5).pack(side="left")
        ttk.Button(fc, text="적용", command=self.do_speed).pack(side="left")
        ttk.Button(fc, text="❓ 도움말", command=self.show_help).pack(side="right", padx=2)
        ttk.Button(fc, text="ⓘ 정보/펌웨어", command=self.show_info).pack(side="right", padx=2)
        ttk.Button(fc, text="📍 티칭 도구", style="Go.TButton", command=self.open_teach).pack(side="right", padx=2)
        ttk.Button(fc, text="▶ Pick & Place", style="Go.TButton", command=self.open_pp).pack(side="right", padx=2)
        root.bind("<F1>", lambda _e: self.show_help())

        self.log = scrolledtext.ScrolledText(root, height=8, font=("Monospace", 9))
        self.log.grid(row=6, column=0, columnspan=2, sticky="nsew", padx=8, pady=8)
        root.columnconfigure(0, weight=1); root.columnconfigure(1, weight=1); root.rowconfigure(6, weight=1)

        self.log_q = queue.Queue()
        self.gripper = Gripper(ROBOT_IP)
        self.dash = Dashboard(ROBOT_IP)
        self._loaded = False
        self._attach_tips(root)
        self.refresh()

    def _attach_tips(self, widget):
        for ch in widget.winfo_children():
            if not isinstance(ch, (ttk.LabelFrame, tk.LabelFrame)):
                try:
                    t = str(ch.cget("text")).strip()
                except tk.TclError:
                    t = ""
                if t in TIPS:
                    ToolTip(ch, TIPS[t])
            self._attach_tips(ch)

    def open_pp(self):
        w = getattr(self, "_pp", None)
        if w and w.win.winfo_exists():
            w.win.lift(); return
        self._pp = PickPlaceWindow(self)

    def open_teach(self):
        tw = getattr(self, "_teach", None)
        if tw and tw.win.winfo_exists():
            tw.win.lift(); return
        self._teach = TeachWindow(self)

    # ---------- 도움말 / 정보 창 ----------
    def show_help(self):
        win = tk.Toplevel(self.root); win.title("도움말 - UR5e 제어 GUI"); win.geometry("720x640")
        txt = scrolledtext.ScrolledText(win, font=("Monospace", 10), wrap="word")
        txt.insert("1.0", HELP_TEXT); txt.config(state="disabled")
        txt.pack(fill="both", expand=True, padx=8, pady=8)
        ttk.Button(win, text="닫기", command=win.destroy).pack(pady=(0, 8))

    def _collect_info(self):
        def dq(c, strip_prefix=True):
            r = self.dash.cmd(c)
            if not r:
                return "-"
            return r.split(":", 1)[-1].strip() if strip_prefix and ":" in r and not r.startswith("URSoftware") else r

        def gq(var):
            r = self.gripper._cmd(f"GET {var}")
            return r.split(" ", 1)[1] if r and " " in r else "-"

        def pkg(name):
            try:
                return subprocess.run(["dpkg-query", "-W", "-f=${Version}", name],
                                      capture_output=True, text=True, timeout=3).stdout.split("-")[0] or "-"
            except (OSError, subprocess.SubprocessError):
                return "-"

        osname = "-"
        try:
            with open("/etc/os-release") as f:
                osname = next(l.split("=", 1)[1].strip().strip('"') for l in f if l.startswith("PRETTY_NAME"))
        except (OSError, StopIteration):
            pass
        model = dq("get robot model")
        return [
            ("로봇", "모델", f"{model}  (e-Series는 '{model}'로 보고됨 → {model}e)" if model in ("UR3", "UR5", "UR10") else model),
            ("로봇", "시리얼 번호", dq("get serial number")),
            ("로봇", "PolyScope (펌웨어)", dq("PolyscopeVersion")),
            ("로봇", "IP 주소", ROBOT_IP),
            ("로봇", "로봇 모드", dq("robotmode")),
            ("로봇", "안전 상태", dq("safetystatus")),
            ("로봇", "원격 제어 모드", dq("is in remote control")),
            ("로봇", "운전 모드", dq("get operational mode")),
            ("로봇", "프로그램 상태", dq("programState")),
            ("로봇", "로드된 프로그램", dq("get loaded program")),
            ("그리퍼", "모델", "Robotiq 2F-85"),
            ("그리퍼", "펌웨어", gq("FWV")),
            ("그리퍼", "드라이버 보드", gq("VER")),
            ("그리퍼", "시리얼 번호", gq("SNU")),
            ("PC", "OS", osname),
            ("PC", "ROS 2", os.environ.get("ROS_DISTRO", "-")),
            ("PC", "ur_robot_driver", pkg("ros-humble-ur-robot-driver")),
            ("PC", "ur_client_library", pkg("ros-humble-ur-client-library")),
        ]

    def show_info(self):
        win = tk.Toplevel(self.root); win.title("로봇 / 펌웨어 정보")
        tree = ttk.Treeview(win, columns=("sec", "k", "v"), show="headings", height=18)
        for col, name, w in (("sec", "구분", 70), ("k", "항목", 160), ("v", "값", 430)):
            tree.heading(col, text=name); tree.column(col, width=w, anchor="w")
        tree.pack(fill="both", expand=True, padx=8, pady=8)
        bf = ttk.Frame(win); bf.pack(fill="x", padx=8, pady=(0, 8))
        msg = tk.StringVar(value="불러오는 중...")
        ttk.Label(bf, textvariable=msg).pack(side="left")
        box = {"data": None}

        def load():
            msg.set("불러오는 중...")
            box["data"] = None
            threading.Thread(target=lambda: box.update(data=self._collect_info()), daemon=True).start()
            win.after(100, check)

        def check():
            if not win.winfo_exists():
                return
            if box["data"] is None:
                win.after(100, check); return
            tree.delete(*tree.get_children())
            for row in box["data"]:
                tree.insert("", "end", values=row)
            msg.set(time.strftime("갱신: %H:%M:%S"))

        def copy():
            if box["data"]:
                self.root.clipboard_clear()
                self.root.clipboard_append("\n".join(f"[{s}] {k}: {v}" for s, k, v in box["data"]))
                msg.set("클립보드에 복사됨")

        ttk.Button(bf, text="닫기", command=win.destroy).pack(side="right", padx=2)
        ttk.Button(bf, text="복사", command=copy).pack(side="right", padx=2)
        ttk.Button(bf, text="새로고침", command=load).pack(side="right", padx=2)
        load()

    # ---------- 위젯 헬퍼 ----------
    def _entries(self, parent, labels):
        ents = []
        for i, l in enumerate(labels):
            ttk.Label(parent, text=l, width=12).grid(row=i, column=0, sticky="w")
            e = ttk.Entry(parent, width=14, justify="right"); e.insert(0, "0")
            e.grid(row=i, column=1, pady=1, sticky="ew"); ents.append(e)
        return ents

    def _param(self, parent, row, label, default):
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w")
        e = ttk.Entry(parent, width=14, justify="right"); e.insert(0, default)
        e.grid(row=row, column=1, pady=1, sticky="ew")
        return e

    def _fill(self, ents, vals, f):
        for e, v in zip(ents, vals):
            e.delete(0, "end"); e.insert(0, f.format(v))

    def _read(self, ents):
        return [float(e.get()) for e in ents]

    def write_log(self, text):  # 어느 스레드에서 불러도 안전 (refresh에서 화면에 출력)
        self.log_q.put(text.rstrip() + "\n")

    # ---------- 주기 갱신 ----------
    def refresh(self):
        while not self.log_q.empty():
            self.log.insert("end", self.log_q.get()); self.log.see("end")
        n = self.node
        if n.joints:
            for v, q in zip(self.cur_j, n.joints):
                v.set(f"{math.degrees(q):.2f}")
        if n.tcp:
            for i, (v, q) in enumerate(zip(self.cur_p, n.tcp)):
                v.set(f"{q * 1000:.1f}" if i < 3 else f"{q:.4f}")
        if n.joints and n.tcp and not self._loaded:
            self.load_joints(); self.load_pose(); self._loaded = True
        self._motion_update()
        prog = {None: "알 수 없음", True: "실행 중", False: "정지"}[n.prog_running]
        self.status.set(f"상태 수신: {'OK' if n.joints else '없음'}   |   로봇 프로그램: {prog}")
        mode_txt = {"RUNNING": ("서보 ON (RUNNING)", "#2e7d32"), "IDLE": ("전원 ON, 브레이크 잠김", "#ef6c00"),
                    "POWER_ON": ("전원 켜는 중", "#ef6c00"), "BOOTING": ("부팅 중", "#ef6c00"),
                    "POWER_OFF": ("서보 OFF", "#757575"), "BACKDRIVE": ("백드라이브", "#ef6c00"),
                    None: ("연결 안 됨", "#c62828")}
        t, c = mode_txt.get(self.dash.mode, (str(self.dash.mode), "#c62828"))
        self.servo_lbl.config(text=t, fg="white", bg=c)
        sf = self.dash.safety
        self.safety_lbl.config(text=sf or "-", fg="white",
                               bg="#2e7d32" if sf == "NORMAL" else ("#757575" if sf is None else "#c62828"))
        g = self.gripper.state
        if not g.get("connected"):
            self.g_status.set("그리퍼: 연결 안 됨 (포트 63352)")
        elif g.get("STA") != 3:
            self.g_status.set(f"그리퍼: 활성화 필요 (STA={g.get('STA')})  ← [활성화] 누르면 한 번 완전히 열고 닫으며 초기화")
        else:
            pos = g.get("POS") or 0
            mm = max(0.0, 85.0 * (1 - pos / 230.0))
            flt = f"   ⚠ 오류 FLT={g.get('FLT')}" if g.get("FLT") else ""
            self.g_status.set(f"그리퍼: 준비됨  |  위치 {pos} (약 {mm:.0f}mm 벌어짐)  |  "
                              f"{Gripper.OBJ_TXT.get(g.get('OBJ'), '-')}{flt}")
        self.root.after(100, self.refresh)

    # ---------- 서보 ----------
    def _bg(self, fn):
        threading.Thread(target=fn, daemon=True).start()

    def servo_on(self):
        def run():
            self.write_log("[서보 ON] 전원 켜는 중...")
            self.dash.cmd("power on")
            if not self.dash.wait_mode("IDLE", 30):
                self.write_log(f"[서보 ON] 실패: 현재 모드 {self.dash.mode}"); return
            self.write_log("[서보 ON] 브레이크 해제 (딸깍 소리 + 살짝 움직임 정상)")
            self.dash.cmd("brake release")
            if not self.dash.wait_mode("RUNNING", 20):
                self.write_log(f"[서보 ON] 실패: 현재 모드 {self.dash.mode}"); return
            time.sleep(1.0)
            self.node.resend()
            self.write_log("[서보 ON] 완료 → ROS 제어 프로그램 재전송")
        self._bg(run)

    def servo_off(self):
        if not messagebox.askyesno("서보 OFF", "로봇 전원을 끄고 브레이크를 잠급니다.\n"
                                   "그리퍼에 물체를 잡고 있다면 그대로 유지됩니다.\n계속할까요?"):
            return
        self._bg(lambda: self.write_log(f"[서보 OFF] {self.dash.cmd('power off')}"))

    def unlock_pstop(self):
        def run():
            self.write_log(f"[보호정지 해제] {self.dash.cmd('close safety popup')} / "
                           f"{self.dash.cmd('unlock protective stop')}")
            time.sleep(1.0)
            self.node.resend()
        self._bg(run)

    def _grip_lbls(self):
        # 2F-85 사양: 속도 20~150 mm/s, 힘 20~235 N (0~255에 대략 선형 대응)
        s, f = int(self.g_spe.get()), int(self.g_for.get())
        self.g_spe.set(s); self.g_for.set(f)
        self.g_spe_lbl.set(f"{s}  (≈{20 + 130 * s / 255:.0f} mm/s)")
        self.g_for_lbl.set(f"{f}  (≈{20 + 215 * f / 255:.0f} N)")

    def grip_activate(self):
        self.gripper.activate()
        self.write_log("[그리퍼] 활성화 요청")

    def grip_move(self, pos):
        try:
            spe = int(self.g_spe.get()); frc = int(self.g_for.get())
        except ValueError:
            self.write_log("[오류] 그리퍼 속도/힘은 0~255 정수"); return
        if self.gripper.state.get("STA") != 3:
            self.write_log("[그리퍼] 먼저 [활성화]를 누르세요"); return
        self.gripper.move(pos, spe, frc)
        self.write_log(f"[그리퍼] POS={pos} SPE={spe} FOR={frc}")

    # ---------- 동작 ----------
    def load_joints(self):
        if self.node.joints:
            self._fill(self.j_ent, [math.degrees(q) for q in self.node.joints], "{:.3f}")
            self.j_mode.set("abs")

    def load_home(self):
        self._fill(self.j_ent, [math.degrees(q) for q in home_joints()], "{:.3f}")
        self.j_mode.set("abs")

    def load_pose(self):
        if self.node.tcp:
            t = self.node.tcp
            self._fill(self.l_ent[:3], [v * 1000 for v in t[:3]], "{:.2f}")
            self._fill(self.l_ent[3:], t[3:], "{:.5f}")
            self.l_mode.set("abs")

    # ---------- 진행률 추적 ----------
    def _track_j(self, target, label):
        if not self.node.joints:
            return
        tot = max(abs(c - t) for c, t in zip(self.node.joints, target))
        self._track_start(dict(kind="j", target=list(target), tot=tot, label=label))

    def _track_l(self, d, frame, label):
        """frame: None=절대(d가 목표 자세), 'base' / 'tool' = 상대"""
        tcp = self.node.tcp
        if not tcp:
            return
        pc, Rc = np.array(tcp[:3]), rotvec_to_mat(tcp[3:])
        dp, Rd = np.array(d[:3]), rotvec_to_mat(d[3:])
        if frame is None:
            pt, Rt = dp, Rd
        elif frame == "base":
            pt, Rt = pc + dp, Rd @ Rc
        else:
            pt, Rt = pc + Rc @ dp, Rc @ Rd
        self._track_start(dict(kind="l", pt=pt, Rt=Rt, tot_p=float(np.linalg.norm(pt - pc)),
                               tot_a=rot_angle(Rc, Rt), label=label))

    def _track_start(self, m):
        m.update(t0=time.time(), started=False, still=None)
        self.motion = m
        self._mv_show("이동 중", "#1565c0", 0, m["label"])

    def _mv_show(self, state, color, prog, info):
        self.mv_state.config(text=state, bg=color)
        self.mv_prog.set(prog * 100)
        self.mv_pct.set(f"{prog * 100:.0f}%")
        self.mv_info.set(info)

    def _motion_finish(self, state, color, prog, extra=""):
        m = self.motion
        self.motion = None
        dt = time.time() - m["t0"]
        self._mv_show(state, color, prog, f"{m['label']}  ({dt:.1f}초){extra}")
        self.write_log(f"[{state}] {m['label']}  {dt:.1f}초{extra}")

    def _motion_update(self):
        m, n = self.motion, self.node
        if not m or not n.joints or not n.tcp:
            return
        if m["kind"] == "j":
            rem = max(abs(c - t) for c, t in zip(n.joints, m["target"]))
            prog = 1 - rem / m["tot"] if m["tot"] > 1e-4 else 1.0
            near = rem < math.radians(0.3)
            rem_txt = f"남은 각도 {math.degrees(rem):.1f}°"
        else:
            pc, Rc = np.array(n.tcp[:3]), rotvec_to_mat(n.tcp[3:])
            pr, ar = float(np.linalg.norm(m["pt"] - pc)), rot_angle(Rc, m["Rt"])
            fr = []
            if m["tot_p"] > 1e-4: fr.append(pr / m["tot_p"])
            if m["tot_a"] > 1e-3: fr.append(ar / m["tot_a"])
            prog = 1 - max(fr) if fr else 1.0
            near = pr < 0.001 and ar < math.radians(0.3)
            rem_txt = f"남은 거리 {pr * 1000:.1f}mm / {math.degrees(ar):.1f}°"
        prog = max(0.0, min(1.0, prog))
        moving = max(abs(v) for v in n.vel) > 0.005
        now = time.time()
        if moving:
            m["started"], m["still"] = True, None
        elif m["still"] is None:
            m["still"] = now
        sf = self.dash.safety
        if sf not in (None, "NORMAL"):
            self._motion_finish("✖ 안전정지", "#c62828", prog, f"  - {sf}")
        elif near and not moving:
            self._motion_finish("✔ 이동 완료", "#2e7d32", 1.0)
        elif m["still"] and now - m["still"] > (1.0 if m["started"] else 3.0):
            self._motion_finish("✖ 중단됨", "#c62828", prog,
                                f"  - 목표 미도달, {rem_txt}" if m["started"] else "  - 로봇이 움직이지 않음 (서보/프로그램 확인)")
        else:
            self._mv_show("이동 중", "#1565c0", prog, f"{m['label']}  |  {rem_txt}  |  {now - m['t0']:.1f}초")

    def go_home(self):
        deg = ", ".join(f"{math.degrees(q):.1f}" for q in home_joints())
        if not messagebox.askyesno("원위치로 이동", f"원위치 [{deg}]° 로 이동합니다.\n"
                                   "경로에 사람/장애물이 없는지 확인하세요.\n계속할까요?"):
            return
        self.load_home()
        self.do_movej()

    def do_movej(self):
        try:
            vals = [math.radians(v) for v in self._read(self.j_ent)]
            v = math.radians(float(self.j_v.get())); a = math.radians(float(self.j_a.get()))
        except ValueError:
            self.write_log("[오류] 숫자를 확인하세요"); return
        if self.j_mode.get() == "rel":
            if not self.node.joints:
                self.write_log("[오류] 현재 관절값 없음"); return
            vals = [c + d for c, d in zip(self.node.joints, vals)]
        if max(abs(x) for x in vals) > 2 * math.pi:
            self.write_log("[오류] 관절 범위(±360°) 초과"); return
        s = self.node.send_script([f"movej({fmt(vals)}, a={a:.4f}, v={v:.4f})"])
        self.write_log("[MoveJ]\n" + s)
        self._track_j(vals, "MoveJ")

    def do_movel(self):
        try:
            vals = self._read(self.l_ent)
            v = float(self.l_v.get()) / 1000; a = float(self.l_a.get()) / 1000
        except ValueError:
            self.write_log("[오류] 숫자를 확인하세요"); return
        d = [x / 1000 for x in vals[:3]] + vals[3:]
        mode = self.l_mode.get()
        if mode == "abs":
            lines = [f"movel(p{fmt(d)}, a={a:.4f}, v={v:.4f})"]
        else:
            lines = self._rel_movel(d, "base" if mode == "rel_base" else "tool", a, v)
        s = self.node.send_script(lines)
        self.write_log("[MoveL]\n" + s)
        self._track_l(d, {"abs": None, "rel_base": "base", "rel_tool": "tool"}[mode], f"MoveL ({mode})")

    @staticmethod
    def _rel_movel(d, frame, a, v):
        """현재 TCP 기준 상대 movel. d = [dx,dy,dz(m), rx,ry,rz(rad 회전벡터)]
        base: 위치는 base 축으로 더하고, 회전은 TCP 위치 그대로 base 축 기준 회전
        tool: 툴 좌표계 기준 (pose_trans)"""
        p = "p" + fmt(d)
        if frame == "tool":
            return [f"movel(pose_trans(get_actual_tcp_pose(), {p}), a={a:.4f}, v={v:.4f})"]
        return ["c = get_actual_tcp_pose()",
                f"d = {p}",
                "r = pose_trans(p[0, 0, 0, d[3], d[4], d[5]], p[0, 0, 0, c[3], c[4], c[5]])",
                f"movel(p[c[0] + d[0], c[1] + d[1], c[2] + d[2], r[3], r[4], r[5]], a={a:.4f}, v={v:.4f})"]

    def do_jog(self, idx, sign):
        try:
            mm = float(self.jog_mm.get()); deg = float(self.jog_deg.get())
            v = float(self.jog_v.get()) / 1000; w = math.radians(float(self.jog_w.get()))
        except ValueError:
            self.write_log("[오류] 조그 값 확인"); return
        if not (0 < mm <= 100 and 0 < deg <= 45):
            self.write_log("[오류] 스텝 범위: 이동 0~100mm, 회전 0~45°"); return
        d = [0.0] * 6
        if idx < 3:
            d[idx] = sign * mm / 1000; spd = v; step = f"{sign * mm:+g}mm"
        else:
            d[idx] = sign * math.radians(deg); spd = w; step = f"{sign * deg:+g}°"
        frame = self.jog_frame.get()
        self.node.send_script(self._rel_movel(d, frame, spd * 3, spd))
        self._track_l(d, frame, f"조그 {['X', 'Y', 'Z', 'Rx', 'Ry', 'Rz'][idx]} {step} ({frame})")
        self.write_log(f"[조그] {['X', 'Y', 'Z', 'Rx', 'Ry', 'Rz'][idx]} {step} ({frame})")

    def do_stop(self):
        self.node.send_script(["stopj(3.0)"])
        self.write_log("[정지] stopj(3.0)")
        if self.motion:
            self._motion_finish("■ 정지됨", "#ef6c00", self.mv_prog.get() / 100)

    def do_resend(self):
        ok = self.node.resend()
        self.write_log("[ROS 제어 복구] 요청 보냄" if ok else "[오류] resend 서비스 없음 (드라이버 확인)")

    def do_speed(self):
        f = self.speed.get() / 100.0
        ok = self.node.set_speed(f)
        self.write_log(f"[속도] {int(f * 100)}%" if ok else "[오류] 속도 서비스 없음")


POINTS_FILE = os.path.expanduser("~/ur5e_points.json")


def load_points():
    try:
        with open(POINTS_FILE) as f:
            return json.load(f).get("points", {})
    except (OSError, ValueError):
        return {}


def save_points(pts):
    tmp = POINTS_FILE + ".tmp"
    with open(tmp, "w") as f:
        json.dump({"version": 1, "robot": "UR5e", "points": pts}, f, indent=2, ensure_ascii=False)
    os.replace(tmp, POINTS_FILE)


class TeachWindow:
    """티칭 도구: Freedrive로 손으로 위치 잡기 → 이름 붙여 저장 → 저장 위치로 이동"""
    FD_TIME = 120  # Freedrive 자동 해제 시간 [s]

    def __init__(self, app):
        self.app = app
        self.pts = load_points()
        self.win = win = tk.Toplevel(app.root)
        win.title("티칭 도구 - UR5e")
        win.geometry("+140+140")

        # 1. Freedrive
        ff = ttk.LabelFrame(win, text="1. 손으로 끌어서 위치 잡기 (Freedrive)", padding=8)
        ff.pack(fill="x", padx=8, pady=6)
        ttk.Button(ff, text="✋ Freedrive ON", style="Go.TButton", command=self.fd_on).pack(side="left", padx=2)
        ttk.Button(ff, text="Freedrive OFF", command=self.fd_off).pack(side="left", padx=2)
        self.fd_lbl = tk.Label(ff, text="OFF", width=26, font=("", 11, "bold"), fg="white", bg="#757575", relief="ridge")
        self.fd_lbl.pack(side="left", padx=10)
        ttk.Label(ff, text="미세 조정은 메인 창의 조그 버튼을 쓰세요", foreground="#666").pack(side="left", padx=6)

        # 2. 저장
        fs = ttk.LabelFrame(win, text="2. 현재 위치 저장", padding=8)
        fs.pack(fill="x", padx=8, pady=6)
        ttk.Label(fs, text="이름").pack(side="left")
        self.name = ttk.Entry(fs, width=16); self.name.pack(side="left", padx=4)
        self.name.bind("<Return>", lambda _e: self.save(self.name.get()))
        ttk.Button(fs, text="💾 저장", style="Go.TButton", command=lambda: self.save(self.name.get())).pack(side="left", padx=2)
        ttk.Label(fs, text="    빠른 저장:").pack(side="left")
        for nm in ("pick", "place", "home"):
            ttk.Button(fs, text=nm, width=7, command=lambda nm=nm: self.save(nm)).pack(side="left", padx=1)
        self.cur_lbl = tk.StringVar()
        ttk.Label(fs, textvariable=self.cur_lbl, width=52, anchor="w", foreground="#444").pack(side="left", padx=10)

        # 3. 목록
        fl = ttk.LabelFrame(win, text=f"3. 저장된 위치   ({POINTS_FILE})", padding=8)
        fl.pack(fill="both", expand=True, padx=8, pady=6)
        cols = (("name", "이름", 90), ("joints", "관절 [°]", 340), ("tcp", "TCP X, Y, Z [mm]", 190),
                ("grip", "그리퍼", 60), ("time", "저장 시각", 150))
        self.tree = ttk.Treeview(fl, columns=[c[0] for c in cols], show="headings", height=8, selectmode="browse")
        for c, t, w in cols:
            self.tree.heading(c, text=t); self.tree.column(c, width=w, anchor="w")
        self.tree.pack(fill="both", expand=True)
        self.tree.bind("<Double-1>", lambda _e: self.to_fields())
        bf = ttk.Frame(fl); bf.pack(fill="x", pady=(6, 0))
        ttk.Button(bf, text="▶ 이 위치로 이동", style="Go.TButton", command=self.goto).pack(side="left", padx=2)
        ttk.Button(bf, text="▲ 접근점으로 이동", command=self.goto_approach).pack(side="left", padx=2)
        ttk.Label(bf, text="접근 높이 [mm]").pack(side="left", padx=(8, 2))
        self.appr = ttk.Entry(bf, width=6, justify="right"); self.appr.insert(0, "100"); self.appr.pack(side="left")
        ttk.Button(bf, text="MoveJ 칸에 불러오기", command=self.to_fields).pack(side="left", padx=(12, 2))
        ttk.Button(bf, text="🗑 삭제", command=self.delete).pack(side="right", padx=2)
        ttk.Button(bf, text="이름 변경", command=self.rename).pack(side="right", padx=2)

        self.refresh_list()
        app._attach_tips(win)
        self.tick()

    # ----- 목록 -----
    def refresh_list(self, select=None):
        self.tree.delete(*self.tree.get_children())
        for nm, p in self.pts.items():
            j = ", ".join(f"{math.degrees(q):.1f}" for q in p["joints"])
            t = ", ".join(f"{v * 1000:.1f}" for v in p["tcp"][:3])
            self.tree.insert("", "end", iid=nm, values=(nm, j, t, p.get("gripper", "-"), p.get("time", "")))
        if select and select in self.pts:
            self.tree.selection_set(select); self.tree.see(select)

    def selected(self):
        sel = self.tree.selection()
        if not sel:
            messagebox.showinfo("선택", "목록에서 위치를 먼저 선택하세요.", parent=self.win)
            return None, None
        return sel[0], self.pts[sel[0]]

    def save(self, name):
        name = name.strip()
        n = self.app.node
        if not name:
            messagebox.showinfo("이름", "저장할 이름을 입력하세요.", parent=self.win); return
        if not (n.joints and n.tcp):
            messagebox.showerror("오류", "로봇 상태를 받지 못했습니다. 드라이버를 확인하세요.", parent=self.win); return
        if name in self.pts and not messagebox.askyesno(
                "덮어쓰기", f"'{name}' 이(가) 이미 있습니다.\n현재 위치로 덮어쓸까요?", parent=self.win):
            return
        self.pts[name] = {"joints": [round(q, 6) for q in n.joints], "tcp": [round(v, 6) for v in n.tcp],
                          "gripper": self.app.gripper.state.get("POS"), "time": time.strftime("%Y-%m-%d %H:%M:%S")}
        save_points(self.pts)
        self.refresh_list(select=name)
        self.app.write_log(f"[티칭] '{name}' 저장  관절={', '.join(f'{math.degrees(q):.1f}' for q in n.joints)}°")

    def delete(self):
        nm, _ = self.selected()
        if nm and messagebox.askyesno("삭제", f"'{nm}' 위치를 삭제할까요?", parent=self.win):
            del self.pts[nm]; save_points(self.pts); self.refresh_list()
            self.app.write_log(f"[티칭] '{nm}' 삭제")

    def rename(self):
        nm, _ = self.selected()
        if not nm:
            return
        new = simpledialog.askstring("이름 변경", f"'{nm}'의 새 이름:", initialvalue=nm, parent=self.win)
        if not new or new.strip() == nm:
            return
        new = new.strip()
        if new in self.pts:
            messagebox.showerror("오류", f"'{new}' 이(가) 이미 있습니다.", parent=self.win); return
        self.pts = {(new if k == nm else k): v for k, v in self.pts.items()}
        save_points(self.pts); self.refresh_list(select=new)
        self.app.write_log(f"[티칭] '{nm}' → '{new}' 이름 변경")

    # ----- 이동 -----
    def to_fields(self):
        nm, p = self.selected()
        if nm:
            self.app._fill(self.app.j_ent, [math.degrees(q) for q in p["joints"]], "{:.3f}")
            self.app.j_mode.set("abs")
            self.app.write_log(f"[티칭] '{nm}' → MoveJ 칸에 불러옴")

    def goto(self):
        nm, p = self.selected()
        if nm and messagebox.askyesno("이동", f"'{nm}' 위치로 MoveJ 이동합니다.\n경로에 장애물이 없는지 확인하세요.",
                                      parent=self.win):
            self.to_fields()
            self.app.do_movej()

    def goto_approach(self):
        nm, p = self.selected()
        if not nm:
            return
        try:
            off = float(self.appr.get()) / 1000
            v = math.radians(float(self.app.j_v.get())); a = math.radians(float(self.app.j_a.get()))
        except ValueError:
            messagebox.showerror("오류", "접근 높이/속도 값을 확인하세요.", parent=self.win); return
        if not 0 <= off <= 0.3:
            messagebox.showerror("오류", "접근 높이는 0~300mm", parent=self.win); return
        if not messagebox.askyesno("이동", f"'{nm}' 의 접근점(툴 기준 {off * 1000:.0f}mm 뒤/위)으로 이동합니다.",
                                   parent=self.win):
            return
        # 로봇이 IK를 풀되, 저장된 관절값 근처의 해를 선택 → 자세가 뒤집히지 않음
        self.app.node.send_script([
            f"target = pose_trans(p{fmt(p['tcp'])}, p[0, 0, {-off:.4f}, 0, 0, 0])",
            f"movej(get_inverse_kin(target, qnear={fmt(p['joints'])}), a={a:.4f}, v={v:.4f})"])
        R = rotvec_to_mat(p["tcp"][3:])
        pt = np.array(p["tcp"][:3]) + R @ np.array([0, 0, -off])
        tcp = self.app.node.tcp
        if tcp:
            self.app._track_start(dict(kind="l", pt=pt, Rt=R, tot_p=float(np.linalg.norm(pt - np.array(tcp[:3]))),
                                       tot_a=rot_angle(rotvec_to_mat(tcp[3:]), R), label=f"'{nm}' 접근점"))
        self.app.write_log(f"[티칭] '{nm}' 접근점({off * 1000:.0f}mm)으로 이동")

    # ----- Freedrive -----
    def fd_on(self):
        if self.app.dash.mode != "RUNNING":
            messagebox.showerror("Freedrive", "서보가 ON(RUNNING) 상태여야 합니다.", parent=self.win); return
        n = self.FD_TIME * 10
        self.app.node.send_script(["freedrive_mode()", "t = 0", f"while t < {n}:", "  sleep(0.1)", "  t = t + 1",
                                   "end", "end_freedrive_mode()"], fd_seconds=self.FD_TIME)
        self.app.write_log(f"[Freedrive] ON (최대 {self.FD_TIME}초)")

    def fd_off(self):
        self.app.node.send_script(["end_freedrive_mode()"])
        self.app.write_log("[Freedrive] OFF")

    def tick(self):
        if not self.win.winfo_exists():
            return
        n = self.app.node
        rem = n.fd_until - time.time()
        if rem > 0:
            self.fd_lbl.config(text=f"✋ Freedrive ON  (남은 {rem:.0f}초)", bg="#ef6c00")
        else:
            self.fd_lbl.config(text="OFF", bg="#757575")
        if n.joints and n.tcp:
            self.cur_lbl.set("현재: " + ", ".join(f"{math.degrees(q):.1f}" for q in n.joints) + "°   |   "
                             + ", ".join(f"{v * 1000:.0f}" for v in n.tcp[:3]) + " mm")
        self.win.after(200, self.tick)


class PickPlaceWindow:
    """Pick & Place 실행 창 - 로직은 ur5e_pick_place.py 의 PickPlace 사용"""
    C_IDLE, C_RUN, C_OK, C_FAIL = "#9e9e9e", "#1565c0", "#2e7d32", "#c62828"

    def __init__(self, app):
        import ur5e_pick_place as ppm   # 순환 import 방지를 위해 여기서 import
        self.app, self.ppm = app, ppm
        self.pp, self.thread, self.cur = None, None, None
        self.q = queue.Queue()
        self.win = win = tk.Toplevel(app.root)
        win.title("Pick & Place - UR5e")
        win.geometry("+160+90")
        win.protocol("WM_DELETE_WINDOW", self.on_close)

        # 설정
        fs = ttk.LabelFrame(win, text="설정", padding=8)
        fs.pack(fill="x", padx=8, pady=6)
        self.ent = {}
        fields = [("cycles", "반복 횟수", 1), ("approach", "접근 높이 [mm]", 100), ("jvel", "MoveJ 속도 [°/s]", 45),
                  ("lvel", "MoveL 속도 [mm/s]", 100), ("slow", "하강 속도 [mm/s]", 40),
                  ("force", "그리퍼 힘 (0~255)", 50), ("gspeed", "그리퍼 속도 (0~255)", 128)]
        for i, (k, label, dv) in enumerate(fields):
            r, c = divmod(i, 4)
            ttk.Label(fs, text=label).grid(row=r, column=c * 2, sticky="w", padx=(12 if c else 0, 3), pady=2)
            e = ttk.Entry(fs, width=7, justify="right"); e.insert(0, str(dv))
            e.grid(row=r, column=c * 2 + 1, pady=2)
            self.ent[k] = e
        self.pts_lbl = tk.StringVar()
        ttk.Label(fs, textvariable=self.pts_lbl, foreground="#444", width=110, anchor="w").grid(
            row=2, column=0, columnspan=8, sticky="w", pady=(6, 0))

        # 실행 버튼
        fb = ttk.Frame(win, padding=(8, 2)); fb.pack(fill="x")
        self.start_btn = ttk.Button(fb, text="▶ Pick & Place 실행", style="Go.TButton", command=self.start)
        self.start_btn.pack(side="left", ipady=5, ipadx=6)
        ttk.Button(fb, text="■ 정지", style="Stop.TButton", command=self.stop).pack(side="left", padx=8, ipady=5, ipadx=6)
        self.state_lbl = tk.Label(fb, text="대기", width=34, font=("", 11, "bold"), fg="white", bg="#757575",
                                  relief="ridge", anchor="w", padx=6)
        self.state_lbl.pack(side="left", padx=6)
        self.cyc_lbl = tk.StringVar(value="")
        ttk.Label(fb, textvariable=self.cyc_lbl, font=("", 11, "bold"), width=16).pack(side="left")

        # 단계 표시
        fst = ttk.LabelFrame(win, text="단계", padding=8)
        fst.pack(fill="x", padx=8, pady=6)
        self.step_lbls = []
        for i, s in enumerate(ppm.STEPS):
            lb = tk.Label(fst, anchor="w", width=36, font=("", 10))
            lb.grid(row=i % 6, column=i // 6, sticky="w", padx=4)
            self.step_lbls.append(lb)
        self.reset_steps()
        self.prog = tk.DoubleVar(value=0)
        pf = ttk.Frame(win, padding=(8, 0)); pf.pack(fill="x")
        ttk.Progressbar(pf, variable=self.prog, maximum=100).pack(side="left", fill="x", expand=True)
        self.pct = tk.StringVar(value="0%")
        ttk.Label(pf, textvariable=self.pct, width=5, font=("", 11, "bold")).pack(side="left", padx=6)

        self.log = scrolledtext.ScrolledText(win, height=11, font=("Monospace", 9))
        self.log.pack(fill="both", expand=True, padx=8, pady=8)

        app._attach_tips(win)
        self.update_pts()
        self.poll()

    # ----- 표시 -----
    def set_step(self, i, mark, color, bold=False):
        self.step_lbls[i].config(text=f" {mark}  {i + 1:2d}. {self.ppm.STEPS[i]}", fg=color,
                                 font=("", 10, "bold" if bold else "normal"))

    def reset_steps(self):
        for i in range(len(self.ppm.STEPS)):
            self.set_step(i, "○", self.C_IDLE)

    def set_state(self, text, bg):
        self.state_lbl.config(text=text, bg=bg)

    def update_pts(self):
        pts = load_points()
        miss = [k for k in ("home", "pick", "place") if k not in pts]
        if miss:
            self.pts_lbl.set(f"⚠ 티칭 위치 없음: {', '.join(miss)}  → [📍 티칭 도구]에서 저장하세요")
        else:
            f = lambda k: ", ".join(f"{v * 1000:.0f}" for v in pts[k]["tcp"][:3])
            self.pts_lbl.set(f"pick [{f('pick')}] mm  →  place [{f('place')}] mm   (저장: {pts['place'].get('time', '')})")
        return pts, miss

    # ----- 실행 -----
    def start(self):
        if self.thread and self.thread.is_alive():
            return
        pts, miss = self.update_pts()
        if miss:
            messagebox.showerror("Pick & Place", f"티칭 위치가 없습니다: {', '.join(miss)}", parent=self.win); return
        try:
            v = {k: float(e.get()) for k, e in self.ent.items()}
        except ValueError:
            messagebox.showerror("Pick & Place", "설정 값은 숫자로 입력하세요.", parent=self.win); return
        if not (1 <= v["cycles"] <= 100 and 0 <= v["approach"] <= 300 and 0 <= v["force"] <= 255
                and 0 <= v["gspeed"] <= 255 and v["jvel"] > 0 and v["lvel"] > 0 and v["slow"] > 0):
            messagebox.showerror("Pick & Place", "설정 범위를 확인하세요.\n(반복 1~100, 접근 0~300mm, 힘/속도 0~255)",
                                 parent=self.win); return
        if not messagebox.askyesno("Pick & Place", f"Pick & Place를 {int(v['cycles'])}회 실행합니다.\n"
                                   "로봇 주변에 사람/장애물이 없는지 확인하세요.\n계속할까요?", parent=self.win):
            return
        args = self.ppm.make_args(cycles=int(v["cycles"]), approach=v["approach"], jvel=v["jvel"], lvel=v["lvel"],
                                  slow=v["slow"], force=v["force"], gspeed=v["gspeed"])
        self.cycles = args.cycles
        self.pp = self.ppm.PickPlace(self.app.node, args, grip=self.app.gripper, dash=self.app.dash,
                                     logger=lambda m: self.q.put(("log", m)),
                                     on_step=lambda c, i, t: self.q.put(("step", c, i, t)))
        self.reset_steps(); self.cur = None
        self.prog.set(0); self.pct.set("0%"); self.cyc_lbl.set("")
        self.set_state("▶ 실행 중", self.C_RUN)
        self.start_btn.state(["disabled"])
        self.app.write_log(f"[Pick&Place] 시작 ({args.cycles}회)")

        def run():
            try:
                ok, msg = self.pp.run(pts)
            except Exception as e:  # noqa: BLE001 - 예기치 못한 오류도 GUI에 표시
                self.pp.stop(); ok, msg = False, f"오류: {e}"
            self.q.put(("done", ok, msg))
        self.thread = threading.Thread(target=run, daemon=True)
        self.thread.start()

    def stop(self):
        self.app.node.send_script(["stopj(2.0)"])   # 즉시 정지 명령
        if self.pp:
            self.pp.stop_req.set()                  # 시퀀스 중단
        self.app.write_log("[Pick&Place] 정지 요청")

    def on_close(self):
        if self.thread and self.thread.is_alive():
            if not messagebox.askyesno("닫기", "실행 중입니다. 정지하고 닫을까요?", parent=self.win):
                return
            self.stop()
        self.win.destroy()

    def poll(self):
        if not self.win.winfo_exists():
            return
        while not self.q.empty():
            ev = self.q.get()
            if ev[0] == "log":
                self.log.insert("end", ev[1] + "\n"); self.log.see("end")
            elif ev[0] == "step":
                _, c, i, t = ev
                if i == 0:
                    self.reset_steps()
                elif self.cur is not None:
                    self.set_step(self.cur, "✔", self.C_OK)
                self.cur = i
                self.set_step(i, "▶", self.C_RUN, bold=True)
                p = ((c - 1) * t + i) / (self.cycles * t)
                self.prog.set(p * 100); self.pct.set(f"{p * 100:.0f}%")
                self.cyc_lbl.set(f"사이클 {c} / {self.cycles}")
            elif ev[0] == "done":
                _, ok, msg = ev
                self.start_btn.state(["!disabled"])
                if ok:
                    for i in range(len(self.ppm.STEPS)):
                        self.set_step(i, "✔", self.C_OK)
                    self.prog.set(100); self.pct.set("100%")
                    self.set_state("✔ 완료", self.C_OK)
                else:
                    if self.cur is not None:
                        self.set_step(self.cur, "✖", self.C_FAIL, bold=True)
                    self.set_state(f"✖ 중단: {msg}", self.C_FAIL)
                self.app.write_log(f"[Pick&Place] {'완료' if ok else '중단: ' + msg}")
        self.win.after(100, self.poll)


def main():
    rclpy.init()
    node = URNode()
    spin_th = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_th.start()
    root = tk.Tk()
    App(root, node)
    try:
        root.mainloop()
    finally:
        rclpy.try_shutdown()       # spin 스레드 먼저 종료 → 노드 정리 (순서 바뀌면 core dump)
        spin_th.join(timeout=2.0)
        node.destroy_node()


if __name__ == "__main__":
    main()
