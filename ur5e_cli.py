#!/usr/bin/env python3
"""UR5e 터미널 조그 CLI (ROS 2 Humble + ur_robot_driver, Robotiq 2F-85)

키 하나로 조작하는 간단한 텍스트 컨트롤러. GUI(ur5e_move_gui.py)와 같은 통신 모듈을 사용.

  관절 조그     1 2 3 4 5 6  = J1~J6 +      q w e r t y = J1~J6 −
  TCP 조그      a s d        = X+ Y+ Z+     z x c       = X− Y− Z−
  스텝          - / =        = 작게 / 크게  f           = 기준 Base ↔ Tool
  그리퍼        g 활성화   o 열기   p 닫기
  서보          n 서보 ON  m 서보 OFF  b 보호정지 해제
  원위치        h 원위치로 이동   j 현재 위치를 home 으로 저장
  기타          v 속도 슬라이더 %   k Pick & Place 1회   i 정보   ? 도움말
  정지          Space (이동 중 즉시 감속 정지)
  종료          Esc 또는 Ctrl+C

실행 (드라이버가 먼저 실행 중이어야 함):
  source /opt/ros/humble/setup.bash
  python3 ur5e_cli.py
"""
import math
import os
import select
import shutil
import sys
import termios
import threading
import time
import tty
import unicodedata

import rclpy

from ur5e_move_gui import (URNode, Gripper, Dashboard, App, ROBOT_IP, fmt, load_points, save_points,
                           home_joints, POINTS_FILE)

J_STEPS = [0.5, 1, 2, 5, 10]     # 관절 스텝 [deg]
L_STEPS = [1, 2, 5, 10, 20]      # 직선 스텝 [mm]
J_VEL, J_ACC = 15, 30            # 관절 조그 [deg/s, deg/s^2]
L_VEL = 50                       # 직선 조그 [mm/s]
HOME_VEL, HOME_ACC = 30, 60      # 원위치 이동 [deg/s, deg/s^2]

KEY_J_PLUS, KEY_J_MINUS = "123456", "qwerty"
KEY_L_PLUS, KEY_L_MINUS = "asd", "zxc"

HELP_ROWS = [
    ("관절 조그", "1 2 3 4 5 6 = J1~J6 +    q w e r t y = J1~J6 -"),
    ("TCP 조그", "a s d = X+ Y+ Z+         z x c = X- Y- Z-"),
    ("스텝", "- 작게   = 크게           f 기준 Base/Tool"),
    ("그리퍼", "g 활성화   o 열기   p 닫기"),
    ("서보", "n ON   m OFF   b 보호정지 해제"),
    ("원위치", "h 이동   j 현재 위치를 home 저장"),
    ("기타", "v 속도%   k Pick&Place   i 정보   ? 도움말"),
    ("정지/종료", "Space 정지         Esc / Ctrl+C 종료"),
]


def disp_width(s):
    return sum(2 if unicodedata.east_asian_width(c) in "WF" else 1 for c in s)


def pad(s, width):
    return s + " " * (width - disp_width(s))


def make_help():
    """한글 2칸 폭을 계산해 테두리가 맞는 도움말 상자 생성"""
    rows = [f" {pad(k, 10)} {v} " for k, v in HELP_ROWS]
    w = max(disp_width(r) for r in rows)
    title = f" UR5e CLI ({ROBOT_IP}) "
    left = (w - len(title)) // 2
    top = "┌" + "─" * left + title + "─" * (w - left - len(title)) + "┐"
    return "\n".join([""] + [top] + [f"│{pad(r, w)}│" for r in rows] + ["└" + "─" * w + "┘"])


HELP = make_help()


def fit(s, width):
    """터미널 폭에 맞게 자르기 (한글 2칸 계산)"""
    out, w = "", 0
    for c in s:
        cw = 2 if unicodedata.east_asian_width(c) in "WF" else 1
        if w + cw > width:
            break
        out += c; w += cw
    return out


class CLI:
    def __init__(self, node):
        self.n = node
        self.dash = Dashboard(ROBOT_IP)
        self.grip = Gripper(ROBOT_IP)
        self.step_i = 2               # 2° / 5mm
        self.frame = "base"
        self.speed = None             # 마지막으로 설정한 속도 슬라이더 [%]
        self.pp = None                # 실행 중인 Pick & Place
        self.fd = sys.stdin.fileno()
        self.old_tty = termios.tcgetattr(self.fd)
        self.lock = threading.Lock()

    # ---------- 화면 ----------
    def status_line(self):
        n, d, g = self.n, self.dash, self.grip.state
        j = " ".join(f"{math.degrees(q):6.1f}" for q in n.joints) if n.joints else "-"
        t = " ".join(f"{v * 1000:6.1f}" for v in n.tcp[:3]) if n.tcp else "-"
        mv = "이동중" if max(abs(v) for v in n.vel) > 0.005 else "정지"
        if self.pp:
            mv = "P&P 실행중"
        gtxt = f"G{g.get('POS')}" if g.get("STA") == 3 else ("G미활성" if g.get("connected") else "G-")
        spd = f"{self.speed}%" if self.speed is not None else "-"
        return (f"[{d.mode or '-'}|{d.safety or '-'}] J:{j} | TCP:{t}mm | {gtxt} | "
                f"스텝 {J_STEPS[self.step_i]}°/{L_STEPS[self.step_i]}mm {self.frame} | 속도 {spd} | {mv}")

    def draw(self):
        width = shutil.get_terminal_size((120, 20)).columns - 1
        with self.lock:
            sys.stdout.write("\r\033[K" + fit(self.status_line(), width))
            sys.stdout.flush()

    def say(self, msg):
        with self.lock:
            sys.stdout.write("\r\033[K" + msg + "\n")
            sys.stdout.flush()

    def ask(self, prompt):
        """한 줄 입력 (잠시 일반 터미널 모드)"""
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_tty)
        try:
            with self.lock:
                sys.stdout.write("\r\033[K")
            return input(prompt).strip()
        except EOFError:
            return ""
        finally:
            tty.setcbreak(self.fd)

    def confirm(self, prompt):
        return self.ask(prompt + " (y/N): ").lower() == "y"

    # ---------- 공통 확인 ----------
    def ready(self, need_joints=True):
        if self.pp:
            self.say("✖ Pick & Place 실행 중입니다 (Space로 정지)"); return False
        if self.dash.mode != "RUNNING":
            self.say(f"✖ 서보가 ON이 아닙니다 (모드 {self.dash.mode}) → n 키로 서보 ON"); return False
        if self.dash.safety != "NORMAL":
            self.say(f"✖ 안전 상태 {self.dash.safety} → b 키로 보호정지 해제"); return False
        if need_joints and not (self.n.joints and self.n.tcp):
            self.say("✖ 로봇 상태 수신 없음 (드라이버 확인)"); return False
        return True

    # ---------- 동작 ----------
    def jog_joint(self, idx, sign):
        if not self.ready():
            return
        step = J_STEPS[self.step_i]
        q = list(self.n.joints)
        q[idx] += sign * math.radians(step)
        if abs(q[idx]) > 2 * math.pi:
            self.say("✖ 관절 범위(±360°) 초과"); return
        self.n.send_script([f"movej({fmt(q)}, a={math.radians(J_ACC):.4f}, v={math.radians(J_VEL):.4f})"])
        self.say(f"→ J{idx + 1} {sign * step:+g}°")

    def jog_lin(self, axis, sign):
        if not self.ready():
            return
        step = L_STEPS[self.step_i]
        d = [0.0] * 6
        d[axis] = sign * step / 1000
        v = L_VEL / 1000
        self.n.send_script(App._rel_movel(d, self.frame, v * 3, v))
        self.say(f"→ {'XYZ'[axis]} {sign * step:+g}mm ({self.frame})")

    def stop(self):
        if self.pp:
            self.pp.stop_req.set()
        self.n.send_script(["stopj(3.0)"])
        self.say("■ 정지")

    def go_home(self):
        if not self.ready():
            return
        q = home_joints()
        src = "ur5e_points.json 'home'" if "home" in load_points() else "기본값"
        diff = max(abs(a - b) for a, b in zip(q, self.n.joints))
        self.say(f"  원위치({src}): " + ", ".join(f"{math.degrees(v):.1f}" for v in q)
                 + f"°   (최대 관절 변화 {math.degrees(diff):.1f}°)")
        if not self.confirm("  원위치로 이동할까요? 경로에 장애물이 없는지 확인"):
            self.say("  취소"); return
        self.n.send_script([f"movej({fmt(q)}, a={math.radians(HOME_ACC):.4f}, v={math.radians(HOME_VEL):.4f})"])
        self.say("→ 원위치로 이동")

    def save_home(self):
        if not (self.n.joints and self.n.tcp):
            self.say("✖ 로봇 상태 수신 없음"); return
        if not self.confirm(f"  현재 위치를 'home'으로 저장할까요? ({POINTS_FILE})"):
            self.say("  취소"); return
        pts = load_points()
        pts["home"] = {"joints": [round(q, 6) for q in self.n.joints], "tcp": [round(v, 6) for v in self.n.tcp],
                       "gripper": self.grip.state.get("POS"), "time": time.strftime("%Y-%m-%d %H:%M:%S")}
        save_points(pts)
        self.say("✔ home 저장")

    def gripper(self, pos):
        if self.grip.state.get("STA") != 3:
            self.say("✖ 그리퍼 미활성 → g 키로 활성화"); return
        self.grip.move(pos, 128, 50)
        self.say("→ 그리퍼 " + ("열기" if pos == 0 else "닫기 (힘 50)"))

    def grip_activate(self):
        if self.confirm("  그리퍼를 활성화할까요? (손가락이 한 번 닫혔다 열림)"):
            self.grip.activate(); self.say("→ 그리퍼 활성화")

    def servo_on(self):
        def run():
            self.say("→ 서보 ON: 전원 켜는 중...")
            self.dash.cmd("power on")
            if not self.dash.wait_mode("IDLE", 30):
                self.say(f"✖ 서보 ON 실패 (모드 {self.dash.mode})"); return
            self.say("→ 브레이크 해제 (딸깍 소리 정상)")
            self.dash.cmd("brake release")
            if not self.dash.wait_mode("RUNNING", 20):
                self.say(f"✖ 서보 ON 실패 (모드 {self.dash.mode})"); return
            time.sleep(1.0); self.n.resend()
            self.say("✔ 서보 ON 완료")
        threading.Thread(target=run, daemon=True).start()

    def servo_off(self):
        if self.confirm("  서보 OFF (전원 끄고 브레이크 잠금) 할까요?"):
            self.say(f"→ 서보 OFF: {self.dash.cmd('power off')}")

    def unlock(self):
        r1 = self.dash.cmd("close safety popup"); r2 = self.dash.cmd("unlock protective stop")
        self.say(f"→ 보호정지 해제: {r2 or r1}")
        time.sleep(1.0); self.n.resend()

    def set_speed(self):
        s = self.ask("  속도 슬라이더 % (1~100): ")
        try:
            v = int(s)
        except ValueError:
            self.say("  취소"); return
        if not 1 <= v <= 100:
            self.say("✖ 1~100 사이로 입력"); return
        if self.n.set_speed(v / 100):
            self.speed = v; self.say(f"✔ 속도 {v}%")
        else:
            self.say("✖ 속도 서비스 없음 (드라이버 확인)")

    def info(self):
        q = self.dash.cmd
        g = lambda k: (self.grip._cmd(f"GET {k}") or "-").split(" ", 1)[-1]
        self.say(f"  모델 {q('get robot model')} | 시리얼 {q('get serial number')} | {q('PolyscopeVersion')}")
        self.say(f"  원격제어 {q('is in remote control')} | 프로그램 {q('programState')}")
        self.say(f"  그리퍼 펌웨어 {g('FWV')} | {g('VER')} | 시리얼 {g('SNU')} | OBJ {self.grip.state.get('OBJ')}")
        pts = load_points()
        self.say(f"  티칭 위치: {', '.join(pts) or '없음'}  ({POINTS_FILE})")

    def pick_place(self):
        if not self.ready():
            return
        import ur5e_pick_place as ppm
        pts = load_points()
        miss = [k for k in ("home", "pick", "place") if k not in pts]
        if miss:
            self.say(f"✖ 티칭 위치 없음: {', '.join(miss)} (GUI 티칭 도구에서 저장)"); return
        if not self.confirm("  Pick & Place 1회 실행할까요? (Space로 정지)"):
            self.say("  취소"); return
        self.pp = ppm.PickPlace(self.n, ppm.make_args(), grip=self.grip, dash=self.dash, logger=self.say)

        def run():
            try:
                self.pp.run(pts)
            finally:
                self.pp = None
        threading.Thread(target=run, daemon=True).start()

    # ---------- 메인 루프 ----------
    def handle(self, k):
        if k in KEY_J_PLUS:
            self.jog_joint(KEY_J_PLUS.index(k), +1)
        elif k in KEY_J_MINUS:
            self.jog_joint(KEY_J_MINUS.index(k), -1)
        elif k in KEY_L_PLUS:
            self.jog_lin(KEY_L_PLUS.index(k), +1)
        elif k in KEY_L_MINUS:
            self.jog_lin(KEY_L_MINUS.index(k), -1)
        elif k == " ":
            self.stop()
        elif k in "-_":
            self.step_i = max(0, self.step_i - 1)
        elif k in "=+":
            self.step_i = min(len(J_STEPS) - 1, self.step_i + 1)
        elif k == "f":
            self.frame = "tool" if self.frame == "base" else "base"; self.say(f"  기준: {self.frame}")
        elif k == "g":
            self.grip_activate()
        elif k == "o":
            self.gripper(0)
        elif k == "p":
            self.gripper(255)
        elif k == "n":
            self.servo_on()
        elif k == "m":
            self.servo_off()
        elif k == "b":
            self.unlock()
        elif k == "h":
            self.go_home()
        elif k == "j":
            self.save_home()
        elif k == "v":
            self.set_speed()
        elif k == "k":
            self.pick_place()
        elif k == "i":
            self.info()
        elif k == "?":
            self.say(HELP)

    def run(self):
        self.say(HELP)
        self.say("로봇 상태 수신 대기 중... (?: 도움말, Esc: 종료)")
        tty.setcbreak(self.fd)
        try:
            while True:
                self.draw()
                r, _, _ = select.select([sys.stdin], [], [], 0.2)
                if not r:
                    continue
                k = os.read(self.fd, 1).decode(errors="ignore")
                if k == "\x1b":
                    # 방향키 등 이스케이프 시퀀스(\x1b[A ...)는 무시, Esc 단독이면 종료
                    r2, _, _ = select.select([sys.stdin], [], [], 0.05)
                    if r2:
                        os.read(self.fd, 8); continue
                    break
                self.handle(k)
        finally:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_tty)
            if max(abs(v) for v in self.n.vel) > 0.005 or self.pp:
                self.stop()
            self.say("종료")


def main():
    if not sys.stdin.isatty():
        print("터미널에서 실행하세요 (키 입력 필요)"); return
    rclpy.init()
    node = URNode()
    spin_th = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_th.start()
    try:
        CLI(node).run()
    except KeyboardInterrupt:
        node.send_script(["stopj(3.0)"])
        print("\n■ Ctrl+C - 정지 후 종료")
    finally:
        time.sleep(0.2)
        rclpy.try_shutdown()
        spin_th.join(timeout=2.0)
        node.destroy_node()


if __name__ == "__main__":
    main()
