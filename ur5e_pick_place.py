#!/usr/bin/env python3
"""UR5e + Robotiq 2F-85 Pick & Place

티칭 도구(~/ur5e_points.json)에 저장한 home / pick / place 를 사용.

순서:
  그리퍼 열기 → home → pick 접근점 → (직선 하강) pick → 그리퍼 닫기 → 파지 확인
  → (직선 상승) → place 접근점 → (직선 하강) place → 그리퍼 열기 → (직선 상승) → home

실행 (드라이버가 먼저 실행 중이어야 함):
  source /opt/ros/humble/setup.bash
  python3 ~/ur5e_pick_place.py                 # 1회
  python3 ~/ur5e_pick_place.py --cycles 3      # 3회 반복 (place → pick 으로 되돌려 놓지는 않음)
  python3 ~/ur5e_pick_place.py --approach 80 --force 30

Ctrl+C → 로봇 즉시 감속 정지.
"""
import argparse
import math
import sys
import threading
import time

import numpy as np
import rclpy

from ur5e_move_gui import (URNode, Gripper, Dashboard, ROBOT_IP, load_points, fmt,
                           rotvec_to_mat, rot_angle, points_file, robot_serial)


class Abort(Exception):
    pass


STEPS = ["그리퍼 열기", "home 으로 이동 (MoveJ)", "pick 접근점", "pick 으로 직선 하강 (MoveL)",
         "그리퍼 닫기 + 파지 확인", "직선 상승", "place 접근점 (MoveJ)", "place 로 직선 하강 (MoveL)",
         "그리퍼 열기 (놓기)", "직선 상승", "home 복귀"]


def make_args(cycles=1, approach=100, jvel=45, jacc=60, lvel=100, slow=40, lacc=300, force=50, gspeed=128):
    """GUI 등에서 사용할 설정 객체 (입력은 사람 단위: deg, mm)"""
    return argparse.Namespace(cycles=cycles, approach=approach, jvel=math.radians(jvel), jacc=math.radians(jacc),
                              lvel=lvel / 1000, slow=slow / 1000, lacc=lacc / 1000, force=int(force), gspeed=int(gspeed))


class PickPlace:
    def __init__(self, node, args, grip=None, dash=None, logger=None, on_step=None):
        """grip/dash: 이미 있는 연결 재사용 (GUI), logger(msg), on_step(cycle, idx, total)"""
        self.n, self.a = node, args
        self.grip = grip or Gripper(ROBOT_IP)
        self.dash = dash or Dashboard(ROBOT_IP)
        self.logger, self.on_step = logger, on_step
        self.stop_req = threading.Event()
        self.step_no = 0
        self.cycle_no = 0
        self.holding = False

    # ---------- 출력 ----------
    def log(self, msg):
        line = f"[{time.strftime('%H:%M:%S')}] {msg}"
        if self.logger:
            self.logger(line)
        else:
            print(line, flush=True)

    def step(self, msg):
        self.step_no += 1
        self.log(f"({self.step_no:2d}) {msg}")
        if self.on_step:
            self.on_step(self.cycle_no, self.step_no - 1, len(STEPS))

    # ---------- 상태 확인 ----------
    def check_safe(self):
        if self.stop_req.is_set():
            raise Abort("사용자 정지")
        if self.dash.safety not in (None, "NORMAL"):
            raise Abort(f"안전 상태 {self.dash.safety}")
        if self.dash.mode not in (None, "RUNNING"):
            raise Abort(f"로봇 모드 {self.dash.mode} (서보 OFF?)")
        if self.holding and self.grip.state.get("OBJ") == 3:
            raise Abort("이동 중 물체를 놓쳤습니다! (그리퍼 OBJ=3)")

    def moving(self):
        return max(abs(v) for v in self.n.vel) > 0.005

    def wait_arrive(self, near_fn, desc, timeout=60.0):
        t0 = time.time(); started = False; still = None
        time.sleep(0.15)
        while True:
            self.check_safe()
            now = time.time()
            mv = self.moving()
            if mv:
                started, still = True, None
            elif still is None:
                still = now
            if near_fn() and not mv:
                self.log(f"     ✔ 도착 ({now - t0:.1f}초)")
                return
            if still and now - still > (1.0 if started else 3.0):
                raise Abort(f"{desc}: 목표에 도달하지 못하고 멈춤" if started else f"{desc}: 로봇이 움직이지 않음")
            if now - t0 > timeout:
                raise Abort(f"{desc}: 시간 초과")
            time.sleep(0.02)

    # ---------- 이동 ----------
    def movej_joints(self, q, desc):
        self.n.send_script([f"movej({fmt(q)}, a={self.a.jacc:.4f}, v={self.a.jvel:.4f})"])
        self.wait_arrive(lambda: max(abs(c - t) for c, t in zip(self.n.joints, q)) < math.radians(0.3), desc)

    def _near_pose(self, pt, Rt):
        def f():
            tcp = self.n.tcp
            return (np.linalg.norm(np.array(tcp[:3]) - pt) < 0.001
                    and rot_angle(rotvec_to_mat(tcp[3:]), Rt) < math.radians(0.3))
        return f

    @staticmethod
    def approach_pose(p, off):
        R = rotvec_to_mat(p["tcp"][3:])
        return np.array(p["tcp"][:3]) + R @ np.array([0, 0, -off]), R

    def movej_approach(self, p, desc):
        off = self.a.approach / 1000
        self.n.send_script([
            f"target = pose_trans(p{fmt(p['tcp'])}, p[0, 0, {-off:.4f}, 0, 0, 0])",
            f"movej(get_inverse_kin(target, qnear={fmt(p['joints'])}), a={self.a.jacc:.4f}, v={self.a.jvel:.4f})"])
        self.wait_arrive(self._near_pose(*self.approach_pose(p, off)), desc)

    def movel_to(self, p, desc, v):
        self.n.send_script([f"movel(p{fmt(p['tcp'])}, a={self.a.lacc:.4f}, v={v:.4f})"])
        self.wait_arrive(self._near_pose(np.array(p["tcp"][:3]), rotvec_to_mat(p["tcp"][3:])), desc)

    def movel_retreat(self, p, desc):
        off = self.a.approach / 1000
        self.n.send_script([f"movel(pose_trans(p{fmt(p['tcp'])}, p[0, 0, {-off:.4f}, 0, 0, 0]), "
                            f"a={self.a.lacc:.4f}, v={self.a.lvel:.4f})"])
        self.wait_arrive(self._near_pose(*self.approach_pose(p, off)), desc)

    # ---------- 그리퍼 ----------
    def grip_wait(self, timeout=5.0):
        """이전 상태값(OBJ=3 등)을 '완료'로 오인하지 않도록, 움직임(OBJ=0)을 본 뒤의 결과를 사용.
        이미 목표 위치라 움직임이 안 보이면 1초 후 현재 값을 사용."""
        t0 = time.time(); seen_moving = False
        while time.time() - t0 < timeout:
            if self.stop_req.is_set():
                raise Abort("사용자 정지")
            obj = self.grip.state.get("OBJ")
            if obj == 0:
                seen_moving = True
            elif obj in (1, 2, 3) and (seen_moving or time.time() - t0 > 1.0):
                return obj
            time.sleep(0.02)
        raise Abort("그리퍼 응답 시간 초과")

    def gripper_open(self):
        self.grip.move(0, self.a.gspeed, self.a.force)
        self.grip_wait()
        self.holding = False

    def gripper_close(self):
        self.grip.move(255, self.a.gspeed, self.a.force)
        obj = self.grip_wait()
        pos = self.grip.state.get("POS")
        if obj != 2:
            raise Abort(f"물체를 잡지 못했습니다 (OBJ={obj}, POS={pos}) - pick 위치/물체를 확인하세요")
        self.holding = True
        self.log(f"     ✔ 파지 성공 (POS={pos}, 약 {max(0, 85 * (1 - pos / 230)):.0f}mm)")

    # ---------- 전체 ----------
    def preflight(self, pts):
        t0 = time.time()
        while not (self.n.joints and self.n.tcp and self.dash.mode and self.dash.serial):
            if time.time() - t0 > 5:
                raise Abort("로봇 상태를 받지 못했습니다 (드라이버 실행 / 네트워크 확인)")
            time.sleep(0.1)
        sn = self.dash.serial
        for k in ("home", "pick", "place"):
            if k not in pts:
                raise Abort(f"이 로봇(S/N {sn})의 '{k}' 위치가 없습니다. 티칭 도구에서 저장하세요.")
        t0 = time.time()
        while not self.grip.state.get("connected"):
            if time.time() - t0 > 3:
                raise Abort(f"이 로봇(S/N {sn})에는 그리퍼가 연결되어 있지 않습니다 (Robotiq URCap 포트 63352 응답 없음)")
            time.sleep(0.1)
        if self.dash.mode != "RUNNING":
            raise Abort(f"서보가 ON이 아닙니다 (모드: {self.dash.mode})")
        self.check_safe()
        if self.grip.state.get("STA") != 3:
            self.log("그리퍼 활성화 중 (손가락이 한 번 열고 닫힘)...")
            self.grip.activate()
            t0 = time.time()
            while self.grip.state.get("STA") != 3:
                if time.time() - t0 > 10:
                    raise Abort("그리퍼 활성화 실패")
                time.sleep(0.1)
        for k in ("pick", "place"):
            zdir = rotvec_to_mat(pts[k]["tcp"][3:])[:, 2]
            if zdir[2] > -0.7:
                raise Abort(f"'{k}'에서 그리퍼가 아래를 보고 있지 않습니다 (툴Z={np.round(zdir, 2)}) - 접근 방향 위험")

    def cycle(self, pts, i):
        self.cycle_no, self.step_no = i, 0
        self.log(f"══════ 사이클 {i} 시작 ══════")
        self.step("그리퍼 열기");                    self.gripper_open()
        self.step("home 으로 이동 (MoveJ)");          self.movej_joints(pts["home"]["joints"], "home")
        self.step(f"pick 접근점 (위 {self.a.approach:.0f}mm)"); self.movej_approach(pts["pick"], "pick 접근점")
        self.step("pick 으로 직선 하강 (MoveL)");     self.movel_to(pts["pick"], "pick", self.a.slow)
        self.step(f"그리퍼 닫기 (힘 {self.a.force})"); self.gripper_close()
        self.step("직선 상승");                       self.movel_retreat(pts["pick"], "pick 후퇴")
        self.step("place 접근점 (MoveJ)");           self.movej_approach(pts["place"], "place 접근점")
        self.step("place 로 직선 하강 (MoveL)");      self.movel_to(pts["place"], "place", self.a.slow)
        self.step("그리퍼 열기 (놓기)");              self.gripper_open()
        self.step("직선 상승");                       self.movel_retreat(pts["place"], "place 후퇴")
        self.step("home 복귀");                       self.movej_joints(pts["home"]["joints"], "home")
        self.log(f"══════ 사이클 {i} 완료 ══════")

    def stop(self):
        self.n.send_script(["stopj(2.0)"])

    def run(self, pts):
        """전체 실행. 반환: (성공 여부, 메시지)"""
        try:
            self.preflight(pts)
            self.log(f"준비 완료: pick {np.round(np.array(pts['pick']['tcp'][:3]) * 1000, 1)}mm → "
                     f"place {np.round(np.array(pts['place']['tcp'][:3]) * 1000, 1)}mm, 접근 {self.a.approach:.0f}mm")
            for i in range(1, self.a.cycles + 1):
                self.cycle(pts, i)
            self.log("✔ 모든 작업 완료")
            return True, "완료"
        except Abort as e:
            self.stop()
            self.log(f"✖ 중단: {e}")
            return False, str(e)


def main():
    ap = argparse.ArgumentParser(description="UR5e Pick & Place")
    ap.add_argument("--cycles", type=int, default=1)
    ap.add_argument("--approach", type=float, default=100, help="접근/후퇴 높이 [mm]")
    ap.add_argument("--jvel", type=float, default=45, help="MoveJ 속도 [deg/s]")
    ap.add_argument("--jacc", type=float, default=60, help="MoveJ 가속 [deg/s^2]")
    ap.add_argument("--lvel", type=float, default=100, help="MoveL 속도 [mm/s]")
    ap.add_argument("--slow", type=float, default=40, help="물체 접근 마지막 하강 속도 [mm/s]")
    ap.add_argument("--lacc", type=float, default=300, help="MoveL 가속 [mm/s^2]")
    ap.add_argument("--force", type=int, default=50, help="그리퍼 힘 0~255")
    ap.add_argument("--gspeed", type=int, default=128, help="그리퍼 속도 0~255")
    o = ap.parse_args()
    a = make_args(o.cycles, o.approach, o.jvel, o.jacc, o.lvel, o.slow, o.lacc, o.force, o.gspeed)

    rclpy.init()
    node = URNode()
    spin_th = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_th.start()
    pp = PickPlace(node, a)
    t0 = time.time()
    while not robot_serial() and time.time() - t0 < 5:   # 로봇 시리얼 확인 → 그 로봇의 티칭 파일 사용
        time.sleep(0.1)
    pts = load_points()
    pp.log(f"로봇 S/N {robot_serial() or '?'}  티칭 파일 {points_file() or '-'}")
    code = 0
    try:
        ok, _ = pp.run(pts)
        code = 0 if ok else 1
    except KeyboardInterrupt:
        pp.stop()
        pp.log("■ 사용자 정지 (Ctrl+C)")
        code = 130
    finally:
        time.sleep(0.2)            # 정지 명령 전송 여유
        rclpy.try_shutdown()       # spin 스레드 종료 → 그 다음 노드 정리 (순서 중요)
        spin_th.join(timeout=2.0)
        node.destroy_node()
    sys.exit(code)


if __name__ == "__main__":
    main()
