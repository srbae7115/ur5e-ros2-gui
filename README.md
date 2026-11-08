# UR5e ROS 2 GUI + Pick & Place

Ubuntu 22.04 + ROS 2 Humble 환경에서 **UR5e (PolyScope 5)** 와 **Robotiq 2F-85** 그리퍼를
PC에서 펜던트처럼 제어하는 GUI와, 티칭 기반 Pick & Place 프로그램입니다.

| 파일 | 설명 |
|---|---|
| `ur5e_move_gui.py` | 메인 GUI: MoveJ / MoveL / 조그 / 그리퍼 / 서보 / 진행률 / 티칭 도구 / Pick & Place 창 |
| `ur5e_pick_place.py` | Pick & Place 로직 (GUI에서 사용, 터미널 단독 실행도 가능) |
| `ur5e_go_home.sh` | ROS 액션으로 원위치 복귀하는 간단한 예제 |
| `ur5e_points.example.json` | 티칭 위치 파일 예시 (`~/ur5e_points.json` 형식) |

---

## 1. 환경

- Ubuntu 22.04, ROS 2 Humble
- `ur_robot_driver` 2.15.0, `ur_client_library` 2.15.2
- UR5e, PolyScope 5.25 (Remote Control 모드)
- Robotiq 2F-85 + Robotiq URCap (로봇에 설치되어 TCP 63352 소켓 제공)

```bash
sudo apt install ros-humble-ur ros-humble-moveit
```

## 2. 네트워크

| 장비 | IP |
|---|---|
| UR5e 컨트롤러 | `192.168.0.9` |
| PC (`eno1`) | `192.168.0.201/24` |

로봇 IP 확인: 펜던트 ☰ → 정보, 또는 ☰ → 설정 → 시스템 → 네트워크

```bash
ping 192.168.0.9
# 사용 포트: 29999(Dashboard) 30001~30004(Primary/Secondary/RTDE) 63352(Robotiq)
```

다른 IP를 쓰려면 `ur5e_move_gui.py`의 `ROBOT_IP`를 수정하세요.

## 3. 실행

```bash
# 터미널 1: 드라이버 (Remote Control 모드 → headless_mode 사용, URCap/펜던트 프로그램 불필요)
source /opt/ros/humble/setup.bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.0.9 headless_mode:=true

# 터미널 2: GUI
source /opt/ros/humble/setup.bash
python3 ur5e_move_gui.py

# (선택) 터미널 3: MoveIt + RViz
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true
```

Remote Control 모드에서는 펜던트 속도 슬라이더가 잠기므로 PC에서 설정합니다.

```bash
ros2 service call /io_and_status_controller/set_speed_slider ur_msgs/srv/SetSpeedSliderFraction "{speed_slider_fraction: 0.3}"
```

정확한 TCP를 위해 캘리브레이션 추출을 권장합니다 (드라이버 로그의 calibration mismatch 경고).

```bash
ros2 launch ur_calibration calibration_correction.launch.py robot_ip:=192.168.0.9 target_filename:="$HOME/ur5e_calib.yaml"
```

## 4. GUI 기능 (`ur5e_move_gui.py`)

모든 버튼에 마우스를 올리면 설명 말풍선이 나오고, **F1** 로 전체 도움말을 볼 수 있습니다.

- **현재 상태**: 관절 각도, TCP (펜던트와 같은 base 좌표계, 회전벡터 rad), 로봇 프로그램 상태
- **서보**: 서보 ON (전원 → 브레이크 해제 → ROS 제어 재연결), 서보 OFF, 보호정지 해제, RUNNING / NORMAL 표시
- **MoveJ**: 관절 각도 절대 / 상대 이동, 원위치 값, 원위치로 이동
- **MoveL**: 직선 이동, 절대 / 상대(base) / 상대(tool)
- **조그**: X, Y, Z, Rx, Ry, Rz ± 버튼 (Base / Tool 기준, 스텝·속도 설정)
- **그리퍼**: 활성화, 열기 / 닫기 / 위치, 속도·힘 슬라이더 (≈N 표시), 물체 감지 상태
- **동작 진행**: 진행률 바, 이동 완료 / 중단 / 안전정지 판정
- **속도 슬라이더**, **■ 정지**, **ROS 제어 복구** (GUI로 URScript를 보낸 뒤 MoveIt을 다시 쓸 때)
- **ⓘ 정보/펌웨어**: 로봇 모델, 시리얼, PolyScope 버전, 그리퍼 펌웨어, PC/ROS 패키지 버전
- **📍 티칭 도구**: Freedrive(손으로 끌기, 120초 자동 해제), 이름 붙여 위치 저장, 접근점 / 위치로 이동
- **▶ Pick & Place**: 설정, 실행 / 정지, 11단계 진행 표시

### 동작 방식

| 기능 | 통신 |
|---|---|
| 팔 이동 (movej / movel / freedrive) | URScript → `/urscript_interface/script_command` |
| 상태 | `/joint_states`, `/tcp_pose_broadcaster/pose`, `/io_and_status_controller/robot_program_running` |
| 속도 / 프로그램 재전송 | `/io_and_status_controller/set_speed_slider`, `.../resend_robot_program` |
| 서보 / 정보 | Dashboard 서버 TCP 29999 |
| 그리퍼 | Robotiq URCap 소켓 TCP 63352 (`SET POS .. SPE .. FOR .. GTO 1`, `GET OBJ`) |

> URScript를 보내면 드라이버의 External Control 프로그램이 중단됩니다.
> MoveIt이나 ROS 컨트롤러로 다시 제어하려면 GUI의 **[ROS 제어 복구]** 를 누르세요.

## 5. 티칭 → Pick & Place

1. GUI → **📍 티칭 도구**
2. **Freedrive ON** → 손으로 대략적인 위치를 잡고, 메인 창 조그로 미세 조정
3. 빠른 저장 **[home] [pick] [place]** → `~/ur5e_points.json` 에 저장
   - pick / place 는 "그리퍼를 닫으면 바로 잡히는 높이"로 저장. 접근점은 자동 계산
4. GUI → **▶ Pick & Place** → 실행, 또는 터미널에서:

```bash
python3 ur5e_pick_place.py                       # 1회
python3 ur5e_pick_place.py --force 30 --slow 20  # 힘 약하게, 하강 천천히
python3 ur5e_pick_place.py --help
```

순서:

```
그리퍼 열기 → home → pick 접근점(MoveJ, IK는 티칭 관절 근처 해) → 직선 하강(MoveL)
→ 그리퍼 닫기 + 파지 확인 → 직선 상승 → place 접근점 → 직선 하강 → 그리퍼 열기
→ 직선 상승 → home
```

### 파지 판단

Robotiq 그리퍼의 물체 감지값 `OBJ` 를 사용합니다.

| OBJ | 의미 |
|---|---|
| 0 | 이동 중 |
| 1 | 여는 도중 막혀서 멈춤 |
| **2** | **닫는 도중 막혀서 멈춤 → 파지 성공** |
| **3** | **목표 위치 도착 → 물체 없음** |

- 닫기 후 `OBJ != 2` 이면 중단합니다. 이전 상태값을 결과로 오인하지 않도록 `OBJ=0`(이동)을 본 뒤의 결과를 사용합니다.
- 이동 중 `OBJ` 가 3으로 바뀌면 물체를 놓친 것으로 보고 정지합니다.
- 보호정지, 서보 OFF, 목표 미도달, 시간 초과, Ctrl+C / [■ 정지] 시에도 정지합니다.

## 6. 안전

- 처음 동작은 속도 10~30%, 작은 스텝으로 시험하세요.
- GUI / Pick & Place는 **충돌 검사를 하지 않습니다.**
- [■ 정지]는 소프트웨어 정지입니다. 위급할 때는 펜던트 비상정지를 누르세요.
- Freedrive 전에 펜던트 설치 탭의 페이로드(2F-85 약 0.9kg)를 확인하세요.

## 7. 참고: 실시간 / 비동기

- `ur_robot_driver` 의 ros2_control 루프는 500Hz입니다. 제대로 쓰려면 lowlatency 커널과 FIFO 스케줄링 설정을 권장합니다.
- 연속 스트리밍 제어: `forward_position_controller` / `forward_velocity_controller`, MoveIt Servo, URScript `servoj` / `speedl`
- 비동기 실행: `FollowJointTrajectory` 액션, 또는 `ur_rtde` 의 `asynchronous=True`
