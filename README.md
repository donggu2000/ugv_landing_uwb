# UGV Landing UWB Simulation

ROS 2 Humble + Gazebo Sim에서 사각 UGV 위 4개 UWB anchor와 x500 tag 거리측정/위치추정/착륙 시뮬레이션을 만들기 위한 시작 워크스페이스입니다.

참고한 Decawave 예제는 Gazebo 플러그인이라기보다 UWB 문자열 패킷 생성/파싱/RViz 표시 예제입니다. 이 워크스페이스는 그 아이디어를 Gazebo/PX4에 붙이기 쉽게 나눴습니다.

- `ugv_landing.sdf`: 사각 UGV, 랜딩패드, anchor 4개, PX4 X500 모델이 보이는 Gazebo Sim world
- `models/x500`, `models/x500_base`: PX4 Gazebo 공식 X500 모델 파일
- `models/x500_uwb`: 공식 X500 본체 아래에 UWB tag를 fixed joint로 단 wrapper
- `uwb_range_simulator`: UGV pose와 x500 tag pose를 받아 anchor-tag 거리값 생성
- `trilateration_estimator`: UWB 거리값으로 tag 위치 추정
- `demo_pose_publisher`: Gazebo/PX4 없이도 토픽 체인을 바로 테스트하는 데모 pose publisher

## 컨테이너 빌드/실행

```bash
cd /home/hyeon/ugv_landing_sim
./scripts/build_image.sh
./scripts/run_container.sh
```

컨테이너 안에서 GPU/GUI 확인:

```bash
nvidia-smi
glxinfo -B
```

## ROS2 패키지 빌드

컨테이너 안에서:

```bash
cd /workspaces/ugv_landing_sim
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## UWB 거리측정/위치추정만 먼저 테스트

```bash
ros2 launch ugv_landing_sim uwb_demo.launch.py
```

다른 터미널에서:

```bash
source /workspaces/ugv_landing_sim/install/setup.bash
ros2 topic echo /uwb/decawave_string
ros2 topic echo /uwb/tag_pose_estimate
```

## Bunker Pro 실장비 anchor RViz 확인

현재 연결된 DWM1001C 4개를 Bunker Pro 기준 1m x 1m 사각형 anchor로 두고 RViz에 표시하는 launch입니다. 좌표계는 `x=전방`, `y=좌측`, `z=위`이며, 기본 anchor 좌표는 Bunker 중심 기준 `(+/-0.5m, +/-0.5m, 0.0m)`입니다.

```bash
cd /home/hyeon/ugv_landing_sim
colcon build --symlink-install
source install/setup.bash
ros2 launch ugv_landing_sim bunker_uwb_viz.launch.py
```

좌표나 USB 포트 매핑은 여기서 바꾸면 됩니다.

```text
src/ugv_landing_sim/config/bunker_uwb_params.yaml
```

태그는 아직 연결하지 않는 전제로 anchor marker만 `/uwb/anchor_markers`에 publish합니다. 나중에 태그를 연결하면 같은 좌표계의 pose/range topic을 붙여 RViz에 tag marker와 추정 위치를 추가하면 됩니다.

태그가 연결되어 있으면 기본값으로 `/dev/ttyACM3`를 열고 DWM1001 TLV/API의 `dwm_loc_get` 응답을 읽습니다. 태그 위치는 2D 모드로 publish하므로 x/y만 사용하고 z는 `0.0m`로 고정합니다.

```bash
ros2 topic echo /uwb/tag_raw
ros2 topic echo /uwb/tag_pose
```

태그 포트가 달라지면:

```bash
ros2 launch ugv_landing_sim bunker_uwb_viz.launch.py tag_port:=/dev/ttyACM3
```

태그 포트가 열리는데 `/uwb/tag_raw`가 비어 있으면, 해당 DWM1001C가 아직 tag 모드가 아닐 수 있습니다. 먼저 확인:

```bash
python3 - <<'PY'
import serial, time
ser = serial.Serial('/dev/ttyACM4', 115200, timeout=0.2)
ser.write(b'\r\r')
time.sleep(0.5)
ser.write(b'nmg\r')
time.sleep(0.5)
print(ser.read(4096).decode(errors='ignore'))
ser.close()
PY
```

`mode: tn`이 아니라면, 그 보드가 태그로 쓸 보드인지 확인한 뒤 DWM shell에서 `nmt`로 tag node 모드로 바꾸면 됩니다.

## Gazebo Sim world 실행

```bash
ros2 launch ugv_landing_sim sim.launch.py
```

또는 직접:

```bash
IGN_GAZEBO_RESOURCE_PATH=$PWD/src/ugv_landing_sim/models ign gazebo -r src/ugv_landing_sim/worlds/ugv_landing.sdf
```

현재 world는 PX4 공식 `x500_base` mesh/body 모델에 UWB tag를 추가한 `x500_uwb` wrapper를 UGV 옆 바닥에 비고정 물리 객체로 띄웁니다. PX4 모터 플러그인까지 포함한 전체 `x500` 모델도 `models/x500`에 같이 들어있으니, PX4 SITL을 붙일 때 `model://x500` 기반 wrapper로 전환하면 됩니다.

## 전체 구조를 이렇게 가져가면 좋음

1. 지금 상태에서 `uwb_demo.launch.py`로 UWB 거리 생성과 위치추정이 정상 동작하는지 확인
2. `sim.launch.py`로 UGV/anchor/world가 Gazebo Sim에서 잘 뜨는지 확인
3. PX4 SITL x500을 붙이고, x500의 tag 위치를 `/x500/tag_pose`로 publish
4. UGV가 움직이면 UGV pose를 `/ugv/pose`로 publish
5. `/uwb/tag_pose_estimate`를 착륙 제어 입력으로 사용

## PX4 x500을 붙일 때

PX4의 Gazebo x500 모델은 보통 `PX4-Autopilot` 쪽 SITL에서 가져오는 편이 제일 편합니다. 이 컨테이너는 ROS 2 Humble/Gazebo Sim/ros_gz 기반을 준비해두었고, PX4는 다음처럼 별도로 붙이는 것을 권장합니다.

```bash
cd /workspaces/ugv_landing_sim/external
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
```

PX4 버전마다 Gazebo 버전 요구사항이 다를 수 있으니, 먼저 PX4 문서 기준의 Ubuntu 22.04 + ROS 2 Humble 호환 태그를 고른 뒤 `make px4_sitl gz_x500` 계열로 확인하는 게 좋습니다.

가져온 X500 모델 출처:

```text
https://github.com/PX4/PX4-gazebo-models/tree/main/models/x500
https://github.com/PX4/PX4-gazebo-models/tree/main/models/x500_base
```

## UWB anchor 배치 주의

UGV 네 귀퉁이에 anchor 4개를 모두 같은 높이로 두면 anchor들이 한 평면에 놓입니다. 이 경우 순수 3D trilateration은 z축이 애매해질 수 있습니다. 현재 estimator는 착륙 시나리오에 맞춰 “tag가 anchor 평면보다 위에 있다”는 가정을 사용해 z를 복원합니다.

더 정확히 하려면 anchor 1개를 살짝 다른 높이에 두거나, IMU/barometer/vision altitude를 같이 융합하는 구조가 좋습니다.

## Decawave 참고 repo

참고 링크:

```text
https://github.com/Xpect8tions/Decawave-ros-data-sim
```

그 repo의 `/output` 문자열 방식은 `/uwb/decawave_string` 토픽으로 비슷하게 남겨두었습니다. 실제 제어/추정에는 파싱이 쉬운 `/uwb/ranges`를 먼저 쓰는 편이 안전합니다.

---

# English Version

This repository is a ROS 2 workspace for UWB-based tag localization and RViz visualization for Bunker Pro drone landing experiments.

The current real-hardware setup uses four DWM1001C boards as anchors and one DWM1001C board as a tag. The anchors are arranged as a 1 m x 1 m square, and the tag position is visualized in RViz using only the x/y plane. The z value is forced to `0.0 m`.

## Features

- RViz visualization of four fixed UWB anchors
- DWM1001C tag serial reader using the binary TLV API
- `/uwb/tag_pose` publisher for the estimated tag position
- `/uwb/tag_markers` publisher for the tag marker, label, center-to-tag line, and distance text
- 2D distance display from the square center to the tag
- Gazebo Sim world and PX4 X500 model files for later simulation work

## Current Anchor Layout

The anchor square is defined in `bunker_base` frame:

```text
A3 left_top      (0.0, 1.0, 0.0)
A0 right_top     (1.0, 1.0, 0.0)
A2 left_bottom   (0.0, 0.0, 0.0)
A1 right_bottom  (1.0, 0.0, 0.0)
```

The square center is:

```text
(0.5, 0.5, 0.0)
```

The center-to-tag distance is calculated in 2D:

```text
distance = sqrt((tag_x - 0.5)^2 + (tag_y - 0.5)^2)
```

## Hardware Mapping

```text
A0 right_top:     2104003357 / J-Link 000760154674
A1 right_bottom:  210400146A / J-Link 000760154625
A2 left_bottom:   2104001DDC / J-Link 000760154382
A3 left_top:      2111001BC4 / J-Link 000760217846
Tag:              J-Link 000760217793
```

The tag is expected at:

```text
/dev/serial/by-id/usb-SEGGER_J-Link_000760217793-if00
```

## Build

On the laptop:

```bash
cd /home/hyeon/ugv_landing_sim
source /opt/ros/humble/setup.bash
colcon build --symlink-install --base-paths src/ugv_landing_sim --packages-select ugv_landing_sim
source install/setup.bash
```

## Run Laptop-Only RViz Visualization

Use this mode when the tag is connected directly to the laptop.

```bash
cd /home/hyeon/ugv_landing_sim
source /opt/ros/humble/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=10
export ROS_LOCALHOST_ONLY=1
unset ROS_DISCOVERY_SERVER

ros2 launch ugv_landing_sim bunker_uwb_viz.launch.py use_tag:=true
```

The four anchors only need power if they have already been configured and saved as anchors. They do not need to be connected to the laptop during tag localization.

## Useful Topics

```bash
ros2 topic echo /uwb/tag_raw
ros2 topic echo /uwb/tag_pose
ros2 topic echo /uwb/tag_markers
ros2 topic echo /uwb/anchor_markers
```

## Configuration

Anchor positions, serial ports, tag display settings, and center-distance display settings are stored here:

```text
src/ugv_landing_sim/config/bunker_uwb_params.yaml
```

Important tag settings:

```yaml
serial_mode: "tlv"
force_2d: true
tag_z_m: 0.0
xy_transform: "swap_invert_unit"
center_x_m: 0.5
center_y_m: 0.5
center_z_m: 0.0
```

## Simulation

Run the Gazebo Sim world:

```bash
ros2 launch ugv_landing_sim sim.launch.py
```

Run the UWB demo without real hardware:

```bash
ros2 launch ugv_landing_sim uwb_demo.launch.py
```

## Reference

This project was inspired by:

```text
https://github.com/Xpect8tions/Decawave-ros-data-sim
```

The referenced repository focuses on generating, parsing, and visualizing Decawave-style UWB data. This workspace adapts the idea for Bunker Pro UWB anchor visualization, DWM1001C tag reading, and future Gazebo/PX4 integration.
