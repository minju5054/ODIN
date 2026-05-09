# ODIN-RESCUE 
**On-device Detection and Inference for Navigation in Disaster Rescue**

ODIN-RESCUE는 **ROS2 Humble**과 **Gazebo** 환경을 활용한 재난 구조 시뮬레이션 시스템입니다. 두 대의 정찰 로봇이 자율 주행 및 SLAM을 통해 지도를 제작하고, 시각 언어 모델(Qwen-VL)이 구조 로봇의 최적 구조 목표점을 결정하는 지능형 협업 아키텍처를 가집니다.

---

##  System Architecture
본 시스템은 정찰(Scouting), 통합 및 판단(Coordination), 그리고 구조(Rescue)의 3단계 프로세스로 운영됩니다.

### 1. 정찰 로봇 (Scout Robots: `robot_1`, `robot_2`)
* **자율 탐색**: 미리 정의된 Waypoint 경로를 순찰하며, LiDAR 기반 Safety Rule을 통해 장애물을 회피합니다.
* **SLAM & 탐지**: LiDAR SLAM으로 지도를 생성하고, RGB 카메라로 ArUco Marker(ID 0, 구조 대상자)를 탐지합니다.
* **이벤트 발행**: 대상자 발견 시 해당 위치 정보가 포함된 `victim_event`를 JSON 형식으로 발행합니다.

### 2. 통합 제어기 (Coordinator)
* **지도 병합**: 정찰 로봇들이 각자 생성한 개별 지도를 `merged_map`으로 통합합니다.
* **데이터 시각화**: 통합된 지도를 PNG 이미지로 변환하고, 로봇 위치와 후보 Waypoint를 표시하여 Qwen-VL에게 전달합니다.
* **안전 검증**: Qwen이 선택한 Waypoint가 실제 이동 가능한 공간(Free Space)인지, 충분한 안전 거리(Clearance)를 확보했는지 검증합니다.

### 3. 구조 로봇 (Rescue Responder: `robot_3`)
* **대기 및 출동**: 안전 구역(Safe Zone)에서 대기하다가 , Coordinator가 검증한 목표 Pose를 수신하면 출동합니다.
* **경로 계획**: **Nav2**를 사용하여 목적지까지의 충돌 없는 최적 경로를 계획하고 주행합니다.

---

## 🎨 Map Visualization & Rules
Coordinator는 Qwen 모델의 판단을 돕기 위해 다음과 같은 색상 규칙으로 지도 이미지를 생성합니다:

| 항목 | 색상/기호 | 설명 |
| :--- | :--- | :--- |
| **이동 가능 공간** | White | 로봇이 다닐 수 있는 Free Space  |
| **장애물** | Black | 벽이나 물체 등 점유된 공간  |
| **미탐사 영역** | Gray | 아직 지도가 그려지지 않은 구간  |
| **구조 로봇** | Blue Circle | `robot_3`의 현재 위치  |
| **구조 대상자** | Red Circle | 발견된 ArUco Marker의 위치  |
| **후보 지점** | Green Number | Qwen이 선택할 수 있는 후보 Waypoint 목록  |

---

## 🛠️ Environment & Tech Stack
* **OS**: Ubuntu 22.04 LTS 
* **ROS**: ROS2 Humble 
* **Simulator**: Gazebo Classic, RViz2 
* **Robot**: TurtleBot3 (Scout x2, Rescue x1)
* **Key Tools**: OpenCV (ArUco Detection), Nav2 (Path Planning)

---

## 📂 Package Structure
본 프로젝트는 다음과 같은 Python 노드들로 구성됩니다:

* `scout_patrol_node.py`: 정찰 로봇의 자율 주행 및 안전 로직 제어.
* `simple_map_merge_node.py`: 다중 로봇 지도를 하나로 통합.
* `aruco_victim_detector_node.py`: RGB 카메라 기반 구조 대상자 인식.
* `coordinator_node.py`: 지도 이미지 생성, Qwen 인터페이스, Waypoint 검증 총괄.
* `qwen_prompt_builder.py`: 시각 지능 모델(Qwen-VL)을 위한 프롬프트 생성.

---

## 🚀 How to Run

### 1. Workspace 설정
```bash
mkdir -p ~/odin_ws/src
cd ~/odin_ws/src
# 본 레포지토리 클론
colcon build --symlink-install
source install/setup.bash
