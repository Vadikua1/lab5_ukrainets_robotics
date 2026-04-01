# 🤖 Lab 5: Obstacle Avoidance


## 🛠 Prerequisites

- Docker Desktop
- WSL
- Native Ubuntu
- DualBoot

---

## 🚀 Getting Started

### 1️⃣ Environment Setup

Clone the repository and navigate to the project folder:

```bash
git clone https://github.com/Vadikua1/lab5_ukrainets_robotics.git
cd lab2_ukrainets_robotics
```
### 2️⃣ Docker Image Build & Run

Build the Docker image
(Required for the first run or after Dockerfile changes):
```bash
./scripts/cmd build
```
### Start the container:
```bash
./scripts/cmd run
```

### 3️⃣  Workspace Build

Inside the container, compile your package:
```bash
cd /opt/ws
colcon build --packages-select lab3 lab5
source install/setup.bash
```
## 🎮 Running the Lab
### ▶️  Main Launch

Launch the simulation environment:
```bash
ros2 launch lab5 obstacle_avoidance_bringup.launch.py
```

