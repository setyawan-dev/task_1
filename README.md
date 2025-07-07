# 🤖 task_1

Pada proyek ini, akan diimplementasikan kontrol PID menggunakan platform Gazebo. Fokus simulasi adalah pada sistem jungkat-jungkit yang memiliki dua motor di sisi kiri dan kanan serta sensor gyroscope di bagian tengah.

Tugas utama adalah membuat program kontrol PID yang mampu menjaga keseimbangan jungkat-jungkit agar tetap stabil. Input dari sistem berupa data sudut dari gyroscope, sedangkan output berupa kecepatan dua motor di sisi kiri dan kanan.

## 📦 Instalasi Gazebo ROS Noetic

```bash
sudo apt update
sudo apt install \
  ros-noetic-joint-state-publisher-gui \
  ros-noetic-robot-state-publisher \
  ros-noetic-gazebo-ros-pkgs \
  ros-noetic-gazebo-ros-control \
  ros-noetic-xacro \
  ros-noetic-rviz \
  ros-noetic-controller-manager \
  ros-noetic-effort-controllers \
  ros-noetic-joint-state-controller \
  ros-noetic-transmission-interface \
  ros-noetic-control-toolbox \
  ros-noetic-ros-controllers \
  ros-noetic-tf \
  ros-noetic-tf2-ros
```

## 📦 Instalasi Gazebo ROS Melodic

```bash
sudo apt update
sudo apt install \
  ros-melodic-joint-state-publisher-gui \
  ros-melodic-robot-state-publisher \
  ros-melodic-gazebo-ros-pkgs \
  ros-melodic-gazebo-ros-control \
  ros-melodic-xacro \
  ros-melodic-rviz \
  ros-melodic-controller-manager \
  ros-melodic-effort-controllers \
  ros-melodic-joint-state-controller \
  ros-melodic-transmission-interface \
  ros-melodic-control-toolbox \
  ros-melodic-ros-controllers \
  ros-melodic-tf \
  ros-melodic-tf2-ros\
```

---

## 📥 Clone Repository

### 1. Clone repo ini ke `src`:

```bash
cd ~/catkin_ws/src
git clone https://github.com/setyawan-dev/task_1.git
```

---

## 🛠️ Build Workspace

```bash
cd ~/catkin_ws
catkin build
source devel/setup.bash
```

---

## 🚀 Menjalankan Simulasi

### 1. Menjalankan Gazebo:

```bash
roslaunch task_1 open_project.launch
```

### 2. Menjalankan Contoh Program:
```bash
rosrun task_1 vel_control_node
```

### 3. Lihat Data yang siap diunakan:
```bash
rostopic list
```
maka akan muncul:
```bash
/clock
/gazebo/link_states
/gazebo/model_states
/gazebo/parameter_descriptions
/gazebo/parameter_updates
/gazebo/performance_metrics
/gazebo/set_link_state
/gazebo/set_model_state
/gyro/data
/imu/data
/motor_kanan_controller/command
/motor_kanan_controller/pid/parameter_descriptions
/motor_kanan_controller/pid/parameter_updates
/motor_kanan_controller/state
/motor_kanan_joint/command
/motor_kiri_controller/command
/motor_kiri_controller/pid/parameter_descriptions
/motor_kiri_controller/pid/parameter_updates
/motor_kiri_controller/state
/motor_kiri_joint/command
/pivot_joint_controller/command
/rosout
/rosout_agg
/vel/cmd
```

pakai data ini!
```bash
/vel/cmd
/gyro/data
```
