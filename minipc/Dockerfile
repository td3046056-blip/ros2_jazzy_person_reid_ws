# Image ROS 2 Jazzy cho robot bam nguoi tren mini PC (i5-5200U, may that Ubuntu 20.04 + ROS 1 Noetic)
#
# Build:  docker build -t robot-jazzy:2 ~/robot_docker      (:1 = ban dau, chua co psmisc/fuser)
# Chay:   bash ~/robot_docker/run_container_auto.sh   (tu chay khi bat may, giu web luon song — buoc 8)
#         IMAGE=robot-jazzy:2 bash ~/robot_docker/run_container.sh   (chay tay, sleep infinity)
#
# torch (CPU) va onnxruntime KHONG cai o day: cai bang pip --user vao ~/.local (nam tren may that,
# giu lai khi tao lai container). NumPy giu ban 1.26 cua ROS (pip --no-deps).
FROM osrf/ros:jazzy-desktop

# Goi he thong: serial, camera, ghi bag, thu vien Python ma YOLOv5/DeepSORT import (pandas, seaborn...),
# va thu vien torch / onnxruntime can khi import (cai bang apt de khong dung NumPy cua ROS)
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-serial python3-scipy python3-opencv python3-numpy python3-yaml \
      python3-pip python3-rosdep python3-colcon-common-extensions \
      ros-jazzy-rosbag2 ros-jazzy-rosbag2-storage-mcap \
      v4l-utils usbutils sudo psmisc \
      python3-pandas python3-requests python3-seaborn python3-tqdm python3-matplotlib \
      python3-psutil python3-pil \
      python3-filelock python3-typing-extensions python3-sympy python3-networkx \
      python3-jinja2 python3-fsspec \
      python3-packaging python3-protobuf python3-flatbuffers python3-coloredlogs \
    && rm -rf /var/lib/apt/lists/*

# Doi user 'ubuntu' (uid 1000) thanh 'xiaoqiang' cho trung may that; them nhom thiet bi
# (so nhom dialout 20, video 44, plugdev 46, audio 29 trung voi may that)
RUN usermod -l xiaoqiang -d /home/xiaoqiang -c xiaoqiang ubuntu \
 && groupmod -n xiaoqiang ubuntu \
 && usermod -aG dialout,video,plugdev,audio xiaoqiang \
 && echo "xiaoqiang ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/xiaoqiang \
 && chmod 440 /etc/sudoers.d/xiaoqiang

# Moi shell trong container: nap ROS 2 Jazzy + workspace (neu da build).
# ~/.bashrc cua may that co dong '[ -f /.dockerenv ] && return' de khong nap ROS 1 Noetic o day.
RUN printf '%s\n' '' '# --- robot bam nguoi (ROS 2 Jazzy) ---' \
      'source /opt/ros/jazzy/setup.bash' \
      '_robot_ws=/home/xiaoqiang/ros2_ws/ros2_jazzy_person_reid_ws' \
      '[ -f "$_robot_ws/install/setup.bash" ] && source "$_robot_ws/install/setup.bash"' \
      'unset _robot_ws' \
    >> /etc/bash.bashrc

# Chi tim node ROS 2 trong may nay (laptop 10.42.0.1 cung co ROS 2 -> tranh 2 nguon /cmd_vel)
ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
USER xiaoqiang
ENV HOME=/home/xiaoqiang
WORKDIR /home/xiaoqiang/ros2_ws/ros2_jazzy_person_reid_ws
CMD ["sleep", "infinity"]
