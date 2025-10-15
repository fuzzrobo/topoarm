FROM ros:humble

# 必要なパッケージのインストール
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions

# 必要なライブラリのインストール
RUN apt install -y ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-gripper-controllers ros-humble-moveit ros-humble-gazebo-* ros-humble-joint-state-publisher-gui

# ワークスペースの読み込み
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc && \
    echo "source /usr/share/gazebo/setup.bash" >> /root/.bashrc
# ワークスペースの作成とパッケージのコピー
WORKDIR /ros2_ws

# エントリポイント
CMD ["/bin/bash"]