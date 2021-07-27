FROM ros:noetic

# only copy autodock_core and autodock_examples repo
COPY autodock_core /root/catkin_ws/src/autodock_core
COPY autodock_examples /root/catkin_ws/src/autodock_examples

SHELL ["bash", "-c"]

# add install deps
RUN apt-get update && apt-get install \
  git -y 

# install ros fiducial repo
RUN cd /root/catkin_ws/src  && \
  git clone https://github.com/UbiquityRobotics/fiducials.git

# install dependencies
# Note: force return as true, as fiducial has some non python3 deps
# https://github.com/UbiquityRobotics/fiducials/issues/252
RUN  cd /root/catkin_ws && \
  apt-get update && \
  rosdep update && \
  rosdep install --from-paths src --ignore-src -yr   || true

# build repo
RUN . /ros_entrypoint.sh && cd /root/catkin_ws && \
  catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release && \
  sed -i '$isource "/root/catkin_ws/devel/setup.bash"' /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
