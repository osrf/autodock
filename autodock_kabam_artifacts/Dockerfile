FROM ros:noetic

SHELL ["bash", "-c"]

# add install deps
RUN apt-get update && apt-get install \
  git -y 

# install osrf autodock
RUN mkdir -p /root/catkin_ws/src && cd /root/catkin_ws/src  && \
  git clone https://github.com/cognicept-admin/autodock

RUN rm -rf /root/catkin_ws/src/autodock/autodock_sim

# copy artifacts to autodock_kabam
COPY . /root/catkin_ws/src/autodock/autodock_kabam

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
