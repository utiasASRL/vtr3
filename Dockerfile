# FROM nvidia/cuda:11.8.0-devel-ubuntu22.04
FROM utiasasrl/vtr3:cpu-latest

CMD ["/bin/bash"]

# Clone libtorrent RC_2_0
WORKDIR $HOMEDIR
RUN git clone --recursive https://github.com/arvidn/libtorrent.git -b RC_2_0

WORKDIR $HOMEDIR/libtorrent/build
RUN cmake -DCMAKE_BUILD_TYPE=Release \
      -DTORRENT_USE_ASSERTS=ON \
      -Dpython-bindings=ON \
      -Dencryption=ON \
      -Ddht=ON \
      -Dcrypto=openssl \
      -Dextensions=ON \
      -DPYTHON_EXECUTABLE=/usr/bin/python3.10 \
      -DCMAKE_CXX_FLAGS="-DTORRENT_USE_OPENSSL=1" \
      .. && \
    make -j$(nproc)


# 4. Verification Step inside the container
RUN nm -D bindings/python/libtorrent*.so | grep -E "dht_put_item|ed25519"
RUN cp $HOMEDIR/libtorrent/build/bindings/python/libtorrent*.so /usr/local/lib/python3.10/dist-packages/libtorrent.so

# Install Zenoh router (zenohd)
RUN echo "deb [trusted=yes] https://download.eclipse.org/zenoh/debian-repo /" | \
    tee /etc/apt/sources.list.d/zenoh.list > /dev/null && \
    apt-get update && \
    printf '#!/bin/sh\nexit 0' > /usr/bin/systemctl && \
    chmod +x /usr/bin/systemctl && \
    apt-get install -y zenohd

RUN pip3 install pynacl eclipse-zenoh msgpack zenoh-cli pprintpp pandas
RUN apt install -q -y ros-humble-rmw-zenoh-cpp
RUN pip3 install --upgrade matplotlib

WORKDIR $HOMEDIR
# 1. Switch back to your custom user context
USER ${USERNAME}

COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]

# 2. Safely write to their specific home directory bashrc
RUN echo "alias build_ui='npm --prefix ${VTRUI} install ${VTRUI}; npm --prefix ${VTRUI} run build'" >> ${HOMEDIR}/.bashrc
RUN echo "alias build_vtr='source /opt/ros/humble/setup.bash; cd ${VTRSRC}/main; colcon build --symlink-install'" >> ${HOMEDIR}/.bashrc
RUN echo "source /opt/ros/humble/setup.bash; export RMW_IMPLEMENTATION=rmw_zenoh_cpp" >> ${HOMEDIR}/.bashrc

# docker run -it --name vtr3   --privileged   --network=myNetwork -p 5200-5204:5200-5204  --ipc=host   -e USER_ID=$(id -u) -e GROUP_ID=$(id -g) -e USER_NAME=$(id -un) -e RMW_IMPLEMENTATION=rmw_zenoh_cpp -e ROBOT_NAME=mr_green -e ROBOT_ID=15 -e DISPLAY=$DISPLAY   -v /tmp/.X11-unix:/tmp/.X11-unix   -v ${VTRROOT}:${VTRROOT}:rw   -v /dev:/dev   vtr3
