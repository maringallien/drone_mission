# PX4 + ROS 2 Development Environment — Docker Edition

**DISCLAIMER: YOU DO NOT NEED TO DO THIS! THIS IS ONLY A RECORD OF MY WORK**

**Host:** Any Linux machine with a working GPU driver (the guide assumes Ubuntu-ish, X11 or Wayland)

PX4 SITL, ROS 2 Jazzy, Gazebo Harmonic, the Micro XRCE-DDS agent inside a single Docker container, with QGroundControl running on the host.

---

## Quick start — everyday use after install

Assumes Steps 1–4 are done and the `px4dev` container exists (i.e. you've run the `docker run` command at least once).

**7.1 — Allow display access and wake the container** (host terminal):

```bash
xhost +local:          # resets every login, so run it each session
docker start px4dev
```

**7.2 — Terminal 1: the PX4 ↔ ROS 2 bridge:**

```bash
docker exec -it px4dev bash
MicroXRCEAgent udp4 -p 8888
```

**7.3 — Terminal 2: the simulator** 

```bash
docker exec -it px4dev bash
make px4_sitl gz_x500
```

**7.4 — Host: QGroundControl**

```bash
./QGroundControl-x86_64.AppImage
```

**7.5 — Extra terminals as needed** (ROS 2 work, logs, etc.):

```bash
docker exec -it px4dev bash
ros2 topic list
```

**7.6 — Done for the day:**

```bash
docker stop px4dev
```

---

## Installation:

### 1. One-time host prep

Install Docker and let your user run it without sudo:

```bash
sudo apt install docker.io
sudo apt install docker-buildx
sudo usermod -aG docker marin
```

The group change needs a **full logout and login** of your desktop session — a new terminal isn't enough. Verify with `groups` (it should list `docker`).

```bash
groups
```

Allow local containers to use your display (needed once per login session):

```bash
xhost +local:
```

---

### 2. Create the Dockerfile

Make a project folder and put this in a file named `Dockerfile`:

```docker
# Download ROS2 jazzy full desktop (includes ROS2 jazzy and Gazebo harmonic)
FROM osrf/ros:jazzy-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
# Noble blocks system-wide pip installs, but that's fine in a Docker container
ENV PIP_BREAK_SYSTEM_PACKAGES=1

# Install basic tools into the image
RUN apt-get update && apt-get install -y git sudo wget curl nano mesa-utils

# Clone PX4
RUN git clone --recursive https://github.com/PX4/PX4-Autopilot.git /root/PX4-Autopilot

# Install PX4
# --no-nuttx skips the embedded ARM toolchain (not needed for SITL). Drop the flag if you'll build firmware for real flight controllers here.
RUN bash /root/PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx

# Install Micro XRCE-DDS Agent (ROS2 PX4 Bridge)
RUN git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /root/xrce-agent \
    && cmake -S /root/xrce-agent -B /root/xrce-agent/build \
    && cmake --build /root/xrce-agent/build -j"$(nproc)" \
    && cmake --install /root/xrce-agent/build \
    && ldconfig /usr/local/lib/

# ROS setup runs in every new shell
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc

WORKDIR /root/PX4-Autopilot
CMD ["bash"]
```

---

### 3. Build the image

```bash
docker build -t px4-jazzy .
```

The PX4 clone and dependency script make this a long first build (15–30 min depending on connection). It's cached afterwards.

---

### 4. Start the container

This setup targets the **NVIDIA GPU on a hybrid laptop** (Intel drives the display, NVIDIA renders on demand via PRIME). Two parts: install the container toolkit once, then run the container with the offload flags.

**4.1 — Install the NVIDIA Container Toolkit** on the host (see [NVIDIA's install guide](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) ):

```bash
# Download sign-in key
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg

# Download nvidia repo definitions
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

# Install toolkit itself
sudo apt update && sudo apt install -y nvidia-container-toolkit

# Edit docker's config file to register Nvidia runtime
sudo nvidia-ctk runtime configure --runtime=docker

# Restart docker so it picks up changes
sudo systemctl restart docker
```

**4.2 — Run the container.** On a hybrid laptop, `--gpus all` alone isn't enough: the display belongs to the integrated GPU, so Gazebo would silently keep rendering on Intel. This command keeps `--device /dev/dri` (the integrated GPU still handles the display connection) and adds the two PRIME render-offload variables so OpenGL actually runs on the NVIDIA card:

```bash
docker run -it --name px4dev 
--network=host --ipc=host 
-e DISPLAY 
-v /tmp/.X11-unix:/tmp/.X11-unix 
--device /dev/dri 
--gpus all -e NVIDIA_DRIVER_CAPABILITIES=all 
-e __NV_PRIME_RENDER_OFFLOAD=1 
-e __GLX_VENDOR_LIBRARY_NAME=nvidia 
px4-jazzy
```

Sanity-check that the right GPU is doing the work before going further:

```bash
# Inside the container — should name the NVIDIA card
glxinfo -B | grep renderer
```

- `-network=host` puts the container on the host's network, so all the usual UDP ports (8888 for XRCE, 14550/18570 for QGC) just work with no port mapping.

---

### 5. Run the stack

**Terminal 1** (the shell from `docker run`) — start the agent first, as before:

```bash
MicroXRCEAgent udp4 -p 8888
```

**Terminal 2** — open a second shell into the same container and start SITL:

```bash
# Inside regular terminal run this to open docker shell
docker exec -it px4dev bash

# Then run 
make px4_sitl gz_x500
```

The Gazebo window appears on your host display, now hardware-accelerated.

**Terminal 3** (optional) — ROS 2 side:

```bash
docker exec -it px4dev bash
ros2 topic list
```

---

### 6. QGroundControl on the host

Download the actual appImage here: https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl-x86_64.AppImage

Then run:

```bash
sudo apt install -y libfuse2 libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor0
chmod +x QGroundControl-x86_64.AppImage
```

Because the container uses host networking, QGC auto-detects the simulated vehicle exactly as in step 11.