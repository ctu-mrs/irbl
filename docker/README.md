# irbl SITL in docker (smart-deployment image)

The docker version of `../tmux/px4_sim_*/start.sh`: the scenario's own tmux session (`session.yml`, unchanged,
same windows / panes / commands), with tmux on the PC and every pane a zsh in the `sim` container. One tmux,
nothing nested. Same idea as `payload_control/docker`: the host tmux is the control room, the panes
`docker exec` into the containers.

```bash
colcon_workspace_builder/build.sh   # once, and after changing ros_packages/: builds the workspace against the image
./tmux.sh ugv_uav_warehouse_dynamic # scenario = a file in sim/scenarios/   (-d detached, -r rosbag, -n keep workspace)
tmux -L whdyn5p a                   # re-attach later: the socket_name of the scenario's session.yml
./kill.sh                           # stop everything
```

The world of `1_uav_3dlidar`: `START_ARGS=city ./tmux.sh 1_uav_3dlidar`. Host variables listed in
`FORWARD_ENV` (`sim/stack.env`) reach the panes, e.g. `PLANNER=local ./tmux.sh ugv_uav_warehouse_dynamic`.

Run one session at a time, and not together with the same session on the host: they would share gz-transport,
the uXRCE-DDS ports, the ROS domain and the tmux socket. `tmux.sh` refuses to start while the host session runs.

## How it works

- `tmux.sh` runs `sim/up.sh` (containers + workspace volume), then tmuxinator on a copy of the scenario's
  `session.yml` (`sim/.session.yml`: absolute root, config `sim/.tmux.conf`). That config sources the
  scenario's `tmux.conf` (or `~/.tmux.conf`) and makes `sim/pane.sh` the shell of every pane:
  `docker exec -it -w <pane dir> irbl_sim-sim-1 zsh`. Two host windows are added at the end:
  `lazydocker` and `workspace` (`colcon_workspace_builder/`).
- The host `tmux/` path is a symlink in the container to the copy in the workspace volume, so the paths
  tmuxinator sends work there too.
- The session keeps the scenario's session / socket names, and the host's `/tmp/tmux-<uid>` is mounted in
  the container: the sessions' scripts that type into the PX4 shells (`set_px4_params.sh`,
  `fix_arming_healthcheck.sh`, `start_lazypx4_link.sh`) reach the host tmux server.
- `tmux/<dir>/scans` is bind-mounted, so scan reports land in the repo (written as root).
- `PYTHONPATH=/usr/lib/python3/dist-packages`: the image's MAVProxy pip install put opencv-python 5 (no
  WeChat QR decoder) and numpy 2 in `/root/.local`; this puts the system OpenCV 4.6 (contrib) first.
  The real fix belongs in the base image's Dockerfile.

## Layout

| path                        | what it is                                                                  |
|-----------------------------|-----------------------------------------------------------------------------|
| `colcon_workspace_builder/` | builds `src/` (`irbl/ros_packages` links to `../../ros_packages`; `px4_msgs` / `px4_ros_com` are fetched by `build.sh` with the messages of the image's PX4) in `smart-deployment:latest`; the result lands in `cache/`, and `up.sh` copies it in |
| `sim/compose.yaml`          | the services (below)                                                        |
| `sim/stack.env`             | settings of every scenario: project name, image, `GPU`, `HEADLESS`, `WEB_VIZ`, `FORWARD_ENV` |
| `sim/scenarios/<s>.env`     | the scenario: its `../tmux` folder, ROS domain                              |
| `sim/up.sh`, `sim/down.sh`  | start / stop the containers (`up.sh <s> -r` also records a rosbag, `-n` keeps the workspace volume) |
| `sim/pane.sh`               | the shell of every pane: zsh in the `sim` container                         |
| `sim/lichtblick_layout.json`| default layout of the browser viewer (`WEB_VIZ=1`)                          |
| `tmux.sh`, `kill.sh`        | start / stop scenario session + containers (`tmux.conf`: optional config)    |

## Image and workspace

No image of our own: every container runs `smart-deployment:latest` (built by
`smart-deployment/docker/lazydocker/base_image/build.sh`) as is. It has ROS 2 jazzy, MicroXRCEAgent, Gazebo
Harmonic, PX4 v1.17 `px4_sitl_default` built with Gazebo (`PX4_DIR=/root/px4/PX4-Autopilot`), lazypx4,
tmuxinator, `ros_gz_bridge`, RViz, and (step 7 of its Dockerfile) what irbl needs on top: zsh, Xvfb,
foxglove_bridge, PCL, OctoMap, OpenCV. New system dependencies go in that image's Dockerfile.

The image has upstream PX4 and upstream gz models: sessions that use the models of the PX4-gazebo-models
fork (`x500_lidar_3d`, `cylinder_forest`) don't work in it.

`colcon_workspace_builder/build.sh` builds the packages in the image at `/etc/docker/colcon_workspace`
(Release, no tests, `colcon_defaults.yaml`). It keeps `build/` and `install/` in `cache/`, so rebuilds are
incremental (`clean_cache.sh` starts over). `sim/up.sh` copies `install/` + `src/` into the
`colcon_workspace` volume, then the live `../tmux` folder at `src/irbl/tmux` (so the sessions'
`source ../../../../install/setup.zsh` finds the workspace). The image's `/entrypoint.sh` sources it.
A code change therefore takes `build.sh` + `./tmux.sh <scenario>`. A change in `../tmux` only takes
`./tmux.sh <scenario>` (or `sim/up.sh <scenario> -n`).

## Containers (`sim/compose.yaml`; all use host networking and host IPC)

| service   | what                                                                                      |
|-----------|-------------------------------------------------------------------------------------------|
| `sim`     | idle; every pane of the scenario's session is a shell in it (PX4, Gazebo, bridges, controllers, rviz2 on the host display, ...) |
| `viz`     | Lichtblick on <http://localhost:8080>, only with `WEB_VIZ=1`                              |
| `rosbag`  | `ros2 bag record -a` to `~/docker/irbl_sim/bag_files`, only with `-r`                     |
| `init`    | primes the `colcon_workspace` volume                                                      |

Outside the session: `docker compose -p irbl_sim ...` in `sim/`. From the host,
`ROS_DOMAIN_ID=<scenario's> ros2 topic list` also shows the topics, since the network is shared.

**Gazebo window:** `HEADLESS` in `sim/stack.env`. Empty (default) leaves it to the scenario, as on the host.

**Browser instead of X11:** `WEB_VIZ=1` in `sim/stack.env`. Gazebo runs on an Xvfb display, the sessions skip
rviz2, and Lichtblick shows the topics (streamed by each session's `foxglove` window) on
<http://localhost:8080/?ds=foxglove-websocket&ds.url=ws://localhost:8765>.

**GPU / rendering:** `GPU` in `sim/stack.env` picks the compose override (`COMPOSE_FILE`):

| `GPU=`     | rendering                                   | when                                                    |
|------------|---------------------------------------------|---------------------------------------------------------|
| `dri`      | host GPU through `/dev/dri` (Mesa). `up.sh` writes `compose.gpu-dri.yaml` with only the non-NVIDIA nodes (the NVIDIA ones make gz's GPU lidar segfault on hybrid laptops) | Intel / AMD graphics (default) |
| `software` | CPU, Mesa llvmpipe (OpenGL 4.5), slower     | any PC; rviz2 / Gazebo say `OpenGL 3.3 is not supported` (NVIDIA host) |
| `nvidia`   | NVIDIA GPU (`sim/compose.gpu-nvidia.yaml`)  | NVIDIA proprietary driver + `nvidia-container-toolkit` on the host |

After changing it: `./kill.sh && ./tmux.sh <scenario>`.
