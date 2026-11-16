# px4_sim_ugv_uav_warehouse_dynamic

One UGV + UAV pair from `../px4_sim_5ugv_5uav_warehouse_light` (same vehicle models, depth cameras, ArUco precision
landing, light settings: read its README), in a warehouse that **moves** and has **codes on the shelves**. The UAV
reads the codes with two lateral cameras, and every shelf ends with a report. `./generate.py --pairs 3` gives more
pairs (up to 5).

```bash
colcon build --packages-select gz_greenhouse_dynamics rbl_mission   # once, from the workspace root (forklift plugin, scanner)
./start.sh            # headless gz (HEADLESS=0 ./start.sh: gz GUI), rviz (WEB_VIZ=1: foxglove)
# `deploy` window: wait for 'Arming' in px4_bridge (+ ~30 s), then arrow-up + Enter
./deploy.sh pair 1 3 2 left       # = row 3 column 2, left shelf (also: 6 3 right, 4 2 right in the history)
./deploy.sh pair 1 5 3 right      # aisle 5: operator_2 walks there
# `scan` window: when the shelf is done
#   REPORT A3-S2-L: locations 19/20, cartons 18/24, ... -> scans/<run>/uav1_A3-S2-L_<time>/report.html
./kill.sh
```

## Shelf codes

Every rack face that looks into an aisle carries QR labels (`generate.py`, ground truth in `worlds/shelf_codes.json`):

| label | where | code |
|---|---|---|
| location (1232) | front edge of every bay × level (4 levels: label centres at 0.27 / 1.47 / 2.67 / 3.87 m) | `LOC:A<aisle>-S<segment>-<L\|R>-B<bay>-L<level>`, e.g. `LOC:A4-S2-L-B03-L2` |
| carton (1263 on 726 cartons) | both faces of every carton on the shelves | `CTN-<id>/SKU-<sku>`, e.g. `CTN-0401/SKU-76605` |

The face naming matches `deploy.sh`: aisle = row, segment = column, `L` = the left (+y) shelf of that aisle. Bays are
numbered along +x and levels from the floor. Each label is a 16 cm plate with a version-1 QR code (21 × 6 mm modules)
and its text. All labels of a rack row share one texture atlas and one mesh (`models/shelf_labels`), so they add
8 visuals instead of 2495.

**Every label can be read.** The shared warehouse generator lets the 1–2 cartons of a bay intersect, which put
about 10% of the carton labels (131) inside another carton. `generate.py` therefore writes this session's own
`worlds/warehouse.sdf` with those cartons packed side by side (`fix_cartons`, 376 cartons moved; visual and
collision boxes together, so the planners' map matches). Generation then checks every label twice and fails if
one doesn't pass:
- it isn't inside or behind a carton;
- its whole QR code is in the lateral camera's view, and not hidden by a shelf's front edge or another carton, from
  at least one coverage pass. The pass is flown at its height and 0.15 m above or below it (`tolerance_z`), 1.2 m
  from the face.

Carton labels sit 0.32 m above the carton's bottom on every level. Lower, a top-level carton standing back in the rack
had its label hidden by the shelf's front edge when seen from the top pass.

## Lateral cameras and the scanner

`uav1_x500_scan` = the light session's UAV + `left_cam` / `right_cam`: 1600×1200 grayscale, 80° FOV, 6 Hz, 10 cm
to each side of the body, looking straight sideways and mounted in **portrait** (rolled 90° about the view axis).
At the 1.2 m coverage standoff that gives ±1.0 m of rack vertically, ±0.8 m along it, and about 4 px per QR module.
The generated `config/warehouse_map.yaml` changes three things:
- **Pass heights 0.8 / 2.0 / 3.4 m**, so that all label levels and the top cartons' labels (about 4.1 m) are in view.
- **Tighter arrival tolerances:** pass ends 0.5 m, pass-through goals 0.4 m (were 0.6 / 1.0).
- **Height tolerance `mission.tolerance_z: 0.15`** (new in warehouse_inspection_node, off by default): a coverage goal
  only counts as reached within 0.15 m of its height. Without it the climb to a level stopped counting 0.4 m short, and
  the whole pass stayed that low (camera at most 3.1 m on the 3.4 m pass, so the top cartons were missed). With it the
  camera flies 0.8–3.4 m.

`shelf_scanner.py` (rbl_mission, `scan` window) follows `/uav1/warehouse_inspection/state`:

- **COVERAGE of a shelf:** a scan starts. Only the camera facing the shelf is on, picked from TF at 2 Hz (the UAV
  turns around between passes, so it alternates left / right).
- **Camera source:** the cameras are read **straight from gz-transport** and subscribed only while scanning, so gz
  renders them only then. There is no ros_gz_bridge for them on purpose: even with `lazy:=true` a bridge keeps its gz
  subscription, and gz then renders the camera all the time. With 5 pairs, 10 such cameras took the idle RTF to 0.04.
  This also applies to the light session's "lazy" down-camera bridges.
- **Decoding:** OpenCV's WeChat QR decoder with its CNN detector + super-resolution models (`rbl_mission/models/wechat_qrcode`,
  Apache-2.0) reads a full frame in 5–15 ms and returns the codes' corners. Without the models it falls back to tiled
  decoding, with positions only where `cv2.QRCodeDetector` finds the corners.
- **Positions:** solvePnP on the 4 corners and the known code size, placed in the world frame with TF at the image stamp.
  The nearest location label of the face then gives the bay / level a carton was read in. A carton found in another
  bay / level than the manifest says is reported as **misplaced**.
- **DONE / ABORTED:** the report is written.

Output in `$SCAN_DATA_DIR/$SCAN_RUN_ID/` (default `scans/<start time>/` in this session folder (git-ignored), one folder per `./start.sh`):

| file | content |
|---|---|
| `uav1_A<a>-S<s>-<L\|R>_<time>/report.html` | KPIs, bay × level grid (location read ✓/✗, cartons per cell), missing codes, thumbnail of every code read |
| `.../report.md`, `.../report.json` | the same as text / data (the JSON also lists codes of other shelves seen through the racks) |
| `.../detections.csv` | every single read: sim time, camera, code, size in px, range, estimated x y z, camera x y z |
| `inventory.csv` | one line per code read, every shelf of the run: carton, SKU, bay / level from the manifest and as read, position, error |
| `index.md` | one line per shelf report |

The scanner also publishes `/uav1/shelf_scanner/report` (JSON summary, latched) and `/uav1/shelf_scanner/state`.

**In rviz** (`ugv_uav_pairs.rviz`, generated):
- **`[1] QR scan (lateral camera)`** (`/uav1/shelf_scanner/debug_image`): the frame being decoded, turned upright, with
  each code boxed and labelled. Green = location label of this shelf, orange = carton, grey = another shelf seen
  through the rack. It's published only while scanning and only while something subscribes. It's half resolution
  (600×800); set `debug_scale` / `debug_rotation` on the scanner to change that.
- **`[1] QR codes read`** (`/uav1/shelf_scanner/codes`, latched): every code read, as a small plate with its text at
  its estimated position in the map. After the report, the missing codes show in red at their true position. They
  stay until the next shelf.

### Test runs (2026-11-15, headless, 1 pair)

| shelf | locations | cartons | mean / max position error | coverage | note |
|---|---|---|---|---|---|
| `3 2 left` (A3-S2-L) | 19 / 20 | 18 / 24 | 0.13 / 0.32 m | 15/15, 141 s | top pass still at 3.2 m |
| `5 3 right` (A5-S3-R) | 18 / 20 | 18 / 20 | 0.20 / 0.37 m | 13/15, 207 s | operator_2 in the aisle; landing and home OK |
| `4 1 right` (A4-S1-R) | 23 / 24 | 24 / 26 | | 18/18, 50 s | after fix_cartons + `tolerance_z` (camera 0.79–3.37 m); the 2 misses were top-level labels behind the shelf's front edge, since moved up |

Idle RTF with 1 pair is 1.0.

## Dynamic scene

All of these are visual only: the depth cameras (so rbl_controller) and the lateral cameras see them, but physics
doesn't, and they're not in the planners' prior map.

- **Operators** (`OPERATORS` in `generate.py`, gz actors) walk the aisle centrelines and stop facing the racks:
  - aisle 2 (y = −8.4) in segments 1–2
  - aisle 5 (y = 4.2) in segments 3–4
  - aisle 3 (y = −4.2) in segments 3–4
  - the main aisle in segments 1–2

  They stay out of the x = 0 cross-aisle, where the pair parks. Mesh: `models/greenhouse_operator`, linked from the
  greenhouse session.
- **Forklifts** (`FORKLIFTS`) carry a pallet up and down the x = −16.5 / +16.5 cross-aisles, from wall strip to
  wall strip, waiting 6–8 s at each end. They're driven by `gz_greenhouse_dynamics::PathFollower`
  (`ros_packages/gz_greenhouse_dynamics`), a model plugin that moves a static model along a looping path.

## Generated files

`generate.py` imports the light session's generator, points it at this folder (session `ugv_uav_warehouse_dynamic`,
socket `whdyn5p`, ROS domain 46, world `warehouse_dynamic`, models `ugv<k>_carrier_dyn` / `uav<k>_x500_scan`), runs it,
and then adds the cameras, labels, dynamic objects and scanner panes. Edit `generate.py`, or the light generator for
what both share, and re-run it. Don't edit `session.yml`, `env.sh`, `models/`, `worlds/warehouse_dynamic.sdf`,
`worlds/shelf_codes.json`, `config/warehouse_map.yaml`, `config/uav_sensors.yaml` or `ugv_uav_pairs.rviz`.

Only one warehouse session can run at a time, because they share the PX4 / gz / MicroXRCEAgent ports.
