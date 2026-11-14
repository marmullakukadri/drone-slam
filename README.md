# Drone SLAM – ROS 2 Jazzy + Gazebo Harmonic + RTAB-Map

Dron i simuluar me lidar 3D (16 rreze) dhe IMU në një ndërtesë 32×22 m (`worlds/shtepia.sdf`):
një korridor në mes, 3 dhoma në veri, 3 në jug dhe një sallë e madhe në lindje, me dyer 1.6 m.
RTAB-Map ndërton hartën 3D dhe vlerëson pozicionin vetëm nga lidar-i.
Ju e drejtoni dronin, ruani hartën (PCD + PGM/YAML) dhe pastaj e përdorni për lokalizim
dhe për ta dërguar dronin te destinacione me rrugë që i anashkalojnë muret.

Gjithçka ekzekutohet brenda kontejnerit `drone-slam-dev` (`~/drone-slam` → `/workspace`).

---

## Me një komandë: `./drone` (Terminal në Mac, nga dosja `drone-slam`)

```bash
./drone start dhoma1     # hartë e re: Gazebo + RViz + SLAM, hapet shfletuesi
./drone fluturo          # dritarja e tastierës në shfletues (disa taste njëkohësisht)
./drone ruaj             # ruan map.pcd + map.pgm
./drone ndal             # ndal simulimin dhe kontejnerin

./drone lokalizim         # harta shtepia, droni në fillim të korridorit (RViz: 2D Pose Estimate, pastaj 2D Goal Pose)
./drone demo | ngrihu | ulu | pamja | status | log | ndihmë
```

---

## Rrjedha kryesore me task-e në VS Code

`Cmd+Shift+P` → **Tasks: Run Task**:

| Hapi | Task | Çfarë bën |
|---|---|---|
| 1 | **1. Hartografim: Nis** | Pyet për emrin e hartës (p.sh. `dhoma1`). Hap shfletuesin me RViz (+Gazebo). |
| – | *fluturoni* | Me tastierë (shih më poshtë) **ose** me **2D Goal Pose** në RViz. Kaloni nëpër gjithë dhomën. |
| 2 | **2. Hartografim: Ruaj hartën** | Ruan hartën pa e ndalur simulimin (mund ta përsërisni). |
| 3 | **3. Hartografim: Ruaj dhe ndal** | Ruan hartën dhe ndal simulimin. RTAB-Map ruan `rtabmap.db`. |
| 4 | **4. Lokalizim: Nis** | Pyet për emrin e hartës dhe vendin ku niset droni. Ngarkon hartën pa krijuar hartë të re. |
| – | *në RViz* | **2D Pose Estimate**: klikoni ku është droni dhe tërhiqni në drejtimin ku shikon. **2D Goal Pose**: klikoni ku doni të shkojë. |
| – | **Ndal gjithçka (edhe kontejnerin)** | Ndal simulimin, VNC-në dhe kontejnerin Docker. |

### Tastiera (`task fluturo`)

Hapet dritarja e vogël **"Tastiera e dronit"** poshtë djathtas në shfletues (desktopi VNC).
Klikoni një herë mbi të, pastaj **mbani disa taste njëkohësisht**: p.sh. ↑ + `q` = para duke u
kthyer majtas, ↑ + ← = diagonale, ↑ + `u` = para duke u ngritur. Tastet e shtypura ndizen jeshil.

| Tasti | Lëvizja |
|---|---|
| ↑ / ↓ | para / mbrapa |
| ← / → | majtas / djathtas (anash) |
| `u` / `d` | lart / poshtë |
| `q` / `e` | rrotullim majtas / djathtas |
| `+` / `-` | më shpejt / më ngadalë |
| hapësirë | ndal |

Droni lëviz sa i mbani tastet dhe ndalet kur i lëshoni (ose kur klikoni jashtë dritares).
Krahët e **kuq** të dronit janë përpara.

Task-e të tjera: **Dron: Ngrihu / Ulu / Ndalo në vend**, **Teleop: Tastiera**,
**Info: Gabimi i SLAM-it**, **Sim: Ndal**, **Terminal: Hyr në kontejner**.
`Cmd+Shift+B` nis demon me mision automatik.

Pamja: **http://localhost:6080/vnc.html** (hapet vetë). Dritaret hapen të maksimizuara;
kaloni nga RViz te Gazebo nga shiriti poshtë.

### Çfarë ruhet te `maps/<emri>/`

| Skedari | Formati | Përdorimi |
|---|---|---|
| `map.pcd` | PCD ASCII (x y z), **pa tokë** | Harta 3D për t'u parë: CloudCompare, PCL, `pcl_viewer` |
| `map_full.pcd` | PCD ASCII (x y z), me tokë | Harta 3D e plotë (lokalizues 3D si NDT kanë nevojë për tokën) |
| `map.pgm` + `map.yaml` | Formati i Nav2 `map_server`, **pa tokë (i filtruar)** | Harta 2D e pastër: e bardhë = e lirë, e zezë = mure/objekte |
| `map_full.pgm` + `map_full.yaml` | Formati i Nav2 `map_server`, pa filtrim | Harta 2D e papërpunuar (pengesat 0.3–3 m mbi dysheme) |
| `rtabmap.db` | Databaza RTAB-Map | Lokalizimi në këtë projekt (`mode:=localization`) |
| `map_info.yaml` | YAML | Ku ishte origjina e hartës në botën e Gazebo |

Hartat bëhen në `shtepia.sdf`. Lokalizimi e hap vetë botën ku është bërë harta (shënohet te `map_info.yaml`).

### Lokalizim (`task lokalizim`)

Përdor gjithmonë hartën `maps/shtepia/`. Droni del në **fillim të korridorit** (x = -6.5, y = -0.6,
i kthyer nga korridori) dhe **initial pose i jepet RTAB-Map-it vetë** (`initial_pose.py`, pret derisa
harta të ngarkohet dhe kontrollon me TF që u pranua).

Në RViz ka dy drone: **"Droni (Gazebo)"** (cian, i tejdukshëm) = ku është vërtet, **"SLAM"**
(portokalli) = ku mendon SLAM-i se është. Duhet të jenë njëri mbi tjetrin.

1. **2D Goal Pose**: klikoni ku të shkojë dhe tërhiqni për drejtimin. Droni ngrihet vetë, gjen rrugën
   rreth mureve dhe nëpër dyer (vija portokalli) dhe qëndron pezull atje. Goal i ri sa herë të doni.
2. Vetëm nëse "SLAM" largohet nga "Droni (Gazebo)": **2D Pose Estimate** mbi dronin cian,
   tërhiqni në drejtimin ku shikon.

Destinacioni duhet të jetë të paktën 0.6 m larg mureve. Hartë/vend tjetër: `task lokalizim MAP=emri X=.. Y=.. YAW=..`.

---

## Me komanda (pa VS Code)

```bash
docker start drone-slam-dev
docker exec -it drone-slam-dev bash
# brenda:
cd /workspace && colcon build --symlink-install && source install/setup.bash
ros2 run drone_slam start_vnc.sh && export DISPLAY=:1

# 1) hartografim
ros2 launch drone_slam slam_sim.launch.py mode:=mapping map_name:=dhoma1
#    terminal tjetër: DISPLAY=:1 ros2 run drone_slam drone_teleop.py   (dritare në VNC)
# 2) ruaj (terminal tjetër), pastaj Ctrl+C te launch-i
ros2 run drone_slam map_saver.py --ros-args -p output_dir:=/workspace/maps/dhoma1
# 3) lokalizim (droni niset te (5,-4.5) i kthyer 90°)
ros2 launch drone_slam slam_sim.launch.py mode:=localization map_name:=dhoma1 spawn_x:=5 spawn_y:=-4.5 spawn_yaw:=1.5708

# komanda për dronin
ros2 topic pub --once /drone/command std_msgs/msg/String '{data: takeoff}'   # land | hover | mission
ros2 run drone_slam stop_sim.sh       # ndal simulimin
```

### Argumentet e launch-it

| Argumenti | Parazgjedhja | Përshkrimi |
|---|---|---|
| `mode` | `demo` | `demo` (mision automatik), `mapping`, `localization` |
| `map_name` | `demo` / `map` | Dosja te `maps/` |
| `spawn_x`, `spawn_y`, `spawn_yaw` | `0`, `0`, `0` | Ku shfaqet droni në Gazebo (yaw në radianë) |
| `gui` | `true` | Dritarja e Gazebo (`false` = vetëm RViz, më shpejt) |
| `rviz` | `true` | RViz2 |

---

## Si funksionon

```
Gazebo (shtepia + slam_drone) ─────/lidar/points, /imu, /odom_gt──► ros_gz_bridge
          ▲                                                              │
          │ /cmd_vel                                                     ▼
     drone_node ◄── /goal_pose, /drone/command, /cmd_vel_teleop    RTAB-Map
     (A* në hartën 2D, ndjek rrugën)  ◄── /cloud_map ────────────  icp_odometry → rtabmap
          │                                                        /odom, /cloud_map, TF
          └─► /planned_path, /planning_map, /mission/state
```

- **TF:** `map → odom → base_link → {lidar_link, imu_link}`, plus `map → world` (Gazebo).
- **drone_node:**
  - Gjendjet: `IDLE → TAKEOFF → GOTO | MISSION → LANDING`.
  - Tastiera ka gjithmonë përparësi.
  - Për destinacionet planifikon rrugë me A* mbi hartën 2D (pengesat fryhen 0.5 m).
  - Nëse droni është në tokë, ngrihet fillimisht.
- **Lokalizimi:** RTAB-Map në `Mem/IncrementalMemory=false`. Pa pozicion fillestar supozon origjinën e hartës. **2D Pose Estimate** (`/initialpose`) ia jep pozicionin e saktë, pastaj ICP-ja e rafinon kundrejt hartës.
- **slam_evaluator:**
  - Krahason SLAM-in me pozicionin e vërtetë.
  - Rezultatet ruhen te `results/*.csv`.
  - Statistikat rinisin me çdo pozicion fillestar të ri.

### Rezultatet e testeve
| Testi | Gabimi i SLAM-it |
|---|---|
| Hartografim, 6 destinacione rreth dhomës (pas ngritjes) | RMSE 8.5 cm, maks 17 cm |
| Lokalizim, nisje te (5,−4.5, 90°) + 2 destinacione | 2.7 cm në fund |

---

## Struktura

```
models/slam_drone/          dron (VelocityControl, lidar 3D, IMU, odometri e vërtetë)
worlds/shtepia.sdf          ndërtesa me dhoma (bota e paracaktuar, droni shfaqet nga launch-i)
worlds/slam_world.sdf       dhoma e vjetër 24×16 m (nuk përdoret më si parazgjedhje)
maps/<emri>/                hartat e ruajtura
results/                    CSV-të e vlerësimit
src/drone_slam/
  src/drone_node.cpp        kontrolluesi: mision, destinacione + A*, komanda, teleop
  scripts/map_saver.py      ruan map.pcd + map.pgm/yaml
  scripts/drone_teleop.py   dritarja e tastierës (disa taste njëkohësisht)
  scripts/sync_check.sh     (në Mac) rinis kontejnerin nëse Docker shërben skedarë të vjetër
  scripts/slam_evaluator.py gabimi i SLAM-it
  scripts/start_vnc.sh      desktop virtual + noVNC (porti 6080)
  scripts/stop_sim.sh       ndal simulimin (RTAB-Map ruan databazën)
  launch/slam_sim.launch.py
  config/                   gz_bridge.yaml, rtabmap.yaml, mission.yaml
  rviz/slam.rviz
.vscode/tasks.json          task-et
```

## Shënime
- **Droni është kinematik:** shpejtësia vendoset drejtpërdrejt dhe graviteti është i çaktivizuar. Nuk ka PX4.
- **Kontrolli i fluturimit** përdor pozicionin e vërtetë nga Gazebo, që gabimet e SLAM-it të mos e prishin fluturimin.
- **Ngjitjet ndalen ngadalë (0.25 m/s):** muret vertikale i japin ICP-së pak informacion për lartësinë.
- **"Live Lidar" në RViz është i çaktivizuar** si parazgjedhje. Pikat e lidar-it hidhen shpesh sepse TF-ja e odometrisë vjen me vonesë. Aktivizojeni me një klik nëse ju duhet.
- **Docker Desktop në Mac ndonjëherë shërben version të vjetër** të një skedari të sapondryshuar. Task-et e kontrollojnë dhe e rinisin kontejnerin vetë. Me dorë: `bash src/drone_slam/scripts/sync_check.sh` ose `docker restart drone-slam-dev`.
