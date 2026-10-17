# Komutlar

Projeyi derlemek, çalıştırmak, test etmek ve hata ayıklamak için sık kullanılan komutlar.
Tüm komutlar workspace kökünden çalıştırılır:

```bash
cd "/home/samsapulee/Desktop/School/Mobile Robotics/Mobile-Robotics-Projesi"
```

> Klasör yolunda boşluk var (`Mobile Robotics`): yolu her zaman tırnak içinde yaz.

## 0. Klonlama ve kurulum

```bash
git clone https://github.com/AbdullahTahaUstunsoy/Mobile-Robotics-Projesi.git ~/Mobile-Robotics-Projesi
cd ~/Mobile-Robotics-Projesi
sudo apt install -y python3-colcon-common-extensions ros-lyrical-ros-gz-sim ros-lyrical-ros-gz-bridge \
  ros-lyrical-robot-state-publisher ros-lyrical-xacro
rosdep update && rosdep install --from-paths src --ignore-src -r -y   # eksik bağımlılıklar
```

Ayrıntılar ve private repo erişimi için `README.md` → *Kurulum*.

## 1. Derleme

```bash
source /opt/ros/lyrical/setup.bash              # ROS 2 (bashrc'de yoksa)
colcon build --symlink-install                  # tüm paketler
colcon build --symlink-install --packages-select midterm_behavior   # tek paket
source install/setup.bash                       # her yeni terminalde bir kez
```

- `--symlink-install` ile Python, launch, xacro, world ve yaml değişiklikleri **yeniden derlemeden** geçerli olur.
  Yeni dosya, yeni node (`setup.py` entry point) veya `CMakeLists.txt` / `package.xml` değişikliğinde tekrar derle.
- `Package 'midterm_bringup' not found` → o terminalde `source install/setup.bash` yapılmamış.
- Temiz derleme: `rm -rf build install log && colcon build --symlink-install`

## 2. Simülasyon

```bash
ros2 launch midterm_bringup sim.launch.py       # Gazebo + robot + bridge
```

| Argüman | Varsayılan | Açıklama |
|---------|------------|----------|
| `headless` | `false` | Gazebo GUI'siz (sadece sunucu) |
| `x`, `y`, `yaw` | `-0.5`, `0.65`, `0.0` | Robotun başlangıç konumu (1. koridor, orta sıra sağda) |
| `world` | `greenhouse.sdf` | Dünya dosyası |

Sadece dünyayı açmak için: `gz sim -r src/midterm_bringup/worlds/greenhouse.sdf`

## 3. Robotu sürmek

```bash
ros2 run midterm_behavior wasd_teleop          # w/a/s/d basılı tut, q/e yay, PgUp/PgDn hız, boşluk dur
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}"
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"                 # dur
```

Robotta sensör yok; yayınlanan topic'ler: `/cmd_vel`, `/odom`, `/tf`, `/tf_static`, `/joint_states`, `/clock`, `/robot_description`.

```bash
ros2 topic list
ros2 topic echo /odom --once
```

## 4. Rapor için görseller

```bash
ros2 run rqt_graph rqt_graph                          # node/topic grafiği
ros2 run tf2_tools view_frames                        # TF ağacı → frames_*.pdf (gitignore'da)
ros2 run tf2_ros tf2_echo odom base_footprint         # iki frame arası dönüşüm
```

## 5. Kontroller

```bash
# Xacro / URDF kontrolü
xacro src/midterm_description/urdf/greenhouse_bot.urdf.xacro > /tmp/bot.urdf && check_urdf /tmp/bot.urdf
gz sdf -k src/midterm_bringup/worlds/greenhouse.sdf   # dünya dosyası geçerli mi
```

### Arayüzsüz simülasyon testi

Açık bir simülasyonla karışmaması için farklı bir `ROS_DOMAIN_ID` kullan:

```bash
export ROS_DOMAIN_ID=42
ros2 launch midterm_bringup sim.launch.py headless:=true
gz model -m greenhouse_bot -p        # Gazebo'daki gerçek konum (ground truth): x y z / roll pitch yaw
gz topic -l                           # Gazebo tarafındaki topic'ler (bridge'den önce)
```

## 6. Sorun giderme

```bash
# Gazebo kapandı ama sunucu arkada kaldıysa (eski dünya açılıyor, simülasyon yavaş)
pgrep -af gz
pkill -f gz-sim-main
```

- `libEGL warning: ... dri2 screen` uyarıları headless çalışmada normal.
- Robot hareket etmiyorsa: `ros2 topic info /cmd_vel` ile publisher var mı, `gz topic -e -t /cmd_vel` ile Gazebo'ya ulaşıyor mu kontrol et.
- Teleop tuşlara tepki vermiyorsa: `wasd_teleop`'un çalıştığı terminal odakta olmalı.

## 7. Git

```bash
git status
git pull --rebase
git add -A && git commit -m "Kısa açıklama"
git push
git log --oneline -10
```

- Repo: https://github.com/AbdullahTahaUstunsoy/Mobile-Robotics-Projesi
- `build/`, `install/`, `log/`, `frames*.pdf` gitignore'da.
