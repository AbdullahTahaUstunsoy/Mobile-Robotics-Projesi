# Mobile-Robotics-Projesi

Sera ortamında olgun meyve (elma) tespiti için Gazebo Sim dünyası ve klavyeyle (WASD) sürülen 4 tekerlekli robot. Robotta şu an RGB kamera var (kırmızı/yeşil elma ve RED/GREEN checkpoint tespiti); diğer sensörler ekleniyor.

## Paketler

| Paket | İçerik |
| --- | --- |
| `midterm_bringup` | `sim.launch.py`, `worlds/greenhouse.sdf`, `config/bridge.yaml` |
| `midterm_description` | Robot modeli (`urdf/greenhouse_bot.urdf.xacro`): 4 tekerlekli skid-steer gövde, Gazebo DiffDrive eklentisi, `urdf/camera.xacro` |
| `midterm_behavior` | `wasd_teleop`: klavyeden `/cmd_vel` yayınlayan teleop düğümü; `color_detector`: kamera görüntüsünde elma ve checkpoint tespiti |

## Dünya: `src/midterm_bringup/worlds/greenhouse.sdf`

- 3 saksı sırası (y = 1.4, 0.0, -1.4) ve aralarında 2 koridor
- Orta sırada 9 meyve: 5 kırmızı (olgun), 4 yeşil (olgun değil)
- Dört duvarla çevrili 11 x 5.4 m alan
- Dış model kullanmıyor; tek dosya olarak çalışır

## Kurulum

### Gereksinimler

- Ubuntu + **ROS 2 Lyrical** (`/opt/ros/lyrical`) ve Gazebo Sim
- `ros_gz_sim`, `ros_gz_bridge`, `robot_state_publisher`, `xacro`, `colcon`

ROS 2 kurulu bir sistemde eksik paketler:

~~~bash
source /opt/ros/lyrical/setup.bash
sudo apt update
sudo apt install -y git python3-colcon-common-extensions python3-rosdep \
  ros-lyrical-ros-gz-sim ros-lyrical-ros-gz-bridge \
  ros-lyrical-robot-state-publisher ros-lyrical-xacro
~~~

### Repoyu klonlama

Repo **private**: önce repo sahibinin sizi GitHub'da *Settings → Collaborators* bölümünden davet etmesi ve daveti kabul etmeniz gerekir.

~~~bash
git clone https://github.com/AbdullahTahaUstunsoy/Mobile-Robotics-Projesi.git ~/Mobile-Robotics-Projesi
cd ~/Mobile-Robotics-Projesi
~~~

`build/`, `install/` ve `log/` klasörleri repoda boş gelir; içleri derleme sırasında dolar ve git'e gönderilmez.

### Bağımlılıkları kontrol etme (isteğe bağlı)

~~~bash
sudo rosdep init        # sadece ilk kez; "already exists" hatası verirse geçin
rosdep update
rosdep install --from-paths src --ignore-src -r -y
~~~

## Derleme

~~~bash
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install
source install/setup.bash
~~~

Her yeni terminalde `source /opt/ros/lyrical/setup.bash && source install/setup.bash` çalıştırın.

## Çalıştırma

Terminal 1 — Gazebo, robot ve köprü:

~~~bash
ros2 launch midterm_bringup sim.launch.py
~~~

Terminal 2 — klavyeyle sürüş (bu terminal odakta olmalı):

~~~bash
source /opt/ros/lyrical/setup.bash && source install/setup.bash
ros2 run midterm_behavior wasd_teleop
~~~

| Tuş | Hareket |
| --- | --- |
| `w` / `s` | ileri / geri |
| `a` / `d` | yerinde sola / sağa dön |
| `q` / `e` | ileri + sola / sağa |
| `PgUp` / `PgDn` | hızı %10 artır / azalt |
| `space` | dur |
| `Ctrl+C` | çık (robotu durdurur) |

Tuşu basılı tutun; bırakınca robot durur.

## RGB kamera ve renk tespiti

Kamera gövdenin önünde, yerden ~0.40 m yükseklikte, **ileri** bakıyor (640×480, 15 Hz, 90° yatay görüş).
Böylece koridor boyunca iki yandaki sıralardaki elmaları ve ilerideki duvar checkpoint'lerini görüyor.
Yönü `urdf/camera.xacro` içindeki `camera_yaw` ile değiştirilebilir (`-pi/2` = sağa bakar).

`color_detector` düğümü `sim.launch.py` ile otomatik başlar. HSV maskeleriyle **renk** (kırmızı / yeşil),
kontur şekliyle **tür** ayrılır: yuvarlak leke = elma, 4 köşeli dışbükey leke (dikdörtgen / açılı bakınca yamuk) = checkpoint paneli.
Yapraklar (S ≈ 110) ve kahverengi saksılar (S ≈ 95) doygunluk eşiğiyle elenir.

| Topic | Tip | Açıklama |
| --- | --- | --- |
| `/camera/image_raw`, `/camera/camera_info` | `sensor_msgs/Image`, `CameraInfo` | Ham kamera görüntüsü |
| `/color_detector/red_apple` | `geometry_msgs/PointStamped` | Kırmızı (olgun) elma |
| `/color_detector/green_apple` | `geometry_msgs/PointStamped` | Yeşil (olgun değil) elma |
| `/color_detector/red_checkpoint` | `geometry_msgs/PointStamped` | Duvardaki RED panel |
| `/color_detector/green_checkpoint` | `geometry_msgs/PointStamped` | Duvardaki GREEN panel |
| `/color_detector/debug_image` | `sensor_msgs/Image` | Tespitler çizilmiş görüntü + sağ üstte maske |

`PointStamped` mesajları yalnızca nesne görüldüğü sürece yayınlanır (her etikette en büyük leke):
`point.x` = görüntü merkezine yatay uzaklık (−1 sol … +1 sağ), `point.y` = dikey (−1 üst … +1 alt),
`point.z` = nesnenin kapladığı görüntü oranı (yaklaştıkça büyür). Navigasyon tarafı, örneğin ~0.5 s mesaj gelmezse "görünmüyor" kabul edebilir.

Checkpoint panelleri için öneri: duvara yapışık ince kutu (ör. 0.02 × 0.4 × 0.3 m, merkez z ≈ 0.45 m),
renkler `0.9 0.05 0.05` (kırmızı) ve `0.05 0.8 0.1` (yeşil). Bu değerlerle test edildi.

~~~bash
ros2 run rqt_image_view rqt_image_view /color_detector/debug_image
ros2 topic echo /color_detector/red_apple
colcon test --packages-select midterm_behavior && colcon test-result --verbose   # birim testleri
~~~

Launch argümanları: `headless:=true` (Gazebo arayüzü olmadan), `x`, `y`, `yaw` (başlangıç konumu, varsayılan 1. koridor: `-0.5`, `0.65`, `0.0`), `world` (farklı bir dünya dosyası).

Sadece dünyayı açmak için: `gz sim -r src/midterm_bringup/worlds/greenhouse.sdf`
