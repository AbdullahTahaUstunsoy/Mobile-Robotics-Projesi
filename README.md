# Mobile-Robotics-Projesi

Sera ortamında olgun meyve (elma) tespiti için Gazebo Sim dünyası ve klavyeyle (WASD) sürülen 4 tekerlekli robot. Robotta henüz sensör yok; sadece gövde, tekerlekler ve sürüş eklentisi var.

## Paketler

| Paket | İçerik |
| --- | --- |
| `midterm_bringup` | `sim.launch.py`, `worlds/greenhouse.sdf`, `config/bridge.yaml` |
| `midterm_description` | Robot modeli (`urdf/greenhouse_bot.urdf.xacro`): 4 tekerlekli skid-steer gövde, Gazebo DiffDrive eklentisi |
| `midterm_behavior` | `wasd_teleop`: klavyeden `/cmd_vel` yayınlayan teleop düğümü |

## Dünya: `src/midterm_bringup/worlds/greenhouse.sdf`

- 3 saksı sırası (y = 1.4, 0.0, -1.4) ve aralarında 2 koridor
- Orta sırada 9 meyve: 5 kırmızı (olgun), 4 yeşil (olgun değil)
- Dört duvarla çevrili 11 x 5.4 m alan
- Dış model kullanmıyor; tek dosya olarak çalışır

## Derleme

~~~bash
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install
source install/setup.bash
~~~

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

Launch argümanları: `headless:=true` (Gazebo arayüzü olmadan), `x`, `y`, `yaw` (başlangıç konumu, varsayılan 1. koridor: `-0.5`, `0.65`, `0.0`), `world` (farklı bir dünya dosyası).

Sadece dünyayı açmak için: `gz sim -r src/midterm_bringup/worlds/greenhouse.sdf`
