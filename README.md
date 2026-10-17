# Mobile-Robotics-Projesi

Sera ortamında olgun meyve (elma) tespiti için Gazebo Sim dünyası.

## Dünya: `worlds/greenhouse.sdf`

- 3 saksı sırası (y = 1.4, 0.0, -1.4) ve aralarında 2 koridor
- Orta sırada 9 meyve: 5 kırmızı (olgun), 4 yeşil (olgun değil)
- Dört duvarla çevrili 11 x 5.4 m alan
- Sensör eklentileri yüklü (`ogre2` render, IMU), kamera ve LiDAR'lı bir robot eklenmeye hazır

Dış model kullanmıyor; tek dosya olarak çalışır.

## Çalıştırma

~~~bash
source /opt/ros/lyrical/setup.bash
gz sim -r worlds/greenhouse.sdf
~~~
