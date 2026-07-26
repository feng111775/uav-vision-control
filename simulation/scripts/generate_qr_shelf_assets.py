#!/usr/bin/env python3
"""Generate Gazebo-visible QR textures and the 24-code shelf world."""

from pathlib import Path
import cv2


def main():
    root = Path(__file__).resolve().parents[1] / 'px4_overlay' / \
        'Tools' / 'simulation' / 'gz'
    material = root / 'models' / 'qr_shelf' / 'materials' / 'textures'
    material.mkdir(parents=True, exist_ok=True)
    encoder = cv2.QRCodeEncoder_create()
    for qr_id in range(1, 25):
        code = encoder.encode(str(qr_id))
        code = cv2.resize(code, (512, 512), interpolation=cv2.INTER_NEAREST)
        cv2.imwrite(str(material / f'qr_{qr_id:02d}.png'), code)

    visuals = []
    for index in range(24):
        qr_id = index + 1
        row, column = divmod(index, 6)
        # Shelf faces west toward the vehicle; thin planes lie on x=5.
        y = -1.25 + column * .5
        z = 2.25 - row * .5
        visuals.append(f'''
        <visual name="qr_{qr_id:02d}">
          <pose>0 {y:.2f} {z:.2f} 0 1.570796 0</pose>
          <geometry><plane><normal>0 0 1</normal>
            <size>0.19 0.19</size></plane></geometry>
          <material><pbr><metal><albedo_map>
            model://qr_shelf/materials/textures/qr_{qr_id:02d}.png
          </albedo_map><roughness>1</roughness><metalness>0</metalness>
          </metal></pbr></material>
        </visual>''')
    world = f'''<?xml version="1.0"?>
<sdf version="1.9"><world name="qr_shelf_world">
  <physics type="ode"><max_step_size>0.004</max_step_size>
    <real_time_factor>1</real_time_factor></physics>
  <gravity>0 0 -9.8</gravity>
  <scene><ambient>0.55 0.55 0.55 1</ambient>
    <background>0.75 0.75 0.75 1</background><shadows>true</shadows></scene>
  <model name="ground"><static>true</static><link name="link">
    <collision name="collision"><geometry><plane><normal>0 0 1</normal>
      <size>100 100</size></plane></geometry></collision>
    <visual name="visual"><geometry><plane><normal>0 0 1</normal>
      <size>100 100</size></plane></geometry><material>
      <diffuse>0.6 0.6 0.6 1</diffuse></material></visual></link></model>
  <model name="qr_shelf"><static>true</static><pose>5 0 0 0 0 0</pose>
    <link name="shelf">
      <visual name="white_back"><pose>0.015 0 1.5 0 0 0</pose>
        <geometry><box><size>0.03 3.2 2.6</size></box></geometry>
        <material><diffuse>0.95 0.95 0.95 1</diffuse></material></visual>
      <collision name="back_collision"><pose>0.03 0 1.5 0 0 0</pose>
        <geometry><box><size>0.06 3.2 2.6</size></box></geometry></collision>
{''.join(visuals)}
    </link>
  </model>
  <model name="landing_pad"><static>true</static><pose>7 0 0.015 0 0 0</pose>
    <link name="link"><visual name="pad"><geometry>
      <cylinder><radius>0.65</radius><length>0.03</length></cylinder>
      </geometry><material><diffuse>0.9 0.05 0.05 1</diffuse>
      </material></visual></link></model>
  <light name="sun" type="directional"><pose>0 0 10 0 0 0</pose>
    <cast_shadows>true</cast_shadows><intensity>1</intensity>
    <direction>-0.3 0.2 -0.9</direction></light>
</world></sdf>
'''
    world_path = root / 'worlds' / 'qr_shelf_world.sdf'
    world_path.parent.mkdir(parents=True, exist_ok=True)
    world_path.write_text(world, encoding='utf-8')
    config = '''<?xml version="1.0"?>
<model><name>qr_shelf</name><version>1.0</version>
<sdf version="1.9">model.sdf</sdf>
<author><name>uav-vision-control</name></author>
<description>Textures used by qr_shelf_world.</description></model>
'''
    (material.parents[1] / 'model.config').write_text(
        config, encoding='utf-8')
    print(world_path)


if __name__ == '__main__':
    main()
