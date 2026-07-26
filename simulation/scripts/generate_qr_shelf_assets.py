#!/usr/bin/env python3
"""Generate Gazebo-visible QR textures and the 24-code shelf world."""

from pathlib import Path
import cv2


def main():
    root = Path(__file__).resolve().parents[1] / 'px4_overlay' / \
        'Tools' / 'simulation' / 'gz'
    material = root / 'models' / 'qr_shelf' / 'materials' / 'textures'
    material.mkdir(parents=True, exist_ok=True)
    mesh_root = material.parents[1] / 'meshes'
    mesh_root.mkdir(parents=True, exist_ok=True)
    for obsolete in mesh_root.glob('*.obj'):
        obsolete.unlink()
    (mesh_root / 'qr_black.mtl').unlink(missing_ok=True)
    encoder = cv2.QRCodeEncoder_create()
    for qr_id in range(1, 25):
        matrix = encoder.encode(str(qr_id))
        texture = cv2.resize(
            matrix, (512, 512), interpolation=cv2.INTER_NEAREST)
        texture = cv2.cvtColor(texture, cv2.COLOR_GRAY2BGR)
        cv2.imwrite(str(material / f'qr_{qr_id:02d}.png'), texture)
        vertices, triangles = [], []
        modules = matrix.shape[0]
        for row in range(modules):
            for column in range(modules):
                # Gazebo's Assimp renderer does not consistently honour dark
                # mesh materials in headless mode.  Render the white cells
                # (including the quiet zone) as geometry on a black backing.
                if matrix[row, column] == 0:
                    continue
                # The shelf is observed from its -X side. Reverse model Y so
                # the camera receives the QR matrix, not its mirror image.
                y0 = (.5 - (column + 1) / modules) * .19
                y1 = (.5 - column / modules) * .19
                z1 = (.5 - row / modules) * .19
                z0 = (.5 - (row + 1) / modules) * .19
                start = len(vertices)
                vertices.extend([
                    (0, y0, z0), (0, y0, z1),
                    (0, y1, z1), (0, y1, z0)])
                triangles.extend((start, start + 1, start + 2,
                                  start, start + 2, start + 3))
        positions = ' '.join('%.8f %.8f %.8f' % item for item in vertices)
        indices = ' '.join(str(item) for item in triangles)
        dae = f'''<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset><unit meter="1" name="meter"/><up_axis>Z_UP</up_axis></asset>
  <library_effects><effect id="white-effect"><profile_COMMON>
    <technique sid="common"><lambert><emission><color>1 1 1 1</color></emission>
      <ambient><color>1 1 1 1</color></ambient>
      <diffuse><color>1 1 1 1</color></diffuse>
    </lambert></technique></profile_COMMON></effect></library_effects>
  <library_materials><material id="white-material" name="white">
    <instance_effect url="#white-effect"/></material></library_materials>
  <library_geometries><geometry id="qr-mesh" name="QR {qr_id}">
    <mesh><source id="positions"><float_array id="positions-array"
      count="{len(vertices) * 3}">{positions}</float_array>
      <technique_common><accessor source="#positions-array"
        count="{len(vertices)}" stride="3"><param name="X" type="float"/>
        <param name="Y" type="float"/><param name="Z" type="float"/>
      </accessor></technique_common></source>
      <vertices id="vertices"><input semantic="POSITION"
        source="#positions"/></vertices>
      <triangles material="white-symbol" count="{len(triangles) // 3}">
        <input semantic="VERTEX" source="#vertices" offset="0"/>
        <p>{indices}</p></triangles>
    </mesh></geometry></library_geometries>
  <library_visual_scenes><visual_scene id="scene">
    <node id="qr"><instance_geometry url="#qr-mesh"><bind_material>
      <technique_common><instance_material symbol="white-symbol"
        target="#white-material"/></technique_common>
    </bind_material></instance_geometry></node>
  </visual_scene></library_visual_scenes>
  <scene><instance_visual_scene url="#scene"/></scene>
</COLLADA>
'''
        (mesh_root / f'qr_{qr_id:02d}.dae').write_text(
            dae, encoding='utf-8')

    visuals = []
    for index in range(24):
        qr_id = index + 1
        row, column = divmod(index, 6)
        # Shelf faces west toward the vehicle; thin planes lie on x=5.
        y = -1.25 + column * .5
        z = 2.25 - row * .5
        visuals.append(f'''
        <visual name="qr_{qr_id:02d}_black">
          <pose>-0.002 {y:.2f} {z:.2f} 0 0 0</pose>
          <geometry><box><size>0.004 0.19 0.19</size></box></geometry>
          <material><ambient>0 0 0 1</ambient><diffuse>0 0 0 1</diffuse>
          <specular>0 0 0 1</specular></material>
        </visual>
        <visual name="qr_{qr_id:02d}_modules">
          <pose>-0.005 {y:.2f} {z:.2f} 0 0 0</pose>
          <geometry><mesh><uri>meshes/qr_{qr_id:02d}.dae</uri></mesh></geometry>
          <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse>
          <specular>0 0 0 1</specular></material>
        </visual>''')
    model_root = material.parents[1]
    shelf_model = f'''<?xml version="1.0"?>
<sdf version="1.9"><model name="qr_shelf"><static>true</static>
  <link name="shelf">
    <visual name="white_back"><pose>0.015 0 1.5 0 0 0</pose>
      <geometry><box><size>0.03 3.2 2.6</size></box></geometry>
      <material><diffuse>0.95 0.95 0.95 1</diffuse></material></visual>
    <collision name="back_collision"><pose>0.03 0 1.5 0 0 0</pose>
      <geometry><box><size>0.06 3.2 2.6</size></box></geometry></collision>
{''.join(visuals)}
  </link>
</model></sdf>
'''
    (model_root / 'model.sdf').write_text(shelf_model, encoding='utf-8')
    world = f'''<?xml version="1.0"?>
<sdf version="1.9"><world name="qr_shelf_world">
  <physics type="ode"><max_step_size>0.004</max_step_size>
    <real_time_factor>1</real_time_factor></physics>
  <gravity>0 0 -9.8</gravity>
  <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
  <atmosphere type="adiabatic"/>
  <scene><ambient>0.55 0.55 0.55 1</ambient>
    <background>0.75 0.75 0.75 1</background><shadows>true</shadows></scene>
  <model name="ground"><static>true</static><link name="link">
    <collision name="collision"><geometry><plane><normal>0 0 1</normal>
      <size>100 100</size></plane></geometry></collision>
    <visual name="visual"><geometry><plane><normal>0 0 1</normal>
      <size>100 100</size></plane></geometry><material>
      <diffuse>0.6 0.6 0.6 1</diffuse></material></visual></link></model>
  <include><uri>model://qr_shelf</uri><pose>1.5 0 0.25 0 0 0</pose></include>
  <model name="landing_pad"><static>true</static><pose>0.2 0 0.006 0 0 0</pose>
    <link name="link"><visual name="pad"><geometry>
      <box><size>1.0 1.0 0.012</size></box>
      </geometry><material><diffuse>0.9 0.05 0.05 1</diffuse>
      </material></visual></link></model>
  <model name="shelf_validation_camera"><static>true</static>
    <pose>-0.5 0 2.0 0 0 0</pose><link name="link">
      <sensor name="camera" type="camera">
        <topic>/camera/shelf_validation/image</topic>
        <camera><horizontal_fov>1.0471975512</horizontal_fov>
          <image><width>640</width><height>480</height>
            <format>R8G8B8</format></image>
          <clip><near>0.05</near><far>10</far></clip></camera>
        <always_on>true</always_on><update_rate>5</update_rate>
      </sensor>
    </link>
  </model>
  <light name="sun" type="directional"><pose>0 0 10 0 0 0</pose>
    <cast_shadows>true</cast_shadows><intensity>1</intensity>
    <direction>-0.3 0.2 -0.9</direction></light>
  <spherical_coordinates>
    <surface_model>EARTH_WGS84</surface_model>
    <world_frame_orientation>ENU</world_frame_orientation>
    <latitude_deg>47.397971057728974</latitude_deg>
    <longitude_deg>8.546163739800146</longitude_deg>
    <elevation>0</elevation>
  </spherical_coordinates>
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
    (model_root / 'model.config').write_text(
        config, encoding='utf-8')
    print(world_path)


if __name__ == '__main__':
    main()
