#!/usr/bin/env python3
import random
import os
import shutil
import cv2
import cv2.aruco as aruco
import numpy as np
import rclpy
from rclpy.node import Node
from ros_gz_interfaces.srv import SpawnEntity

VERTICAL_X   = [8.5, 11.5, 14.5, 17.5, 20.5, 23.5, 26.5, 29.5, 32.5]
HORIZONTAL_Y = [0, 3, 6, 9]
EXCLUDE      = [(0, 0)]
MARKER_SIZE  = 0.4
MARKER_Z     = 0.02
NUM_MARKERS  = 4
ARUCO_DICT   = aruco.DICT_4X4_50
MARKER_PX    = 200
SAVE_DIR     = '/tmp/aruco_markers'


def get_all_intersections():
    pts = []
    for x in VERTICAL_X:
        for y in HORIZONTAL_Y:
            if (x, y) not in EXCLUDE:
                pts.append((x, y))
    return pts


def generate_marker_png(marker_id, save_path):
    dictionary = aruco.getPredefinedDictionary(ARUCO_DICT)
    marker_img = aruco.generateImageMarker(dictionary, marker_id, MARKER_PX)
    cv2.imwrite(save_path, marker_img)


def create_model_dir(marker_id, png_path):
    model_dir = os.path.join(SAVE_DIR, f'model_{marker_id}')
    tex_dir = os.path.join(model_dir, 'materials', 'textures')
    script_dir = os.path.join(model_dir, 'materials', 'scripts')
    os.makedirs(tex_dir, exist_ok=True)
    os.makedirs(script_dir, exist_ok=True)

    tex_filename = f'marker_{marker_id}.png'
    tex_path = os.path.join(tex_dir, tex_filename)
    shutil.copy(png_path, tex_path)

    mat_name = f'ArUco/Marker{marker_id}'
    with open(os.path.join(script_dir, f'marker_{marker_id}.material'), 'w') as f:
        f.write(f"""material {mat_name}
{{
  technique
  {{
    pass
    {{
      texture_unit
      {{
        texture {tex_filename}
        filtering none
      }}
    }}
  }}
}}
""")

    with open(os.path.join(model_dir, 'model.config'), 'w') as f:
        f.write(f"""<?xml version="1.0"?>
<model>
  <name>aruco_marker_{marker_id}</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
</model>
""")

    return model_dir, mat_name


def make_sdf(model_name, model_dir, mat_name, x, y, z, size):
    tex_path = f'file://{model_dir}/materials/textures/marker_{model_name.split("_")[-1]}.png'
    sdf = f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="{model_name}">
    <static>true</static>
    <link name="link">
      <pose>{x} {y} {z} 0 0 0</pose>
      <visual name="visual">
        <geometry>
          <box>
            <size>{size} {size} 0.001</size>
          </box>
        </geometry>
        <material>
          <diffuse>1 1 1 1</diffuse>
          <specular>0 0 0 1</specular>
          <pbr>
            <metal>
              <albedo_map>{tex_path}</albedo_map>
            </metal>
          </pbr>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""
    return sdf


class ArucoSpawner(Node):
    def __init__(self, intersections):
        super().__init__('aruco_spawner')
        self.intersections = intersections
        self.client = self.create_client(SpawnEntity, '/world/default/create')

    def spawn_all(self):
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('서비스 /spawn_entity 사용 불가!')
            return

        os.makedirs(SAVE_DIR, exist_ok=True)
        FIXED_MARKER_ID = 23
        num_random = len(self.intersections) - 1
        candidate_ids = [i for i in range(50) if i != FIXED_MARKER_ID]
        random_ids = random.sample(candidate_ids, num_random)

        for i, (x, y, z) in enumerate(self.intersections):
            marker_id = FIXED_MARKER_ID if i == 0 else random_ids[i - 1]
            png_path = os.path.join(SAVE_DIR, f'marker_{marker_id}.png')
            model_name = f'aruco_marker_{marker_id}'

            generate_marker_png(marker_id, png_path)
            self.get_logger().info(f'마커 {marker_id} -> PNG 생성: {png_path}')

            model_dir, mat_name = create_model_dir(marker_id, png_path)
            self.get_logger().info(f'마커 {marker_id} -> 모델 디렉토리: {model_dir}')

            sdf_str = make_sdf(model_name, model_dir, mat_name, x, y, z, MARKER_SIZE)

            req = SpawnEntity.Request()
            req.entity_factory.name = model_name
            req.entity_factory.sdf = sdf_str

            future = self.client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

            if future.result() is not None:
                self.get_logger().info(
                    f'마커 {marker_id} 스폰 완료 ({x}, {y}, {z}): success={future.result().success}'
                )
            else:
                self.get_logger().error(f'마커 {marker_id} 스폰 실패')


def main():
    rclpy.init()

    fixed_position = (0, 0, 0.02)

    all_pts = get_all_intersections()

    fixed_xy = (fixed_position[0], fixed_position[1])
    if fixed_xy in all_pts:
        all_pts.remove(fixed_xy)

    random_pts = random.sample(all_pts, min(NUM_MARKERS, len(all_pts)))
    random_pts_with_z = [(x, y, MARKER_Z) for x, y in random_pts]

    chosen = [fixed_position] + random_pts_with_z

    print(f'선택된 교차점: {chosen}')

    node = ArucoSpawner(chosen)
    node.spawn_all()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
