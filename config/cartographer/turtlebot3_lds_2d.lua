-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

-- /* Author: Darby Lim */

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu_link",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.min_range = 0.12
TRAJECTORY_BUILDER_2D.max_range = 3.5
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true 
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)

-- [2026-09-06] 루프클로저를 다시 켠다.
--
-- 경위: 2026-09-01 에 지도가 여러 겹으로 밀려서 optimize_every_n_nodes = 0 으로
-- 루프클로저를 아예 껐다. 그때는 작은 방이라 네 벽이 서로 비슷해서 다른 벽을
-- 같은 벽으로 오판하는 게 문제였다.
-- 그런데 완전히 꺼두니 반대 문제가 났다 — 드리프트를 아무도 보정하지 않아서
-- 한 바퀴 도는 **도중에** 자세가 틀어지고, 같은 벽이 다른 각도로 두 번 그려진다
-- (2026-09-06 실측: 한 바퀴만 돌아도 지도 가운데를 대각선 벽이 가로지른다).
-- "한 바퀴만 돌기" 로는 해결되지 않았다. 겹치는 게 아니라 도는 중에 틀어지는 것이라서다.
--
-- 그래서 켜되 **오판하지 않도록 문턱을 높인다**:
--   min_score 0.65 -> 0.72   확신이 높을 때만 같은 곳으로 인정한다
--   optimize_every_n_nodes 0 -> 20   자주 보정해 오차가 쌓이기 전에 잡는다
-- 지도가 다시 여러 겹이 되면 min_score 를 더 올릴 것(0.75~0.8).
POSE_GRAPH.constraint_builder.min_score = 0.72
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.75

POSE_GRAPH.optimize_every_n_nodes = 20

-- 회전할 때 오차가 가장 크게 생긴다. 스캔 매칭에서 회전을 더 신뢰하도록
-- 가중치를 올려 제자리 회전 중 자세가 미끄러지는 것을 줄인다.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.

return options
