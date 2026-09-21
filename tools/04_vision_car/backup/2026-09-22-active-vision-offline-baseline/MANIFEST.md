# RAICOM active-vision offline baseline manifest

## Baseline refs
- Baseline commit: `caef538` (`caef538de815b343251489a46e162dee646f2789`)
- Tag: `raicom-active-vision-pre-20260922`
- Backup branch: `backup/raicom-active-vision-pre-20260922`

## Source and backup paths
- Package source: `ROS1/ros_ws-main/ros_ws/src/raicom/`
- Package backup: `tools/04_vision_car/backup/2026-09-22-active-vision-offline-baseline/raicom_package_baseline_caef538/`
- YAML source: `tools/04_vision_car/raicom_*.yaml`
- YAML backup: `tools/04_vision_car/backup/2026-09-22-active-vision-offline-baseline/raicom_yaml_baseline_caef538/`

## Safe package-only rollback
```bash
git restore --source raicom-active-vision-pre-20260922 -- ROS1/ros_ws-main/ros_ws/src/raicom
```

**Warning:** Do not use `git reset --hard` for rollback; it discards unrelated working-tree changes.

## File SHA-256

| Source path | Backup path | SHA-256 | Verified |
| --- | --- | --- | --- |
| `ROS1/ros_ws-main/ros_ws/src/raicom/CMakeLists.txt` | `tools/04_vision_car/backup/2026-09-22-active-vision-offline-baseline/raicom_package_baseline_caef538/CMakeLists.txt` | `55ba1c7161fab6ab8ac0b5d13f596736494bb253b8bad646bdc6c5ed9bd1ddbf` | PASS |
| `ROS1/ros_ws-main/ros_ws/src/raicom/launch/nav.launch` | `tools/04_vision_car/backup/2026-09-22-active-vision-offline-baseline/raicom_package_baseline_caef538/launch/nav.launch` | `a4606eada6a2c865fee66261ac2a85df42eea53d0f2038addf3f91079c8a9368` | PASS |
| `ROS1/ros_ws-main/ros_ws/src/raicom/launch/nodes.launch` | `tools/04_vision_car/backup/2026-09-22-active-vision-offline-baseline/raicom_package_baseline_caef538/launch/nodes.launch` | `03807d40dae4e2923ca2890c3e7d62a94b659e893c1470078bd2304f8ed27cfd` | PASS |
| `ROS1/ros_ws-main/ros_ws/src/raicom/launch/raicom.launch` | `tools/04_vision_car/backup/2026-09-22-active-vision-offline-baseline/raicom_package_baseline_caef538/launch/raicom.launch` | `1c87e5f66ee89cff37b1de72df0c885ea4bbe208224c0337255775d7055adda0` | PASS |
| `ROS1/ros_ws-main/ros_ws/src/raicom/launch/voice.launch` | `tools/04_vision_car/backup/2026-09-22-active-vision-offline-baseline/raicom_package_baseline_caef538/launch/voice.launch` | `95def8462a60f1e987b6b4fb2247b6c2c67ba156265d1c5ee21762ae80042603` | PASS |
| `ROS1/ros_ws-main/ros_ws/src/raicom/package.xml` | `tools/04_vision_car/backup/2026-09-22-active-vision-offline-baseline/raicom_package_baseline_caef538/package.xml` | `58368eb6a405f54b3cae8389baf68a497636268ef64a95c926100ef614586caa` | PASS |
| `ROS1/ros_ws-main/ros_ws/src/raicom/scripts/raicom_arm.py` | `tools/04_vision_car/backup/2026-09-22-active-vision-offline-baseline/raicom_package_baseline_caef538/scripts/raicom_arm.py` | `38af24134056dc470fa4f1e5778dec122702a844ab4b678594663acdc242cd73` | PASS |
| `ROS1/ros_ws-main/ros_ws/src/raicom/scripts/raicom_auto.py` | `tools/04_vision_car/backup/2026-09-22-active-vision-offline-baseline/raicom_package_baseline_caef538/scripts/raicom_auto.py` | `9f456874581f7d8274bb24b4a62babb51661ac365440a4ddef2139711fe75fcd` | PASS |
| `ROS1/ros_ws-main/ros_ws/src/raicom/scripts/raicom_voice.py` | `tools/04_vision_car/backup/2026-09-22-active-vision-offline-baseline/raicom_package_baseline_caef538/scripts/raicom_voice.py` | `f03ade3ce3c4c3e861d175128a09d44e0ed5c03746eda54947720a05942c26fb` | PASS |
| `ROS1/ros_ws-main/ros_ws/src/raicom/scripts/raicom_yolo_trt.py` | `tools/04_vision_car/backup/2026-09-22-active-vision-offline-baseline/raicom_package_baseline_caef538/scripts/raicom_yolo_trt.py` | `f3a80ac3096c74a35640b29b2a8e68fefef93df1599e082be57a3599f8ff52de` | PASS |
| `tools/04_vision_car/raicom_arm_poses.yaml` | `tools/04_vision_car/backup/2026-09-22-active-vision-offline-baseline/raicom_yaml_baseline_caef538/raicom_arm_poses.yaml` | `9dd8293d10e7a27badacc7db1f56491fe8d8972268749fd32da834bd366001c6` | PASS |
| `tools/04_vision_car/raicom_grasp_spot.yaml` | `tools/04_vision_car/backup/2026-09-22-active-vision-offline-baseline/raicom_yaml_baseline_caef538/raicom_grasp_spot.yaml` | `ec7d1205287ea651435433592e96d1a7e250f6fb578a223c7c1bbcd57361d222` | PASS |
| `tools/04_vision_car/raicom_waypoints.yaml` | `tools/04_vision_car/backup/2026-09-22-active-vision-offline-baseline/raicom_yaml_baseline_caef538/raicom_waypoints.yaml` | `73b25e6a30f4c0ba7ca05517d576064c7a26e536094da1b6fab097a6d84fac86` | PASS |

## Verification summary
- Copied files: 13 (10 raicom + 3 yaml)
- Hash verification: ALL PASS
