python scripts/bvh_broadcaster.py example/Dataset/csv/froga_ia.bvh world -l

rosrun rviz rviz

rosrun tf static_transform_publisher 0 0 0 1 0 0 1  map world 50


~/src/valu3s/human_motions/bvh_broadcaster/example/Dataset/csv$ csv2bvh female_90_hierarchy.csv female_90_poses.csv female_90_rotations_finko_hipX.csv  -o ./froga_ia.bvh
