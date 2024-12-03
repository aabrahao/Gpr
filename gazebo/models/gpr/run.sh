#gz sim model.sdf
#ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=./bridge.yaml

#ros2 launch ros_gz_sim gz_sim.launch.py gz_args:=./model.sdf 

ros2 launch ros_gz_sim ros_gz_sim.launch.py \
    world_sdf_file:=./model.sdf \
    bridge_name:=ros_gz_bridge \
    config_file:=./bridge.yaml \
    use_composition:=True \
    create_own_container:=True
