map_name=$1
if [ -z "$map_name" ]; then
    map_name="default"
fi

rosrun map_manager trans_modifier.py $map_name
rosrun map_manager map3d_transform.py $map_name
rosrun map_manager map2d_transform.py $map_name