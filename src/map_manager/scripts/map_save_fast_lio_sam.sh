#!/usr/bin/env bash
map_name=$1
if [ -z "$map_name" ]; then
    map_name="default"
fi

if [ ! -d "`rospack find map_manager`/output" ]; then
    mkdir "`rospack find map_manager`/output"
fi

echo "### map name: $map_name ###"
echo "### start to save map ###"
rosservice call /fast_lio_sam/save_map 0.05 "`rospack find map_manager`/output/${map_name}"