#!/bin/bash

for i in $(seq 0 5); do
	rosservice call /gazebo/delete_model "model_name: 'cube$i'"
done

rosservice call /gazebo/delete_model "model_name: 'bucket'"
