To run code on the robot,
  1. Navigate to the BurgerClazz Directory
  2. Run colcon build
  3. Run source install/local_setup.bash
  4. You can now run whatever package node you want. You can see what nodes you can run in object_follower with: ros2 pkg executables aizen_object_follower
  5. You can run a node with: ros2 run aizen_object_follower find_object.py  (Replace find_object.py with whateva ya node is)
  6. If you wanna check to see if stuff is being published just open a new terminal and do: ros2 topic echo /topic (Replace /topic with whatever the topic name is i.e. /tracking/centroid)


TODO

