# bravo_description

To launch the gazebo simulation:
```
roslaunch bravo_description bravo_7_gazebo.launch
```

To move the arm publish to the `/arm_controller/command` topic using:
```
rostopic pub -1 arm_controller/command std_msgs/Float64MultiArray "layout:
  dim:
  - label: ''
    size: 0
    stride: 0
  data_offset: 0
data: [0,1,1,1,1,1,1,1]"
```
Joint order is described as:\
<jaws, b, c, d, e, f, g>.\
\
The joints propagate from the end of the arm to the base.
