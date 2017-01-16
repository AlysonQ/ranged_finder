# ranged_finder
A ROS package for multiple VL53L0X application on Nvidia Jetson TK1
- Author : Ying-Hua(Alyson) Chen
- E-mail : qoo810823@gmail.com
- Date   : 2016.11.03
- Check detail steps to my blog [Click me](https://hollyqood.wordpress.com/2016/11/07/ros-package-multi-vl53l0x-sensors-on-jetson-tk1/)<br />


# How to build source code
> cd catkin_ws/src
>
> git clone https://github.com/AlysonQ/ranged_finder
>
> cd ..
>
> catkin_make

# How to RUN source code
>
### Must be sudo !!
> sudo -s
>
> roscore
### Then open second terminal
- If you only want to read one sensor
>
> rosrun ranged_finder measure1sensor_node
>
- If you  want to read two sensors
>
> rosrun ranged_finder measure2sensor_node
>
- If you  want to read three sensors
>
> rosrun ranged_finder measure3sensor_node


