# How to

rospy는 ROS에서 파이썬 코드를 실행할 수 있게 만들어주는 Python client library
많은 ROS tool들은 rospy가 사용됨

```cpp
cd workspace/src
source /opt/ros/~~~/setup.sh
catkin_init_workspace
git clone https://github.com/Phw9/ros_myutil.git
cd ../
catkin_make
```

```cpp
roscore

python ***.py
```
