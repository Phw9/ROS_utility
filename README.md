# How to

rospy는 ROS에서 파이썬 코드를 실행할 수 있게 만들어주는 Python client library
많은 ROS tool들은 rospy가 사용됨

```cpp
source /opt/ros/~~~/setup.sh
cd catkin_ws/src
catkin_create_pkg my_pkg rospy
```

```cpp
cd my_pkg
mkdir scripts
```

사용할 파이썬 코드를 scripts 디렉토리 안에 이동

CMakelists.txt 수정

```cpp
catkin_install_python(PROGRAMS
  scripts/bgr2gray.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

python 파일을 실행이 가능하게 만들어줘야한다.

```cpp
cd scripts
sudo chmod +x bgr2gray.py
cd ~/workspace
catkin_make
```
다음과 같이 실행
```cpp
python bgr2gray.py
```
