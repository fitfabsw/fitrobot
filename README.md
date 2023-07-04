# fitrobot套件


## 簡介

本程式為ROS2套件，可輸出機器人相對於地圖原點的座標

Slam Toolbox支援先建圖後導航，以及一邊建圖一邊導航，在先建圖後導航的情境，可以透過監聽 /amcl_pose 這個topic來獲得機器人的座標；而一邊建圖一邊導航沒有這個topic可以使用，因此需透過本程式取得機器人座標

此程式也可以完全取代 /amcl_pose ，不管先建圖後導航，或一邊建圖一邊導航，都可以改成監聽 /custom_pose


<br/>

## 安裝


```
sudo apt install python3-colcon-common-extensions
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone git@github.com:fitfabsw/fitrobot.git
cd ..
colcon build --packages-select fitrobot
source install/setup.bash
```
如果有修改到 tf_converter_node.py，需要重新跑colcon build；如果需要頻繁修改 tf_converter_node.py，不想每次都重新build的話，可以跑一次以下指令：
```
colcon build --symlink-install --packages-select fitrobot
```
之後的修改都可以直接執行，不用重build


<br/>

## 用法

**先執行建圖或導航的程式**，再執行以下指令
```
ros2 run fitrobot tf_converter_node
```
此時會產生 **/custom_pose** 這個topic，回傳的座標格式跟amcl_pose一樣，message type都是 **geometry_msgs/msg/PoseWithCovarianceStamped**


