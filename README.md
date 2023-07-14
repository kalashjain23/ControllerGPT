# **ControllerGPT**
#### An AI controller that can control your robot and perform the tasks that you want.  
*Currently supports only 2 message types (Twist, String) but will be adding more.*

## **Prerequisites**
You should have your own working *OpenAI API key*.

## **How to use**
*Cloning the package*
```
git clone https://github.com/kalashjain23/ControllerGPT.git
cd ~/ControllerGPT
```
*Run `rosbridge_server`*
```
source /opt/ros/humble/setup.bash # source your ROS distribution
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```
*Running the main script along with the key*
```
python3 main.py --key (OpenAI API Key)
``` 
*Enter the required information about the topics and the goals, and you should be good to go!*

## *Visuals*

https://github.com/kalashjain23/ControllerGPT/assets/97672680/85c0e2ab-09b9-4412-a0df-23141ee88d36

