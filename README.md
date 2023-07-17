# **ControllerGPT**
**An AI controller that can control your robot and perform the tasks that you want.**  
*Currently supports few interfaces (Twist, String, Spawn) but will be adding more.*  

ROS2 is interfaced via WebSockets through [rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite).  
_LangChain_ is used to create an [interface](https://github.com/kalashjain23/ControllerGPT/tree/main/ai_interface) with ChatGPT.

## **Prerequisites**
→ You should have your own working _**OpenAI API key**_.

## **How to use**
*Cloning the package*
```
git clone https://github.com/kalashjain23/ControllerGPT.git
cd ~/ControllerGPT
```
*Install the dependencies in your virtual environment*
```
python3 -m venv env
source env/bin/activate

pip install -r requirements.txt
```
*Run `rosbridge_server` to establish connection with ROS*
```
source /opt/ros/humble/setup.bash # source your ROS distribution
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```
*Running the main script along with the key*
```
python3 main.py --key (OpenAI API Key)
``` 
*Now you'll be asked for the topic that you want to control and your goals*
```
Topic for publishing messages (leave blank if not any) → {/publisher_topic}
Topic for using services (leave blank if not any) → {/service_topic}

What do you want your robot to do? --> {your_prompt}
```
**Sit back, relax and watch ControllerGPT complete the tasks for you!! :D**  
_Some more example prompts for you to try:_
```
→ Move forwards for 2 seconds and then stop the robot. (on /cmd_vel)
→ Move forwards for 2 seconds and then stop the robot. Also, spawn another robot at (1,2). (on /turtle1/cmd_vel and /spawn)
```

## *Visuals*

https://github.com/kalashjain23/ControllerGPT/assets/97672680/85c0e2ab-09b9-4412-a0df-23141ee88d36

This project is inspired by [ChatGPT_TurtleSim by Mhubii](https://github.com/mhubii/chatgpt_turtlesim/).