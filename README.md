# **ControllerGPT**
**An AI controller that uses text prompts to control your robot.**  

ROS2 is interfaced via WebSockets through [rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite).  
***LangChain*** is used to create an [interface](https://github.com/kalashjain23/ControllerGPT/tree/main/ai_interface) with ChatGPT.

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
*Start your robot (for the showcase, I'll be using Turtlesim)*
```
ros2 run turtlesim turtlesim_node
```
*Run `rosbridge_server` to establish a connection with ROS*
```
source /opt/ros/humble/setup.bash # source your ROS distribution
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```
*Running the main script along with the key*
```
python3 main.py --key (OpenAI API Key)    # Run "python3 main.py -h" for help
``` 
*Now you'll be asked for the topic that you want ControllerGPT to control along with your goals*
```
Topic for publishing messages (leave blank if not any) → {/publisher_topic}
Topic for using services (leave blank if not any) → {/service_topic}

What do you want your robot to do? --> {your_prompt}
```
**Sit back, relax, and watch ControllerGPT complete the tasks for you!! :D**  
  
_Some more example prompts for you to try:_
```
→ Move forwards for 2 seconds and then stop the robot. (on /cmd_vel)
→ Move forwards for 2 seconds and then stop the robot. Also, spawn another robot at (1,2). (on /turtle1/cmd_vel and /spawn)
```
## *Adding custom interfaces*
You can add your own custom interfaces in the respective [messages](https://github.com/kalashjain23/ControllerGPT/tree/main/msg) and [services](https://github.com/kalashjain23/ControllerGPT/tree/main/srv) directory following a certain format.  
  
**The required format:**  
*Messages (.msg)*
```
{"message_type": message_type, "format": {format_of_your_message}}

Example, {"message_type": "std_msgs/String", "format": {"data": _}}
```
*Services (.srv)*
```
{"service_type": service_type, "format": {format_of_your_service}}

Example, {"service_type": "turtlesim/Spawn", "format": {"x": _, "y": _, "theta": _}}
```
*Note: The values of the interfaces are to be replaced with '_', which will be filled by ChatGPT.*

## *Visuals*

https://github.com/kalashjain23/ControllerGPT/assets/97672680/85c0e2ab-09b9-4412-a0df-23141ee88d36

This project is inspired by [ChatGPT_TurtleSim by Mhubii](https://github.com/mhubii/chatgpt_turtlesim/).
