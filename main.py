import argparse
import roslibpy
import time

from ai_interface import OpenAIInterface

def args_factory() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    
    parser.add_argument('--key', type=str, required=True, help='OpenAI API key.')
    parser.add_argument('--host', type=str, default='localhost', help='ROS host.')
    parser.add_argument('--port', type=int, default='9090', help='ROS port.')
    parser.add_argument('--topic', type=str, default='geometry_msgs/Twist', help='Topic for velocities of the robot.')
    
    args = parser.parse_args()
    return args
    
def main() -> None:
    args = args_factory()
    
    velocity_topic = str(input('Enter the name of the topic that you want to control --> '))
    
    ros_client = roslibpy.Ros(host=args.host, port=args.port)
    ros_client.run()
    
    openai_interface = OpenAIInterface(key=args.key)
    
    while True:
        try:
            prompt = str(input("What do you want your robot to do? --> "))
            print("Breaking down the goal and creating steps...")
            messages = openai_interface.get_messages(prompt=prompt)
            print("Done...")
            
            publisher = roslibpy.Topic(ros_client, velocity_topic, args.topic)
            if ros_client.is_connected:
                for message in messages:
                    publisher.publish(roslibpy.Message(message))
                    time.sleep(1)  
        except KeyboardInterrupt:
            break
        
    ros_client.terminate()
    
    print(velocity_topic, args)
    
if __name__ == '__main__':
    main()
    