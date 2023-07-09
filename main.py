import argparse
import roslibpy

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
    
    while True:
        try:
            publisher = roslibpy.Topic(ros_client, velocity_topic, args.topic)
            if ros_client.is_connected:
                # msg = {linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}
                publisher.publish(roslibpy.Message({'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0}, 'angular': {'x': 0.0, 'y': 0.0, 'z': 1.0}}))    
        except KeyboardInterrupt:
            break
        
    ros_client.terminate()
    
    print(velocity_topic, args)
    
if __name__ == '__main__':
    main()
    