import argparse
import roslibpy
import time

from ai_interface import AIInterface

def args_factory() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    
    parser.add_argument('--key', type=str, required=True, help='OpenAI API key.')
    parser.add_argument('--model', type=str, default='gpt-3.5-turbo', help='OpenAI API model.')
    parser.add_argument('--host', type=str, default='localhost', help='ROS host.')
    parser.add_argument('--port', type=int, default='9090', help='ROS port.')
    
    args = parser.parse_args()
    return args
    
def main() -> None:
    args = args_factory()
    
    publisher_topic = str(input('Topic for publishing messages (leave blank if not any) → '))
    service_topic = str(input('Topic for using services (leave blank if not any) → '))
    
    ros_client = roslibpy.Ros(host=args.host, port=args.port)
    ros_client.run()
    
    openai_interface = AIInterface(key=args.key, model=args.model)
    
    while True:
        try:
            prompt = str(input("\nWhat do you want your robot to do? → "))
            print("Breaking down the goal and creating steps...")
            interfaces_list = openai_interface.get_messages(prompt=prompt)
            print("Done...\n")
            
            for interface in interfaces_list:
                interface_category = interface["category"]
                interface_type = interface["type"]
                interface_data = interface["data"]
                
                if interface_category == "msg":
                    publisher = roslibpy.Topic(ros_client, publisher_topic, interface_type)
                    if ros_client.is_connected:
                        publisher.publish(roslibpy.Message(interface_data))
                        time.sleep(1)
                elif interface_category == "srv":
                    service = roslibpy.Service(ros_client, service_topic, interface_type)
                    request = roslibpy.ServiceRequest()
                    if ros_client.is_connected:
                        service.call(request=request)
                        time.sleep(1)
                else:
                    print("\nOops! We were facing some issues with this prompt. Try reframing.")
                    raise Exception
                    
        except Exception:
            print("\nThank you for using ControllerGPT!\n")
            break
        
    ros_client.terminate()
    
if __name__ == '__main__':
    main()
    