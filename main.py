import argparse
import roslibpy
import time

from ai_interface import AIInterface

def args_factory() -> argparse.Namespace:
    '''
    Declaring all the required and optional arguments.
    Get help by using 'python3 main.py -h'.
    '''
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
    
    '''Setting up connection with the ROS server'''
    ros_client = roslibpy.Ros(host=args.host, port=args.port)
    ros_client.run()
    
    '''Creating an interface with ChatGPT'''
    openai_interface = AIInterface(key=args.key, model=args.model)
    
    while True:
        try:
            '''Getting all the required ROS2 interfaces using ChatGPT '''
            prompt = str(input("\nWhat do you want your robot to do? → "))
            print("Breaking down the goal and creating steps...")
            interfaces_list = openai_interface.get_interfaces(prompt=prompt)
            print("Done...\n")
            
            '''Iterating through the response by ChatGPT and taking required actions.'''
            for interface in interfaces_list:
                interface_category = interface["category"]
                interface_type = interface["type"]
                interface_data = interface["data"]
                
                match interface_category:
                    case "msg":
                        publisher = roslibpy.Topic(ros_client, publisher_topic, interface_type)
                        if ros_client.is_connected:
                            publisher.publish(roslibpy.Message(interface_data))
                    case "srv":
                        service = roslibpy.Service(ros_client, service_topic, interface_type)
                        request = roslibpy.ServiceRequest()
                        if ros_client.is_connected:
                            service.call(request=request)
                    case _:
                        print("\nOops! We were facing some issues with this prompt. Try reframing.")
                        raise Exception
                time.sleep(1)
                    
        except Exception:
            print("\nThank you for using ControllerGPT!\n")
            break
        
    ros_client.terminate()
    
if __name__ == '__main__':
    main()
    