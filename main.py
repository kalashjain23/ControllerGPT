import argparse

def args_factory() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument('--key', type=str, required=True, help='OpenAI API key.')
    args = parser.parse_args()
    return args
    
def main() -> None:
    args = args_factory()
    
    velocity_topic = input('Enter the name of the topic that you want to control --> ')
    
    print(velocity_topic, args)
    
if __name__ == '__main__':
    main()
    