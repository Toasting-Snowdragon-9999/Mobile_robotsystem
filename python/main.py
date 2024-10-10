import os
from packages.share_data import SharedData
from packages.socket_connection import SocketConnect

def path():
    try:
        # Get the current user's home directory
        home_dir = os.path.expanduser('~')

        # Initialize target_dir as None
        target_dir = None

        # Walk through the home directory and subdirectories to find 'Mobile_robotsystem'
        for root, dirs, files in os.walk(home_dir):
            if 'Mobile_robotsystem' in dirs:
                target_dir = os.path.join(root, 'Mobile_robotsystem')
                break

        # Check if the target directory was found
        if target_dir:
            os.chdir(target_dir)
            print(f"Changed working directory to: {os.getcwd()}")
        else:
            print("Mobile_robotsystem directory not found.")

    except Exception as e:
        print(f"Failed to change directory: {e}")
        return

def main(): 
    path()
    print(os.getcwd())
    data = [
        [11, 8, 5]

    ]
    shared_data = SharedData()
    socket = SocketConnect()
    bits = shared_data.compute_bin(data)
    socket.add_validation(bits)
    try:
        socket.connect()
        socket.send_data(bits)
    except SharedData.SharedDataException as e:
        print(f"Error: {e}")
    socket.disconnect()


if __name__ == '__main__':
    main()

