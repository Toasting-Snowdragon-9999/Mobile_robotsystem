import os
from packages.share_data import SharedData

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
        [11, 8, 5],
        [14, 7, 7],
        [11, 9, 9],
    ]
    shared_data = SharedData()
    bits = shared_data.calculate_bit_string(data)
    value = shared_data.bits_to_int(bits)
    print (f"Value: {value}")
    try:
        shared_data.save_to_file(value)
        bits2 = shared_data.int_to_bit(value)
        value2 = shared_data.bits_to_int(bits2)
        if bits2 == bits and value == value2:
            print("success")
        else:
            print("failure")
    except SharedData.SharedDataException as e:
        print(f"Error: {e}")

if __name__ == '__main__':
    main()
