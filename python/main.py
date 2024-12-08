import os, sys
from PySide6.QtWidgets import QApplication
from gui.mainwindow import MainWindow

def path():
    try:
        # Get the current user's home directory
        home_dir = os.path.expanduser('~')

        # Initialize target_dir as None
        target_dir = None

        # Walk through the home directory and subdirectories to find 'Mobile_robotsystem'
        for root, dirs, files in os.walk(home_dir):
            if 'Mobile_robotsystem' in dirs:
                print("Root dir: ", root)   
                target_dir = os.path.join(root, 'Mobile_robotsystem//python')
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

def clear_file():
    path = "..//Docs//shared_file.txt"
    try:
        with open(path, 'w') as file:
            file.write(" ")
            print(os.getcwd(), " ", path)
    except Exception as e:
        print(f"Failed to clear file: {e}")
        return

def main(): 
    path()
    clear_file()
    app = QApplication([])
    widget = MainWindow()
    widget.show()
    sys.exit(app.exec())

if __name__ == '__main__':
    main()
