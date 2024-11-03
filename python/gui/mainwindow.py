# This Python file uses the following encoding: utf-8

from gui.shell import Shell
from gui.rules import Rules
from gui.send_path import SendPath
from gui.run_cpp import RunCpp

from PySide6.QtWidgets import QMainWindow
# Important:
# You need to run the following command to generate the ui_form.py file
#     pyside6-uic form.ui -o ui_form.py, or
#     pyside2-uic form.ui -o ui_form.py
from gui.ui_form import Ui_MainWindow

class MainWindow(QMainWindow):
    def __init__(self, *args: tuple, **kwargs: dict) -> None:
        super().__init__(*args, **kwargs)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.setup()

    def setup(self):
        self.ui.send_path.clicked.connect(self.send_path)
        self.rules = Rules(self.ui)
        self.shell = Shell(self.ui, self.rules.get_rules())
        self.ui.send_path.clicked.disconnect()
        self.ui.send_path.clicked.connect(self.send_path)
    
    def send_path(self):
        self.sender = SendPath(self.rules.get_rules())
        message = self.sender.send(self.shell.get_path())
        if message == "success":
            self.shell.clear_path()
        self.shell.append_text_plain(message)
        RunCpp().run()

    