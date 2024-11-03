# This Python file uses the following encoding: utf-8
import os
from pathlib import Path
import sys

from shell import Shell
from rules import Rules

from PySide6.QtWidgets import QApplication, QMainWindow
from PySide6.QtCore import QFile
# Important:
# You need to run the following command to generate the ui_form.py file
#     pyside6-uic form.ui -o ui_form.py, or
#     pyside2-uic form.ui -o ui_form.py
from ui_form import Ui_MainWindow

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

    def set_up_rules(self):
        return 
    
    def send_path(self):
        return

if __name__ == "__main__":
    app = QApplication([])
    widget = MainWindow()
    widget.show()
    sys.exit(app.exec())
