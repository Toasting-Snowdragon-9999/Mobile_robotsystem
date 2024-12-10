from PySide6.QtGui import QFont, QStandardItemModel, QStandardItem, QColor
from gui.ui_form import Ui_MainWindow

class PathGenerator:

    def __init__ (self, ui: Ui_MainWindow) -> None:
        self.ui = ui
        self.path = []
        self.index = 0
        self.ui.path_overview.setReadOnly(True) 
        self.beginning()

    def add_path(self, command: list, add_to_path: bool = True) -> None:
        if add_to_path:
            self.path.append(command)
        arrow, move, unit = self.decode(command[1])
        padding = "&nbsp;" * 6
        text = f'<span style="color: #57c979;">#{self.index}{padding}</span> ' \
            f'<span style="color: #4c89c7;">{arrow}</span> &nbsp; <span style="color: white;">{move} {command[2]}{unit}</span>'
        self.add_item(text)

    def add_item(self, text: str):
        self.ui.path_overview.insertHtml(text)
        self.ui.path_overview.append("")
        self.index += 1

    def decode(self, command: str) -> str:
        if command == "-fw":
            return "↑", "Move forward", " cm"
        if command == "-bw":    
            return "↓", "Mow backwards", " cm"
        if command == "-l":
            return "←", "Turn left", "\u00B0"
        if command == "-r":
            return "→", "Turn right", "\u00B0"
    
    def add_whole_path(self, command: list) -> None:
        for com in command:
            self.add_path(com, False) 
    
    def delete_path(self, index: int,  stop: int = 0) -> None:
        self.ui.path_overview.clear()
        self.index: int = 0
        self.beginning()
        if index == -1:
            self.path.clear()
            return
        if index == 1: 
            self.path.pop(-1)
        else: 
            self.index = 0
            del self.path[index:stop]
        self.add_whole_path(self.path)

    def reset(self):
        self.ui.path_overview.clear()
        self.index = 0
        self.beginning()
        self.path.clear()


    def beginning(self):
        self.ui.path_overview.append("\n")
    
    
        
