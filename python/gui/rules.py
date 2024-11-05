from PySide6.QtGui import QFont, QStandardItemModel, QStandardItem
from gui.ui_form import Ui_MainWindow

class Rules:
    def __init__(self, ui: Ui_MainWindow) -> None:
        self.ui = ui
        self.rules = []
        self.attributes = []
        self.longest_rule = 0
        self.setup()

    def setup(self):
        self.model = QStandardItemModel()
        self.ui.rules.setModel(self.model)

        font = QFont("Courier", 10)
        self.ui.rules.setFont(font)

        self.define_rules()
        self.define_attribute()
        self.longest_rule = self.find_longest_rule_with_attribute()
        self.display_rules()

    def define_rules(self) -> None:
        self.rules.append(("move", 0))
        self.rules.append(("turn", 1))
        self.rules.append(("delete", 2))
        self.rules.append(("help", 3))
        self.rules.append(("clear", 4))
        self.rules.append(("save", 5))
        self.rules.append(("load", 6))

    def define_attribute(self) -> None:
        self.attributes.append([0, "-fw", "Move the robot forward x amount of cm",12])
        self.attributes.append([0, "-bw", "Move the robot backward x amount of cm", 13])
        self.attributes.append([1, "-l", "Turn the robot left x amount of degrees", 15])
        self.attributes.append([1, "-r", "Turn the robot right x amount of degrees", 14])
        self.attributes.append([2, "-d", "Delete the x index command from the path list", 30])
        self.attributes.append([2, "-i", "Delete the x to y index command from the path list", 31])
        self.attributes.append([2, "-a", "Delete all commands from the path", 32])
        self.attributes.append([3, "", "Display the help menu or specify command", 40])
        self.attributes.append([4, "", "Clear the command prompt", 50])
        self.attributes.append([5, "", "Save the current path to a file x", 60])
        self.attributes.append([6, "", "Load a path from a file x", 70])

    def find_longest_rule_with_attribute(self) -> int:
        longest_combined_length = 0

        for attribute in self.attributes:
            rule_index = attribute[0]          
            attribute_suffix = attribute[1]    
            rule_name = next((rule[0] for rule in self.rules if rule[1] == rule_index), None)
            
            if rule_name is not None:
                combined_name = rule_name + attribute_suffix
                
                if len(combined_name) > longest_combined_length:
                    longest_combined_length = len(combined_name)
        return longest_combined_length

    def add_item(self, text):
        item = QStandardItem(text)
        item.setEditable(False)
        self.model.appendRow(item)

    def display_rules(self) -> None:
        padding = " " * 20
        self.add_item("")
        for attribute in self.attributes:
            rule_index = attribute[0]
            attribute_suffix = attribute[1]
            description = attribute[2]

            # Get the corresponding rule name
            rule_name = next(rule[0] for rule in self.rules if rule[1] == rule_index)
            
            # Combine rule name and attribute with padding
            combined_rule = (rule_name + " " + attribute_suffix).ljust(self.longest_rule + 5)
            
            # Format and display as "rule attribute{padding_rulename}{padding}attribute[i][2]"
            display_text = f"{combined_rule} {padding} {description}"
            self.add_item(display_text)
            self.add_item("")

    def get_rules(self) -> tuple:
        return self.rules, self.attributes