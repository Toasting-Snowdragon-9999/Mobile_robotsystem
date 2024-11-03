import os, sys
import json
from PySide6.QtGui import QTextCursor
from PySide6.QtCore import Qt
from PySide6.QtWidgets import QLabel, QLineEdit
from ui_form import Ui_MainWindow
import re

from path import PathGenerator
from rules import Rules

class Shell:
    def __init__(self, ui: Ui_MainWindow, rules: tuple) -> None:
        self.ui = ui
        self.history = []
        self.username = "user"
        #self.default_prompt_text = f"{self.username}@local: ~/ "  # Plain text prompt for logic
        self.default_prompt_text = ""  # Empty prompt for styling
        self.colored_prompt_html = f'<span style="color: #57c979;">{self.username}@local</span>: <span style="color: #4c89c7;">~/ </span>'
        self.rules, self.attributes = rules
        self.path_gen = PathGenerator(ui)
        self.setup()

    def setup(self):
        # Create a QLabel overlay for the prompt text
        self.prompt_label = QLabel(self.ui.shell_command_field)
        self.prompt_label.setText(self.colored_prompt_html)
        self.prompt_label.setGeometry(-101, -4, 250, self.ui.shell_command_field.height())  # Adjust width as needed
        self.prompt_label.setAttribute(Qt.WA_TransparentForMouseEvents)

        self.ui.shell_command_field.setStyleSheet("padding-left: 102px;")  # Adjust padding to match prompt width
        self.ui.shell_command_field.returnPressed.connect(self.store_command)
        self.ui.shell_command_field.textChanged.connect(self.enforce_prompt)

    def enforce_prompt(self):
        current_text = self.ui.shell_command_field.text()

        if not current_text.startswith(self.default_prompt_text):
            self.ui.shell_command_field.setText(self.default_prompt_text)
        
        self.ui.shell_command_field.setCursorPosition(len(current_text))

    def store_command(self):
        full_command = self.ui.shell_command_field.text()[len(self.default_prompt_text):]  # Strip prompt
        if full_command == "clear":
            self.ui.shell_text_field.clear()
            self.reset_command_field()
            return
        
        if self.search_for_help(full_command) is not None:
            first_word, second_word = self.search_for_help(full_command)
            if first_word == "help" or first_word == "Help":
                self.show_specific_help(second_word)
                self.reset_command_field()
                return
            if first_word == "load" or first_word == "load ":
                self.load(second_word)
                self.reset_command_field()
                return
            if first_word == "save" or first_word == "save ":
                self.save(second_word)
                self.reset_command_field()
                return

        if full_command == "help"  or full_command == "help ":
            self.show_help()
            self.reset_command_field()
            return
        command, attr, distance, ekstr = self.search_for_rule(full_command)
        print(f"Command: {command}, attr: {attr}, Distance: {distance}, ekstra: {ekstr}")
        self.append_text(full_command)

        for rule_index, rule in enumerate(self.rules):
            if rule[0] == command:
                break
        if not self.check_for_rule(command) or not self.check_for_attribute(attr, rule_index):
            error = f"Command not found: {command}"
            print(error)
            if distance == 0:
                error = f"Command not found: {full_command}"
            self.append_text_error(error)
            self.reset_command_field()
            return
        if command == "delete" or command == "delete ":
            self.delete(attr, distance, ekstr)
            print("Are we here in shell?")
            self.reset_command_field()
            return
        self.history.append([command, attr, distance])
        self.path_gen.add_path([command, attr, distance])
        self.reset_command_field()

    def reset_command_field(self):
        self.ui.shell_command_field.setText(self.default_prompt_text)
        self.ui.shell_command_field.setCursorPosition(len(self.default_prompt_text))

    def add_padding(self):
        line_height = self.ui.shell_text_field.fontMetrics().lineSpacing()
        widget_height = self.ui.shell_text_field.height()
        visible_lines = widget_height // line_height
        
        current_line_count = self.ui.shell_text_field.document().blockCount()
        padding_lines = visible_lines - current_line_count

        if padding_lines > 0:
            self.ui.shell_text_field.append('\n' * padding_lines)

    def append_text(self, text):

        self.add_padding()
        
        to_append = f'<span style="color: #57c979;">{self.username}@local</span>: <span style="color: #4c89c7;">~/ </span> {text}'
        self.ui.shell_text_field.append(to_append)
        
        cursor = self.ui.shell_text_field.textCursor()
        cursor.movePosition(QTextCursor.End)
        self.ui.shell_text_field.setTextCursor(cursor)
        self.ui.shell_text_field.ensureCursorVisible()

    def append_text_plain(self, text):
        self.add_padding()
        self.ui.shell_text_field.append(text)
        
        cursor = self.ui.shell_text_field.textCursor()
        cursor.movePosition(QTextCursor.End)
        self.ui.shell_text_field.setTextCursor(cursor)
        self.ui.shell_text_field.ensureCursorVisible()
    
    def append_text_error(self, text):

        self.add_padding()
        
        to_append = f'<span style="color: #b54343;">[ Error ]</span>:  {text}'
        self.ui.shell_text_field.append(to_append)
        
        cursor = self.ui.shell_text_field.textCursor()
        cursor.movePosition(QTextCursor.End)
        self.ui.shell_text_field.setTextCursor(cursor)
        self.ui.shell_text_field.ensureCursorVisible()

    def append_text_help(self, text):

        self.add_padding()
        
        to_append = f'<span style="color: #dede6d;">[ Help ]</span>: {text}'
        self.ui.shell_text_field.append(to_append)
        
        cursor = self.ui.shell_text_field.textCursor()
        cursor.movePosition(QTextCursor.End)
        self.ui.shell_text_field.setTextCursor(cursor)
        self.ui.shell_text_field.ensureCursorVisible()

    def delete(self, attr, start, stop):
        if attr == "-d":
            self.history.pop(-1)
            self.path_gen.delete_path(1)
            return
        elif attr == "-i":
            if stop is not None and start is not None:
                del self.history[start:stop+1]
                self.path_gen.delete_path(start, stop+1)
                return
        elif attr == "-a":
            self.path_gen.delete_path(-1)
            self.history.clear()
            return

    def execute(self):
        pass
    
    def search_for_help(self, help_text: str):
        match = re.match(r"(\w+)\s+(\w+)", help_text)
        if match:
            first_word = match.group(1)
            second_word = match.group(2)
            return first_word, second_word
        return None

    def search_for_rule(self, rule_text: str):
        match = re.match(r"(\w+)(?:\s+(-\w+))?(?:\s+(\d+))?(?:\s+(\d+))?", rule_text)
        command = None
        attr = None
        distance = None
        ekstr = None

        if match:
            command = match.group(1)         # The main command (e.g., "move", "delete", "help")
            attr = match.group(2)            # Optional attribute (e.g., "-fw", "-i")
            distance = match.group(3)        # Optional first number (e.g., "10")
            ekstr = match.group(4)           # Optional second number (e.g., "12")

        if distance is not None:
            distance = int(distance)
        if ekstr is not None:
            ekstr = int(ekstr)

        return command, attr, distance, ekstr

    def change_username(self, username: str) -> None:
        self.username = username
        self.default_prompt = f"{self.username}@local: "
        self.colored_prompt_html = f'<span style="color: #57c979;">{self.username}@local</span>: <span style="color: #4c89c7;">~/ </span>'
        self.prompt_label.setText(self.colored_prompt_html)

    def check_for_rule(self, rule_name: str) -> bool:
        if rule_name in [rule[0] for rule in self.rules]:
            return True
        else:
            return False
    
    def check_for_attribute(self, attr_name: str, rule_index: int) -> bool:
        for attr in self.attributes:
            if attr[0] == rule_index and attr[1] == attr_name:
                return True
        return False

    def save(self, filename="history"):
        filename = filename + ".json"
        path = os.path.join("saved_paths", filename)
        with open(path, "w") as f:
            json.dump(self.history, f, indent=4)  # indent=4 for pretty printing
        self.append_text_plain(f"Successfully saved path to {filename}")

    def load(self, filename="history"):
        filename = filename + ".json"
        path = os.path.join("saved_paths", filename)
        try:
            with open(path, "r") as f:
                self.history = json.load(f)
                self.path_gen.reset()
                self.path_gen.add_whole_path(self.history)
                self.append_text_plain(f"Path successfully loaded from {filename}")
            print(f"History loaded from {path}")
        except FileNotFoundError:
            self.append_text_error(f"No saved history found at {path}.")
        except json.JSONDecodeError:
            self.append_text_error(f"Error: The file at {path} is not a valid JSON file.")

    def show_specific_help(self, command: str):
        help_text = "Command invalid"
        if command == "move":
            help_text = """
<pre>
To use the command it should be in the following format:
    move -fw &lt;x&gt;         # Move forward x amount of cm
    move -bw &lt;x&gt;         # Move backwards x amount of cm
This will then make the robot move in the direction it is facing
</pre>
        """
        if command == "turn":
            help_text = """
<pre>
To use the command it should be in the following format:
    turn -l  &lt;x&gt;         # Turn left x amount of degrees
    turn -r  &lt;x&gt;         # Turn right x amount of degrees
This will then make the robot turn in the direction it is facing
</pre>
        """
        if command == "delete":
            help_text = """
<pre>
To use the command it should be in the following format:

    delete -d &lt;x&gt;        # Delete the x command
This will effectivly delete the last added command

    delete -i &lt;x&gt; &lt;y&gt;    # Delete the x to y command
This is inteded to delete a range of commands 
this uses 0 indexing so the first command is 0

    delete -a            # Delete all commands
This will delete all the commands in the current path
</pre>
        """
        if command == "save":
            help_text = """
<pre>
To use the command it should be in the following format:    
        save &lt;file name&gt;
This will save to a file with the name you specify
located in the folder called "saved_paths"
</pre>
        """
        if command == "load":
            help_text = """
<pre>
To use the command it should be in the following format:    
        load &lt;file name&gt;
This will load a file with the name you specify 
located in the folder called "saved_paths"
</pre>
        """
        if command == "clear":
            help_text = """
<pre>
    clear  # Clear the terminal
This will clear the terminal of all text
</pre>
    """
        self.append_text_help(help_text)


    def show_help(self):
        help_text = """
<pre>
To use a command it should be in the following format:

    move -fw &lt;x&gt;         # Move forward x amount of cm
    move -bw &lt;x&gt;         # Move backwards x amount of cm
    turn -l  &lt;x&gt;         # Turn left x amount of degrees
    turn -r  &lt;x&gt;         # Turn right x amount of degrees
    delete -d &lt;x&gt;        # Delete the x command
    delete -i &lt;x&gt; &lt;y&gt;    # Delete the x to y command
    delete -a            # Delete all commands
    save &lt;file name&gt;     # Save the current path to a file
    load &lt;file name&gt;     # Load a path from a file
    help &lt;command&gt;       # Display help for a specific command
    clear                # Clear the terminal

Now you can use clear to delete this help menu and start using the commands
</pre>
        """
        self.append_text_help(help_text)
