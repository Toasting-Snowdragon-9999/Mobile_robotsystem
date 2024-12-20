import os, json, sys
from gui.share_data import SharedData

class SendPath:
    def __init__(self, rules: list) -> None:
        self.shared_data = SharedData("..//Docs/shared_file.txt", "..//Docs")
        self.rules, self.attr = rules

    def get_attr_and_distance(self, path: list) -> list:
        sequence = []
        for command in path:
            attr = command[1]
            number = command[2]
            numbers = [attr, number]
            sequence.append(numbers)
        return sequence
    
    def save_command_to_json(self, path) -> None:
        filename = "../../Docs/shared_file"
        filename = filename + ".json"
        file_path = os.path.join("saved_paths", filename)
        with open(file_path, "w") as f:
            json.dump(path, f, indent=4)  # indent=4 for pretty printing
    
    def send(self, path) -> str: 
        if path is not []:
            commands = self.get_attr_and_distance(path)
            print(commands)
            self.save_command_to_json(commands)
            print("success")
            return "success"

    
    
    # def send(self, path) -> str: 
    #     if path is not []:
    #         numbers = self.convert_to_number(path)
    #         bits = self.shared_data.calculate_bit_string(numbers)
    #         value = self.shared_data.bits_to_int(bits)
    #         print(value)
    #         try:
    #             self.shared_data.save_to_file(value)
    #             print(numbers)
    #             print("success")
    #             return "success"

    #         except SharedData.SharedDataException as e:
    #             print(f"Error: {e}")
    #             return "Error: {e}"

    # def convert_to_number(self, path: list) -> list:
    #     sequence = []
    #     for command in path:
    #         numbers = []
    #         attribute_index = command[1]
    #         number = next((attribute[3] for attribute in self.attr if attribute[1] == attribute_index), None)
    #         if number is not None:
    #             numbers.append(number)
    #             first, second = self.split_number(command[2])
    #             numbers.append(first)
    #             numbers.append(second)
    #             sequence.append(numbers)
    #     return sequence
    
    # def split_number(self, number) -> tuple:
    #     if 1 <= number <= 99:
    #         tens = number // 10  # Tens place
    #         units = number % 10  # Units place
    #         return tens, units
    #     else:
    #         raise ValueError("Number must be between 1 and 99")