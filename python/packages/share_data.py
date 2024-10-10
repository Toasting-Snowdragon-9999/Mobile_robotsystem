import os

class SharedData:
    class SharedDataException(Exception):
        def __init__(self, message, error_code=None) -> None:
            super().__init__(message)  # Call the base class constructor with the message
            self.error_code = error_code  # Optional error code attribute

        def __str__(self) -> tuple:
            if self.error_code:
                return f"{self.args[0]} (Error code: {self.error_code})"
            return self.args[0]

    def __init__(self) -> None:
        pass

    def compute_bin(self, sequence):
        bin_dict = {
            1: 0b0001,
            2: 0b0010,
            3: 0b0011,
            4: 0b0100,
            5: 0b0101,
            6: 0b0110,
            7: 0b0111,
            8: 0b1000,
            9: 0b1001,
            10: 0b1010,
            11: 0b1011,
            12: 0b1100,
            13: 0b1101,
            14: 0b1110,
            15: 0b1111,
            16: 0b0000,
        }

        complete_bin = 0

        for inner_list in sequence:
            mapped_inner_list = [bin_dict[number] for number in inner_list if number in bin_dict]
            for value in mapped_inner_list:
                complete_bin = (complete_bin << 4) | value  # Shift and concatenate
        print(f"Concatenated binary result: {bin(complete_bin)}")  # Print the binary result
        print(type(complete_bin))
        return complete_bin

    def save_to_file(self, value: int) -> None:
        if os.path.exists("Docs") == False:
            raise self.SharedDataException("File not found", 10)
        if os.path.getsize("Docs/shared_file.txt") > 0:
            raise self.SharedDataException("File not empty, other applitcation has not read the file", 11)
        value_in_string = str(value)
        f = open("Docs/shared_file.txt", "w")
        f.write(value_in_string)
        f.close()

    def bits_to_int(self, bits: str) -> int:
        return int(bits, 2)

    def int_to_bit(self, value: int) -> str:
        four_bit_format = '04b'
        return format(value, four_bit_format)

    def array_to_bit(self, sequence: list) -> str:
        sequence_string = ""
        for a in sequence:
            sequence_string += self.int_to_bit(a)
        return sequence_string

    def calculate_bit_string(self, sequence: list[list]) -> str:
        if len(sequence) > 10:
            raise self.SharedDataException("Limit of commands is exceeded")
        sequence_string = ""
        for a in sequence:
            sequence_string += self.array_to_bit(a)
        return sequence_string


