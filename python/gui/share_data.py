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

    def __init__(self, path, docs_path) -> None:
        self.path = path    
        self.docs_path = docs_path

    def save_to_file(self, value: int) -> None:
        if os.path.exists(self.docs_path) == False:
            raise self.SharedDataException("File not found", 10)
        if os.path.getsize(self.path) > 1:
            print("Size of file = ", os.path.getsize(self.path))
            raise self.SharedDataException("File not empty, other applitcation has not read the file", 11)
        value_in_string = str(value)
        f = open(self.path, "w")
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


