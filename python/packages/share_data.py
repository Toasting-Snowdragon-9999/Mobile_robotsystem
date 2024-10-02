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
        # Convert to binary and pad to 4 bits
        return format(value, '04b')  # Format to 4-bit binary string

    def array_to_bit(self, sequence: list) -> str:
        sequence_string = ""
        for a in sequence:
            sequence_string += self.int_to_bit(a)  # No need for range(len(sequence))
        return sequence_string

    def calculate_bit_string(self, sequence: list[list]) -> str:
        sequence_string = ""
        for a in sequence:
            sequence_string += self.array_to_bit(a)  # No need for range(len(sequence))
        return sequence_string

