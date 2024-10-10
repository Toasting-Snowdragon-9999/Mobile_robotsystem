import socket
import time

class SocketConnect: 
    def __init__(self):
        self.client_socket = None  # Initialize to None

    def connect(self):
        """
        This method make a client socket connection on port 62908, and using the ip = 127.0.0.1 for local connection.
        """
        try:
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            port = 62908  # Randomly chosen port
            ip = '127.0.0.1'
            self.client_socket.connect((ip, port))
            print("Connected to server.")
        except socket.error as e:
            print(f"Connection error: {e}")
            self.client_socket = None

    def send_data(self, data: int):
        """
        This method sends data using the client socket connection created earlier,
        the data should be without any start/stop bit or checksum, this will be applied in this method.
        Using CRC checksum and 1001 as start/stop bit.
        """
        if self.client_socket is None:
            print("Socket not connected. Call connect() first.")
            return

        validated_data = self.add_validation(data)
        num_bytes = (validated_data.bit_length() + 7) // 8  # This will be 1 byte, as bit_length is 8

        # Convert to bytes
        byte_data = validated_data.to_bytes(num_bytes, byteorder='big')
        print("num of bytes: ", num_bytes)
        if validated_data:
            try:
                self.client_socket.sendall(byte_data)
                print(f"Sent: {byte_data}")
                self.wait_for_socket()
            except socket.error as e:
                print(f"Send error: {e}")
            finally:
                self.client_socket.close()
                print("Connection closed.")

    def add_validation(self, data: int):
        start_bit = 0b11111110
        stop_bit = 0b11111101
        checkbyte = self.checksum(data)
        data_length = data.bit_length()
        if data_length % 8 != 0:
            print("Warning: Data cannot be evenly split into 8-bit parts.")
            compensation = 4
            start_bit = start_bit << (data_length + compensation)
        else:
            start_bit = start_bit << data_length
        data = start_bit | data
        # print(f"after con {bin(data)}")  # Print the binary result
        data = (data << checkbyte.bit_length()) | checkbyte
        data = (data << 8) | stop_bit
        # print(f"Concatenated binary result: {bin(data)}")  # Print the binary result

        return data

    def binary_division(self, data: int, key: int):
        if key == 0:
            raise ValueError("Cannot divide by zero")

        remainder = 0
        data_length = data.bit_length()

        # Perform long division
        for i in range(data_length - 1, -1, -1):
            # Bring down the next bit (left shift remainder)
            remainder = (remainder << 1) | ((data >> i) & 1)

            # Subtract key if remainder is greater than or equal to key
            if remainder >= key:
                remainder -= key

        # Print the binary result and its bit length
        print(f"Checksum: {bin(remainder)}, Bit length: {remainder.bit_length()}")
        return remainder

    def checksum(self, data):
        """
        Using x^8+x^2+x+1 as the polynomial key, and using CRC for checksum, by doing binary division between the data and the key
        """
        key = 0b100110001
        remainder = self.binary_division(data, key)
        return remainder

    def wait_for_socket(self):
        time.sleep(1)  # Wait for 1 second after sending

    def disconnect(self):
        self.client_socket.close()


