from packages.share_data import SharedData

def main(): 
    data = [
        [11, 8, 5],
        [14, 7, 7],
        [11, 9, 9],
        [13, 6, 9],
        [11, 1, 0],
        [14, 3, 0],
        [11, 0, 2],
        [13, 9, 1],
        [11, 2, 3],
        [14, 8, 3]
    ]
    shared_data = SharedData()
    bits = shared_data.calculate_bit_string(data)
    value = shared_data.bits_to_int(bits)
    print (f"Value: {value}")
    try:
        shared_data.save_to_file(value)
        bits2 = shared_data.int_to_bit(value)
        value2 = shared_data.bits_to_int(bits2)
        if bits2 == bits and value == value2:
            print("success")
        else:
            print("failure")
    except SharedData.SharedDataException as e:
        print(f"Error: {e}")

if __name__ == '__main__':
    main()
