from packages.share_data import SharedData

def main(): 
    data = [[11, 4, 1], [12, 7, 2], [14, 0, 0], [13, 0, 0], [11, 8, 0]]
    shared_data = SharedData()
    # bits= '101101000001110001110010111000000000110100000000101101110001'
    bits = shared_data.calculate_bit_string(data)
    value = shared_data.bits_to_int(bits)
    print (f"Value: {value}")
    try:
        shared_data.save_to_file(value)
    except SharedData.SharedDataException as e:
        print(f"Error: {e}")

if __name__ == '__main__':
    main()
