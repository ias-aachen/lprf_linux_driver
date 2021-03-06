import argparse

if __name__ == "__main__":
    
    parser = argparse.ArgumentParser(description="""
                                     This Script allows to write binary data to /dev/lprf. When called
                                     with no arguments some default data will be used. You can specify
                                     specific data as a list of integer values or as a file. Note that
                                     you migth need to have administrator rights to run this script.""")
    parser.add_argument('-d', '--data', type=int, default=None, nargs='*',
                        help="""List of integer values to write to /dev/lprf. If not specified 
                        a file or the default data will be used.""")
    parser.add_argument('-f', '--file', default=None,
                        help="""Input file to write to to /dev/lprf. If not specified the 
                        default data will be used.""")
    parser.add_argument('-n', type=int, default=1, 
                        help="number of times the input data will be written to /dev/lprf")
    
    args = parser.parse_args()
    
    
    if args.data != None:
        data = bytearray(args.data)
    elif args.file != None:
        input_file = open(args.file, 'rb')
        data = input_file.read();
        input_file.close()
    else:
        data = bytearray([0x61, 0x88, 0x01, 0xad, 0xde, 0xef, 0xbe, 0x42, 0x42, 0x00, 0x0a, 0x00, 0x00, 0xab, 0xab, 0xab, 0xab, 0xab, 0xab, 0x55, 0xa3])


    lprf = open("/dev/lprf", 'wb')

    for i in range(args.n):
        lprf.write(data)
        lprf.flush()
        print(str(i) + ": Wrote " + str(len(data)) + " bytes to /dev/lprf.")
    
    lprf.close()
    