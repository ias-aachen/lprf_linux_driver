import argparse
import socket
import subprocess

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="""
                                     Receive incoming UDP data""")
    parser.add_argument('rpi', type=int,
                        help="""Number of Raspberry Pi. Should be either 1 or 2.""")
    parser.add_argument('-p', '--port', type=int, default=9000,
                        help="""Port to be used (default 9000)""")
    args = parser.parse_args()
    
    if args.rpi == 1:
        interface_ip = "fe80::e8f4:6683:61e0:7c01"
    else:
        interface_ip = "fe80::e8f4:6683:61e0:7c02"
    
    port = args.port
    
    # Get interface number
    output = subprocess.check_output("ip link show", shell=True)
    for line in output.split("\n"):
        if "lowpan" in line:
            interface_number = int(line.split(":")[0])
    
    # Open and bind socket
    s = socket.socket(socket.AF_INET6, socket.SOCK_DGRAM)
    s.bind((interface_ip, port, 0, interface_number))
    counter = 1
    
    # Receive Packets
    while True:
        data, addr = s.recvfrom(1024)
        print(str(counter) + ': Received "' + data + '" from ' + addr[0])
        counter += 1