import argparse
import socket
import subprocess

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="""
                                     Configure static IP routes for Treufunk chip and send UDP data in an infinite loop.""")
    parser.add_argument('rpi', type=int,
                        help="""Number of Raspberry Pi. Should be either 1 or 2.""")
    parser.add_argument('-m', '--message', default="IAS, RWTH AACHEN",
                        help="""Payload to send via UDP (default "IAS, RWTH AACHEN")""")
    parser.add_argument('-p', '--port', type=int, default=9000,
                        help="""Port to be used (default 9000)""")
    args = parser.parse_args()
    
    if args.rpi == 1:
        dest_mac = "EA:F4:66:83:61:E0:7C:02"
        dest_ip = "fe80::e8f4:6683:61e0:7c02"
    else:
        dest_mac = "EA:F4:66:83:61:E0:7C:01"
        dest_ip = "fe80::e8f4:6683:61e0:7c01"
    
    message = args.message
    port = args.port
    
    # Configure IP routes
    try:
        subprocess.check_call("ip -6 route add " + dest_ip + " dev lowpan0", shell=True)
        subprocess.check_call("ip -6 neigh add " + dest_ip + " lladdr " + dest_mac + " dev lowpan0", shell=True)
    except:
        subprocess.check_call("ip -6 route change " + dest_ip + " dev lowpan0", shell=True)
        subprocess.check_call("ip -6 neigh change " + dest_ip + " lladdr " + dest_mac + " dev lowpan0", shell=True)
    
    
    s = socket.socket(socket.AF_INET6, socket.SOCK_DGRAM)
    
    while True:
        s.sendto(message, (dest_ip, port, 0, 0))