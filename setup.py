"""
setup.py runs the initial setup for MOE
Gen3 Optical Engine.

References
----------
https://pymodbustcp.readthedocs.io/en/latest/index.html
"""

import fcntl
import requests
import socket
import struct
from pyModbusTCP.client import ModbusClient
from pyModbusTCP.utils import decode_ieee, word_list_to_long

class FloatModbusClient(ModbusClient):
    """A ModbusClient class with float support."""

    def read_input_float_32(self, address, number):
        """Read 32-bit float(s) with read input registers."""
        reg_list = self.read_input_registers(address, number*2)
        if reg_list:
            return [decode_ieee(f) for f in word_list_to_long(reg_list)]
        else:
            return None

    def read_input_float_64(self, address, number):
        """Read 64-bit float(s) with read input registers."""
        reg_list = self.read_input_registers(address, number*4)
        if reg_list:
            return [decode_ieee(f, double=True) for f in word_list_to_long(reg_list, long_long=True)]
        else:
            return None

def get_ip_address(ifname: str) -> (str | None):
    """
    Retrieve the IP address of a network interface.

    Args
    ----------
    ifname (str) : the name of the network interface (ie: 'eth0')

    Returns
    ----------
    (str) the IP address of the network interface.

    References
    ----------
    https://stackoverflow.com/questions/24196932/how-can-i-get-the-ip-address-from-a-nic-network-interface-controller-in-python
    """

    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        _fd = s.fileno()
        _request = 0x8915  # SIOCGIFADDR
        _arg = struct.pack('256s', bytes(ifname[:15], 'utf-8'))

        # Perform the operation 'request' on file descriptor 'fd'
        _packed_ip = fcntl.ioctl(_fd, _request, _arg)[20:24]

        # convert the buffer to a string
        ip_str = socket.inet_ntoa(_packed_ip)

        return ip_str
    except OSError:
        return None

def session_for_src_addr(addr: str) -> requests.Session:
    """
    Create `Session` which will bind to the specified local address
    rather than auto-selecting it.
    """
    session = requests.Session()
    for prefix in ('http://', 'https://'):
        session.get_adapter(prefix).init_poolmanager(
            # those are default values from HTTPAdapter's constructor
            connections=requests.adapters.DEFAULT_POOLSIZE,
            maxsize=requests.adapters.DEFAULT_POOLSIZE,
            # This should be a tuple of (address, port). Port 0 means auto-selection.
            source_address=(addr, 0),
        )

    return session

def main():
    """
    Main setup script.
    """
    print('-'*80)
    print("MOE Gen3 Optical Engine - Setup")
    print('-'*80)
    print("\n")

    # get IP addresses of network interfaces
    print('-'*50)
    print("Interface\tIP Address")
    print('-'*50)
    eth1 = get_ip_address('enp1s0')

    eth2 = get_ip_address('enp2s0')

    print(f"enp1s0\t\t{eth1}\nenp2s0\t\t{eth2}")
    print('-'*50)

    # check eth1 network connection
    if eth1:
        print(f"\nTesting outbound connection on {eth1}...")
        s = session_for_src_addr(eth1)
        response = s.get(f"https://www.google.com/")
        if response.status_code == 200:
            print("Success!")
        else:
            print(f"{eth1} Unable to connect, expected STATUS=200, received {response.status_code}")

    # check eth2 network connection
    if eth2:
        print(f"\nTesting outbound connection on {eth2}...")
        s = session_for_src_addr(eth2)
        response = s.get(f"https://www.google.com/")
        if response.status_code == 200:
            print("Success!")
        else:
            print(f"{eth2} Unable to connect, expected STATUS=200, received {response.status_code}")

    # initialize modbus client
    print("\nChecking modbus connection...")
    mbc = FloatModbusClient(debug=False, auto_open=True)

    # read input registers 30000 - 30023
    print("\nReading input registers...")
    in_regs_list = mbc.read_input_float_64(0, 7)

    if in_regs_list:
        print(f"\nRead {len(in_regs_list)} input registers\n")
        print('-'*50)
        print("Name\t\tValue\t\tUnit")
        print('-'*50)
        print(f"nd_filter\t{round(in_regs_list[0], 2)}\t\tOhms")
        print(f"moe1\t\t{round(in_regs_list[1], 2)}\t\tOhms")
        print(f"moe2\t\t{round(in_regs_list[2], 2)}\t\tOhms")
        print(f"H2O_conc\t{round(in_regs_list[3], 2)}\tppm")
        print(f"temp_c\t\t{round(in_regs_list[4], 2)}\t\t\u00b0C")
        print(f"hum\t\t{round(in_regs_list[5], 2)}\t\t% RH")
        print(f"press\t\t{round(in_regs_list[6], 2)}\t\tPSI")
        print('-'*50)
    else:
        print("Unable to read registers")

    print("\nReading discrete inputs...")
    in_disc_list = mbc.read_discrete_inputs(0, 12)

    if in_disc_list:
        print(f"\nRead {len(in_disc_list)} discrete inputs\n")
        print('-'*50)
        print("Name\t\t\tValue")
        print('-'*50)
        print(f"cycle_counter_alarm\t{in_disc_list[0]}")
        print(f"hw_alarm\t\t{in_disc_list[1]}")
        print(f"motor_off\t\t{in_disc_list[2]}")
        print(f"lightsource_off\t\t{in_disc_list[3]}")
        print(f"detector_off\t\t{in_disc_list[4]}")
        print(f"temp_high_alarm\t\t{in_disc_list[5]}")
        print(f"temp_low_alam\t\t{in_disc_list[6]}")
        print(f"hum_high_alarm\t\t{in_disc_list[7]}")
        print(f"H2O_conc_pred_valid\t{in_disc_list[8]}")
        print(f"oversaturated_alarm\t{in_disc_list[9]}")
        print(f"low_signal_alarm\t{in_disc_list[10]}")
        print(f"low_press_alarm\t\t{in_disc_list[11]}")
        print('-'*50)
    else:
        print(mbc.last_error_as_txt, mbc.last_except_as_full_txt)

if __name__ == "__main__":
    main()
