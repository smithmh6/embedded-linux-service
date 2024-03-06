/* Copyright 2023
 * Thorlabs Spectral Works
 * Author: Heath Smith
 * Email: hsmith@thorlabs.com
 */

// C system
#include <modbus.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <linux/i2c.h>
#include <unistd.h>
extern "C" {
  #include <linux/i2c-dev.h>
  #include <i2c/smbus.h>
}

// C++ system
#include <stdio.h>
#include <cerrno>
#include <cstddef>
#include <cstring>
#include <iostream>
#include <string>
#include <csignal>  // using std::signal
#include <bitset>

void print_bytes(__u8 *val, int n_bytes) {
  std::cout << "0x";
  for (int i = 0; i < n_bytes; i++) {
    printf("%02X", static_cast<int>(val[i]));
  }
}

int main(int argc, char* argv[]) {
    // int r = -1;  // used for checking i2c status

    int f;  // file descriptor

    char fname[] = "/dev/i2c-0";  // bus name
    if ((f = open(fname, O_RDWR)) < 0) {
      /* ERROR HANDLING: you can check errno to see what went wrong */
      std::cout << "[ERR " << errno << "] Unable to open bus ---> ";
      std::cout << std::strerror(errno) << std::endl;
      return -1;
    }

    /* To use this properly, zero pad the address on the left and store it as 0b:00101001.
    The calls to read and write after the ioctl will automatically set the proper read
    and write bit when signaling the peripheral.*/
    int addr = 0x22;
    if (ioctl(f, I2C_SLAVE, addr) < 0) {
      std::cout << "[ERR " << errno << "] Unable to initiate comm --> ";
      std::cout << std::strerror(errno) << std::endl;
      return -1;
    }

    // write 32-bit value to register
    unsigned char comm = 0x10;

    // spin at 20% full speed
    // unsigned char buf[] = { 0x00, 0xEC, 0x00, 0x00, 0x99, 0x99 };

    // set to 0% speed
    unsigned char buf[] = { 0x00, 0xEC, 0x00, 0x00, 0x00, 0x80 };

    // r = mcf8315a::write_32bit(f, buf, sizeof(buf));
    if (i2c_smbus_write_i2c_block_data(f, comm, sizeof(buf), buf) < 0) {
        std::cout << "[ERR " << errno << "] ---> ";
        std::cout << std::strerror(errno) << std::endl;
    }

    close(f);

    return 0;
}
