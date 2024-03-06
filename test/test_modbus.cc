/* Copyright 2023
 * Thorlabs Spectral Works
 * Author: Heath Smith
 * Email: hsmith@thorlabs.com
 */

#include <math.h>
#include <cerrno>
#include <modbus.h>
#include <cstddef>
#include <cstring>
#include <iostream>
#include <unistd.h>

#define MICROSECOND 1000000

int main(int argc, char* argv[]) {

    //int s = -1;
    int nb = 8;
    //uint8_t *dest = new uint8_t[nb*sizeof(uint8_t)];
    uint16_t *reg_dest = new uint16_t[nb*sizeof(uint16_t)];
    // for (size_t i = 0; i < 8; i++) {
    //     dest[i] = '1';
    // }
    // //printf("%s", dest);
    // for (size_t i = 0; i < 8*sizeof(unsigned char); i++) {
    //     std::cout << "[" << i << "] -->" << dest[i] << std::endl;
    // }

    modbus_t *ctx = NULL;

    // 127.0.0.1 -> localhost
    ctx = modbus_new_tcp("10.59.10.58", 1502);
    modbus_set_debug(ctx, true);
    if (ctx == NULL) {
        std::cout << "Unable to allocate libmodbus context" << std::endl;
        modbus_free(ctx);
        return -1;
    }

    /* modbus_connect() is used to connect to a server*/
    if (modbus_connect(ctx) == -1) {
        //std::cout << "Connection failed" << std::endl;
        fprintf(stderr, "Connection failed: %s\n", modbus_strerror(errno));
        modbus_free(ctx);
        return -1;
    }

    // write a bit to a register
    // std::cout << "Writing 1 to register address 10001" << std::endl;
    // rc = modbus_write_bit(ctx, 0, 1);
    // if (rc == -1) {
    //     fprintf(stderr, "Failed to write bits: %s\n", modbus_strerror(errno));
    //     modbus_free(ctx);
    //     return -1;
    // }

	int gain = 1;
	//double adc_scale = ((2.4 / gain) / pow(2.0, 24.0));

    while (1) {
        //int bits_read = modbus_read_input_bits(ctx, 10001, nb, dest);
        // read register space 30000
        int r = modbus_read_input_registers(ctx, 0, 4, reg_dest);
        if (r == -1) {
            fprintf(stderr, "Read input bits failed: %s\n", modbus_strerror(errno));
            modbus_free(ctx);
            return -1;
        }

        std::cout << "Read ---> " << r << " registers" << std::endl;

        printf("0x%04X,0x%04X,0x%04X,0x%04X\n", reg_dest[0], reg_dest[1], reg_dest[2], reg_dest[3]);

        uint64_t reg0 = (uint64_t) reg_dest[0] << 48;
        uint64_t reg1 = (uint64_t) reg_dest[1] << 32;
        uint64_t reg2 = (uint64_t) reg_dest[2] << 16;
        uint64_t reg3 = (uint64_t) reg_dest[3];


        uint64_t ch0 = reg0 | reg1 | reg2 | reg3;

        double ch0_f;
        memcpy(&ch0_f, &ch0, sizeof(uint64_t));

        printf("%f\n", ch0_f);

        usleep(1 * MICROSECOND);
    }


    modbus_close(ctx);
    modbus_free(ctx);

    return 0;
}