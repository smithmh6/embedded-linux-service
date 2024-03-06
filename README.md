# Modbus Reference
```cpp
printf("[LOG] nb_bits ---> %d\n", mb_mapping->nb_bits);
printf("[LOG] start_bits ---> %d\n", mb_mapping->start_bits);
printf("[LOG] nb_input_bits ---> %d\n", mb_mapping->nb_input_bits);
printf("[LOG] start_input_bits ---> %d\n", mb_mapping->start_input_bits);
printf("[LOG] nb_input_regs ---> %d\n", mb_mapping->nb_input_registers);
printf("[LOG] start_in_regs ---> %d\n", mb_mapping->start_input_registers);
printf("[LOG] nb_registers ---> %d\n", mb_mapping->nb_registers);
printf("[LOG] start_registers ---> %d\n", mb_mapping->start_registers);

mb_mapping->tab_bits[0] = 1;
mb_mapping->tab_bits[1] = 1;
mb_mapping->tab_input_bits[0] = 0;
mb_mapping->tab_input_bits[1] = 0;
mb_mapping->tab_input_bits[2] = 0;
mb_mapping->tab_input_bits[3] = 0;
mb_mapping->tab_input_bits[4] = 0;
mb_mapping->tab_input_bits[5] = 0;
mb_mapping->tab_input_bits[6] = 0;
mb_mapping->tab_input_bits[7] = cycle_counter_alarm;

write cycle_counter
mb_mapping->tab_input_registers[0] = cycle_counter;
```

# Code for Writing/Reading SMBUS
```cpp
// to write to i2c bus, first send 7-bit start condidition with
// target addres + 0 for write ---> 01000000
char buf[10] = {0};
char start_cond[10] = {0b01000010};
char reg_addr[10] = {0b00000011};  // 0x03 --> configuration (0=output, 1=input)
char config[10] = {0b111000};  // set pins 4-7 as outputs


// write to device
// buf[0] = 0b00000011;  // register address
// buf[1] = 0b11110000;  // pin output configuration
// buf[2] = 0b00000001;  // output port address
// buf[2] = 0b11110000;  // set pins 4-7 to logic low
__u8 reg = 0x02;
__u8 pinconfig = 0x00;
int res;
if ((res = i2c_smbus_write_byte_data(f, reg, pinconfig)) == -1) {
    std::cout << "[ERR " << errno << "] --->  " << std::strerror(errno) << std::endl;
    return -1;
}
std::cout << "Wrote pin config --> " << std::bitset<8>(pinconfig) << std::endl;


// set pin levels
__u8 pinlogic = 0x00;
reg = 0x01;
// if ((res = i2c_smbus_write_byte(f, reg)) == -1) {
if ((res = i2c_smbus_write_byte_data(f, reg, pinlogic)) == -1) {
    std::cout << "[ERR " << errno << "] --->  " << std::strerror(errno) << std::endl;
    return -1;
}
std::cout << "Wrote pin logic levels --> " << std::bitset<8>(pinlogic) << std::endl;

// read pin levels
reg = 0x00;
if ((res = i2c_smbus_read_word_data(f, reg)) == -1) {
    std::cout << "[ERR " << errno << "] --->  " << std::strerror(errno) << std::endl;
    return -1;
}

std::cout << "Read word --> " << std::bitset<8>(res) << std::endl;

// exit program before modbus
return -1;
```
