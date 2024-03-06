/* Copyright 2023
 * Thorlabs Spectral Works
 * Author: Heath Smith
 * Email: hsmith@thorlabs.com
 */

// C system headers
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <linux/i2c.h>
#include <modbus.h>
#include <stdlib.h>
#include <fcntl.h>
#include <math.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <yaml.h>
extern "C" {
    #include <linux/i2c-dev.h>
    #include <i2c/smbus.h>
}

// C++ system headers
#include <exception>
#include <stdexcept>
#include <thread>  // NOLINT
#include <cerrno>
#include <chrono>  // NOLINT
#include <cstddef>
#include <cstring>
#include <ctime>
#include <iostream>
#include <fstream>
#include <string>
#include <csignal>
#include <bitset>

#include <dow_gen3_embedded/inc/definitions.h>
#include <dow_gen3_embedded/inc/logger.h>

using std::chrono::high_resolution_clock;
using std::chrono::duration;
using std::chrono::duration_cast;

// exception pointer
static std::exception_ptr _exceptionPtr = nullptr;

// Logger resources
Logger logger;
char* msg_ptr;
Logger modbusLogger;
char* mb_msg_ptr;
Logger dataLogger;
char* data_msg_ptr;

// Modbus resources
static modbus_t *ctx = NULL;
static modbus_mapping_t *mb_mapping;
static int server_socket = -1;

// logging timestamp reference
time_t now;

// STD Calibration Temperature
double std_cal_temp = 33.34;
double pred_mode = 0;
calibration_settings *moe1_cal;
calibration_settings *moe2_cal;
calibration_settings *ndf_cal;
prediction_settings *h20_pred_model;

/* handle signal interupt */
static void close_sigint(int x);

/* Start the stepper motor */
int startMotor(bool _killswitch=false);

/* Poll serial device */
void serialPollingTask(adc_settings *adc_config);

/* check/set the GPIO alarm pins */
void checkLowPressureAlarm();

/* Main program loop */
int main(int argc, char* argv[]) {

  //checkLowPressureAlarm(mb_mapping);
  //return 0;

  setvbuf(stdout, NULL, _IONBF, 0);

  // set up main logger (25mb file size limit)
  logger = Logger("/var/log/thorlabs", "engine", Debug, 25);

  // setup modbus logger (100mb file size limit)
  modbusLogger = Logger("/var/log/thorlabs", "modbus", Debug, 100);

  // set up data logger
  dataLogger = Logger("/var/log/thorlabs", "data", Debug, 50);

  // read yaml config
  YAML::Node config;
  std::string ip_address = "0.0.0.0";
  int port = 502;
  int adc_gain = 1;
  adc_settings *adc_cfg;

  // set the default calibration settings
  moe1_cal = new calibration_settings();
  moe1_cal->SC2 = 3.68002396976259E-10;
  moe1_cal->SC1 = -6.67697931983353E-08;
  moe1_cal->SC0 = -0.000001302186524551;
  moe1_cal->IC2 = -0.0000276154099638092;
  moe1_cal->IC1 = 0.00343857707282065;
  moe1_cal->IC0 = 1.35453679098324;

  moe2_cal = new calibration_settings();
  moe2_cal->SC2 = 3.04033735972386E-11;
  moe2_cal->SC1 = -1.51818211835769E-08;
  moe2_cal->SC0 = 4.88035837274824E-06;
  moe2_cal->IC2 = -0.0000344399115209197;
  moe2_cal->IC1 = 0.00456297308857503;
  moe2_cal->IC0 = 1.41859422542362;

  ndf_cal = new calibration_settings();
  ndf_cal->SC2 = 0.0;
  ndf_cal->SC1 = 0.0;
  ndf_cal->SC0 = 0.0;
  ndf_cal->IC2 = 0.0;
  ndf_cal->IC1 = 0.0;
  ndf_cal->IC0 = 0.0;

  h20_pred_model = new prediction_settings();
  h20_pred_model->C9 = 0.0;                // moe1^2
  h20_pred_model->C8 = 0.0;                // moe2^2
  h20_pred_model->C7 = 0.0;                // nd^2
  h20_pred_model->C6 = -295672.244000044;  // moe1
  h20_pred_model->C5 = 11350.7084501288;   // moe2
  h20_pred_model->C4 = 0.0;                // nd
  h20_pred_model->C3 = 27536.7614307018;   // moe1 * moe2
  h20_pred_model->C2 = 0.0;                // moe1 * nd
  h20_pred_model->C1 = 0.0;                // moe2 * nd
  h20_pred_model->C0 = 347415.130895996;   // offset

  try {
    config = YAML::LoadFile("/etc/thorlabs/config.yaml");
    if (config["acquisition"]) {
      logger.Log(Info, "main() Reading acquisition settings");
      YAML::Node daq_node = config["acquisition"];

      if (daq_node["gain"]) {
        adc_gain = daq_node["gain"].as<int>();
        for (unsigned int i = 0; i < adc_entries; ++i) {
          if (adc_config_table[i].gain == adc_gain) {
            adc_cfg = &adc_config_table[i];
          }
        }
      }
    }

    if (config["modbus"]) {
      logger.Log(Info, "main() Reading modbus settings");
      YAML::Node modbus_node = config["modbus"];

      if (modbus_node["address"]) {
        ip_address = modbus_node["address"].as<std::string>();
      }

      if (modbus_node["port"]) {
        port = modbus_node["port"].as<int>();
      }
    }

    if (config["calibration"]) {
      logger.Log(Info, "main() Reading calibration settings");
      YAML::Node cal_node = config["calibration"];

      if (cal_node["temp_c"]) {
        std_cal_temp = cal_node["temp_c"].as<double>();
      }

      if (cal_node["moe1"]) {
        YAML::Node moe1_node = cal_node["moe1"];
        moe1_cal->HC2 = moe1_node["H2"].as<double>();
        moe1_cal->HC1 = moe1_node["H1"].as<double>();
        moe1_cal->HC0 = moe1_node["H0"].as<double>();
        moe1_cal->SC2 = moe1_node["S2"].as<double>();
        moe1_cal->SC1 = moe1_node["S1"].as<double>();
        moe1_cal->SC0 = moe1_node["S0"].as<double>();
        moe1_cal->IC2 = moe1_node["I2"].as<double>();
        moe1_cal->IC1 = moe1_node["I1"].as<double>();
        moe1_cal->IC0 = moe1_node["I0"].as<double>();
      }

      if (cal_node["moe2"]) {
        YAML::Node moe2_node = cal_node["moe2"];
        moe2_cal->HC2 = moe2_node["H2"].as<double>();
        moe2_cal->HC1 = moe2_node["H1"].as<double>();
        moe2_cal->HC0 = moe2_node["H0"].as<double>();
        moe2_cal->SC2 = moe2_node["S2"].as<double>();
        moe2_cal->SC1 = moe2_node["S1"].as<double>();
        moe2_cal->SC0 = moe2_node["S0"].as<double>();
        moe2_cal->IC2 = moe2_node["I2"].as<double>();
        moe2_cal->IC1 = moe2_node["I1"].as<double>();
        moe2_cal->IC0 = moe2_node["I0"].as<double>();
      }

      if (cal_node["ndf"]) {
        YAML::Node ndf_node = cal_node["ndf"];
        ndf_cal->HC2 = ndf_node["H2"].as<double>();
        ndf_cal->HC1 = ndf_node["H1"].as<double>();
        ndf_cal->HC0 = ndf_node["H0"].as<double>();
        ndf_cal->SC2 = ndf_node["S2"].as<double>();
        ndf_cal->SC1 = ndf_node["S1"].as<double>();
        ndf_cal->SC0 = ndf_node["S0"].as<double>();
        ndf_cal->IC2 = ndf_node["I2"].as<double>();
        ndf_cal->IC1 = ndf_node["I1"].as<double>();
        ndf_cal->IC0 = ndf_node["I0"].as<double>();
      }
    }

    if (config["prediction"]) {
      logger.Log(Info, "main() Reading prediction settings");
      YAML::Node pred_node = config["prediction"];

      pred_mode = pred_node["mode"].as<int>();
      h20_pred_model->C9 = pred_node["C9"].as<double>();
      h20_pred_model->C8 = pred_node["C8"].as<double>();
      h20_pred_model->C7 = pred_node["C7"].as<double>();
      h20_pred_model->C6 = pred_node["C6"].as<double>();
      h20_pred_model->C5 = pred_node["C5"].as<double>();
      h20_pred_model->C4 = pred_node["C4"].as<double>();
      h20_pred_model->C3 = pred_node["C3"].as<double>();
      h20_pred_model->C2 = pred_node["C2"].as<double>();
      h20_pred_model->C1 = pred_node["C1"].as<double>();
      h20_pred_model->C0 = pred_node["C0"].as<double>();
    }

  } catch (const YAML::BadFile& e) {
    logger.Log(Error, "main() error reading config file");
    logger.Log(Fatal, &e.msg[0]);
    return 1;
  }

  // set up modbus parameters
  int master_socket;
  int rc;
  fd_set refset;
  fd_set rdset;
  int fdmax;  // max file descriptors

  // use "0.0.0.0" for localhost
  const char *ip = ip_address.c_str();

  asprintf(&msg_ptr, "main() setting modbus context at %s:%i", ip, port);
  logger.Log(Info, msg_ptr);
  free(msg_ptr);

  ctx = modbus_new_tcp(ip, port);

  if (ctx == NULL) {
    char err_buffer[256];
    char* err_msg = strerror_r(errno, err_buffer, 256);
    asprintf(&msg_ptr, "main() Error(%i) %s", errno, err_msg);
    logger.Log(Fatal, msg_ptr);
    free(msg_ptr);
    return 1;
  }

  // int BITS_ADDRESS = 0;
  int BITS_NB = 0;
  // int INPUT_BITS_ADDRESS = 0;
  int INPUT_BITS_NB = 16;
  // int REGISTERS_ADDRESS = 0;
  int REGISTERS_NB = 0;
  // int INPUT_REGISTERS_ADDRESS = 0;
  int INPUT_REGISTERS_NB = 64;

  // initialize the modbus mapping
  mb_mapping = modbus_mapping_new(
    BITS_NB,              // MODBUS_MAX_READ_BITS,
    INPUT_BITS_NB,        // MODBUS_MAX_READ_BITS,
    REGISTERS_NB,         // MODBUS_MAX_READ_REGISTERS,
    INPUT_REGISTERS_NB);  // MODBUS_MAX_READ_REGISTERS)

  if (mb_mapping == NULL) {
    char err_buffer[256];
    char* err_msg = strerror_r(errno, err_buffer, 256);
    asprintf(&msg_ptr, "main() Error(%i) %s", errno, err_msg);
    logger.Log(Fatal, msg_ptr);
    free(msg_ptr);
    modbus_free(ctx);
    return 1;
  }

  // initialize modbus input bits (Registers => 1000X)
  logger.Log(Info, "main() Initializing alarm registers");
  mb_mapping->tab_input_bits[0] = 0;  // cycle_counter_alarm
  mb_mapping->tab_input_bits[1] = 0;  // hw_alarm
  mb_mapping->tab_input_bits[2] = 0;  // motor_off
  mb_mapping->tab_input_bits[3] = 0;  // lightsource_off
  mb_mapping->tab_input_bits[4] = 0;  // detector_off
  mb_mapping->tab_input_bits[5] = 0;  // temp_high_alarm
  mb_mapping->tab_input_bits[6] = 0;  // temp_low_alarm
  mb_mapping->tab_input_bits[7] = 0;  // hum_high_alarm
  mb_mapping->tab_input_bits[8] = 1;  // H2O_conc_pred_valid
  mb_mapping->tab_input_bits[9] = 0;  // saturated alarm
  mb_mapping->tab_input_bits[10] = 0; // low-signal alarm
  mb_mapping->tab_input_bits[11] = 0; // low-pressure alarm

  // start the motor
  int motor_started = startMotor();

  // wait for motor start up
  logger.Log(Info, "main() Waiting for motor startup");
  sleep(30);
  if (motor_started == 0) {
    logger.Log(Info, "main() Motor startup success");
  }

  if (motor_started != 0) {
    logger.Log(Fatal, "main() Motor startup failed");
    return 1;
  }

  {
    // begin polling the serial stream
    std::thread t1(serialPollingTask, adc_cfg);
    t1.detach();  // detach from thread
    // reference:
    /* https://stackoverflow.com/questions/7381757/c-terminate-called-without-an-active-exception */
  }  // thread handle is destroyed here and goes out of scope

  // give time for t1 to startup
  sleep(2);

  {
    // begin polling the hardware alarm relay signal
    std::thread t2(checkLowPressureAlarm);
    t2.detach();  // detach from thread
  }

  // give time for t2 to startup
  sleep(2);

  // create the server socket
  server_socket = modbus_tcp_listen(ctx, NB_CONNECTION);
  if (server_socket == -1) {
    char err_buffer[256];
    char* err_msg = strerror_r(errno, err_buffer, 256);
    asprintf(&msg_ptr, "main() Error(%i) %s", errno, err_msg);
    logger.Log(Fatal, msg_ptr);
    free(msg_ptr);
    modbus_free(ctx);
    return 1;
  }

  // handle interrupt
  logger.Log(Info, "main() Initializing interrupt handler");
  signal(SIGINT, close_sigint);

  // clear the reference set of sockets
  logger.Log(Info, "main() Clearing reference sockets");
  FD_ZERO(&refset);

  // add the new server socket
  logger.Log(Info, "main() Creating new server socket");
  FD_SET(server_socket, &refset);

  // keep track of max file descriptors
  fdmax = server_socket;

  // initialize buffer for modbus queries
  uint8_t *query = new uint8_t[MODBUS_TCP_MAX_ADU_LENGTH*sizeof(uint8_t)];

  logger.Log(Info, "main() Initializing modbus host");
  while (1) {
    // update socket set
    rdset = refset;
    if (select(fdmax + 1, &rdset, NULL, NULL, NULL) == -1) {
        char err_buffer[256];
        char* err_msg = strerror_r(errno, err_buffer, 256);
        asprintf(&msg_ptr, "main() Error(%i) %s", errno, err_msg);
        logger.Log(Fatal, msg_ptr);
        free(msg_ptr);
        startMotor(true);
        close_sigint(1);
        sleep(2);
        return 1;
    }

    // loop through existing connections looking for data to read
    for (master_socket = 0; master_socket <= fdmax; master_socket++) {
      if (!FD_ISSET(master_socket, &rdset)) {
        continue;
      }

      // if master == server, client is asking for new connection
      if (master_socket == server_socket) {
        socklen_t addrlen;
        struct sockaddr_in clientaddr;
        int newfd;

        // handle the new incoming connection
        addrlen = sizeof(clientaddr);
        memset(&clientaddr, 0, sizeof(clientaddr));
        newfd = accept(server_socket,
                      (struct sockaddr *) &clientaddr,
                      &addrlen);

        // handle errors
        if (newfd == -1) {
          char err_buffer[256];
          char* err_msg = strerror_r(errno, err_buffer, 256);
          asprintf(&msg_ptr, "main() Error(%i) %s", errno, err_msg);
          logger.Log(Fatal, msg_ptr);
          free(msg_ptr);
          return 1;
        } else {
          // add reference to new fd
          FD_SET(newfd, &refset);

          if (newfd > fdmax) {
            // keep track of max fd
            fdmax = newfd;
          }

          asprintf(
            &msg_ptr,
            "main() New connection from %s:%d on socket %d",
            inet_ntoa(clientaddr.sin_addr),
            clientaddr.sin_port,
            newfd);
          logger.Log(Info, msg_ptr);
          free(msg_ptr);
        }
      } else {
        // read existing connection
        modbus_set_socket(ctx, master_socket);
        rc = modbus_receive(ctx, query);

        asprintf(
          &mb_msg_ptr,
          "host received %i bytes on socket %d", rc, master_socket);
        modbusLogger.Log(Info, mb_msg_ptr);
        free(mb_msg_ptr);

        std::string receive_bytes = "";
        for (int i = 0; i < rc; i++) {
          char* byte_ptr;
          asprintf(&byte_ptr, " [%02X]", query[i]);
          receive_bytes += (std::string)byte_ptr;
          free(byte_ptr);
        }
        modbusLogger.Log(Info, &receive_bytes[0]);

        if (rc > 0) {
          int rp = modbus_reply(ctx, query, rc, mb_mapping);

          std::string reply_bytes = "";
          for (int i = 0; i < rp; i++) {
            char* byte_ptr;
            asprintf(&byte_ptr, " [%02X]", query[i]);
            reply_bytes += (std::string)byte_ptr;
            free(byte_ptr);
          }

          asprintf(
            &mb_msg_ptr,
            "host sent %i bytes on socket %d", rp, master_socket);
          modbusLogger.Log(Info, mb_msg_ptr);
          free(mb_msg_ptr);
          modbusLogger.Log(Info, &reply_bytes[0]);

        } else if (rc == -1) {
          // close the socket
          close(master_socket);

          // remove from reference set
          FD_CLR(master_socket, &refset);

          // end connection or err
          asprintf(
            &msg_ptr,
            "main() Connection closed on socket %d",
            master_socket);
          logger.Log(Info, msg_ptr);
          free(msg_ptr);

          // track fdmax
          if (master_socket == fdmax) {
            fdmax--;
          }
        }
      }
    }

    if (_exceptionPtr) {
      try {
        modbus_free(ctx);
        startMotor(true);  // kill the motor
        close_sigint(1);
        sleep(2);
        std::rethrow_exception(_exceptionPtr);
      } catch(const std::exception &ex) {
        std::cout << "[Fatal] exited with exception: " << ex.what() << std::endl;
        return 1;
      }
    }
  }

  return 0;
}

static void close_sigint(int x) {
    if (server_socket != -1) {
        close(server_socket);
    }
}

int startMotor(bool _killswitch) {
  int fd;  // file descriptor

  logger.Log(Info, "startMotor() Opening handle /dev/i2c-0");
  if ((fd = open("/dev/i2c-0", O_RDWR)) < 0) {
    logger.Log(Warn, "startMotor() hardware alarm enabled");
    mb_mapping->tab_input_bits[1] = 1;

    logger.Log(Warn, "startMotor() motor alarm enabled");
    mb_mapping->tab_input_bits[2] = 1;

    char err_buffer[256];
    char* err_msg = strerror_r(errno, err_buffer, 256);
    asprintf(&msg_ptr, "startMotor() Error(%i) %s", errno, err_msg);
    logger.Log(Fatal, msg_ptr);
    free(msg_ptr);
    return 1;
  }

  /* To use this properly, zero pad the address on the left and store it as 0b:00101001.
  The calls to read and write after the ioctl will automatically set the proper read
  and write bit when signaling the peripheral. ADDR == 0x22 */
  logger.Log(Info, "startMotor() Setting I2C device address 0x22");
  if (ioctl(fd, I2C_SLAVE, 0x22) < 0) {
    logger.Log(Warn, "startMotor() hardware alarm enabled");
    mb_mapping->tab_input_bits[1] = 1;

    logger.Log(Warn, "startMotor() motor alarm enabled");
    mb_mapping->tab_input_bits[2] = 1;

    char err_buffer[256];
    char* err_msg = strerror_r(errno, err_buffer, 256);
    asprintf(&msg_ptr, "startMotor() Error(%i) %s", errno, err_msg);
    logger.Log(Fatal, msg_ptr);
    free(msg_ptr);
    return 1;
  }

  // write 32-bit value to register
  unsigned char comm = 0x10;
  // set to 20% full speed
  unsigned char buf[] = { 0x00, 0xEC, 0x00, 0x00, 0x1F, 0x90 };
  if (_killswitch) {
    // set to 0% speed
    buf[5] = 0x80;
    logger.Log(Info, "startMotor() KILLSWITCH TRIGGERED writing 0x00EC00001F80 to register 0x10");
  } else {
    logger.Log(Info, "startMotor() Writing 0x00EC00001F90 to register 0x10");
  }

  if (i2c_smbus_write_i2c_block_data(fd, comm, sizeof(buf), buf) < 0) {
    logger.Log(Warn, "startMotor() hardware alarm enabled");
    mb_mapping->tab_input_bits[1] = 1;

    logger.Log(Warn, "startMotor() motor alarm enabled");
    mb_mapping->tab_input_bits[2] = 1;

    char err_buffer[256];
    char* err_msg = strerror_r(errno, err_buffer, 256);
    asprintf(&msg_ptr, "startMotor() Error(%i) %s", errno, err_msg);
    logger.Log(Fatal, msg_ptr);
    free(msg_ptr);
    return 1;
  }

  close(fd);
  logger.Log(Info, "startMotor() Closed device at address 0x22");

  return 0;
}

void serialPollingTask(adc_settings *adc_config) {
  // open port
  logger.Log(Info, "serialPollingTask() Opening device handle /dev/ttyACM0");
  int serial_port = open("/dev/ttyACM0", O_RDWR);

  // new termios struct
  struct termios tty;

  // read existing settings, handle errors
  try {
    if (tcgetattr(serial_port, &tty) != 0) {
      char err_buffer[256];
      char* err_msg = strerror_r(errno, err_buffer, 256);
      asprintf(&msg_ptr, "serialPollingTask() Error(%i) %s", errno, err_msg);
      logger.Log(Fatal, msg_ptr);
      throw std::runtime_error(msg_ptr);
    }
  } catch (const std::runtime_error &ex) {
    _exceptionPtr = std::current_exception();
  }

  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag |= CREAD | CLOCAL;

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO;
  tty.c_lflag &= ~ECHOE;
  tty.c_lflag &= ~ECHONL;
  tty.c_lflag &= ~ISIG;

  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

  tty.c_oflag &= ~OPOST;
  tty.c_oflag &= ~ONLCR;

  tty.c_cc[VTIME] = 10;
  tty.c_cc[VMIN] = 0;

  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);

  try {
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
      // throw the hardware alarm enabled
      logger.Log(Warn, "serialPollingTask() hardware alarm enabled");
      mb_mapping->tab_input_bits[1] = 1;

      char err_buffer[256];
      char* err_msg = strerror_r(errno, err_buffer, 256);
      asprintf(&msg_ptr, "serialPollingTask() Error(%i) %s", errno, err_msg);
      logger.Log(Fatal, msg_ptr);
      close(serial_port);
      throw std::runtime_error(msg_ptr);
    }
  } catch (const std::runtime_error &ex) {
    _exceptionPtr = std::current_exception();
  }

  // write ADC gain to RP2040
  asprintf(&msg_ptr, "serialPollingTask() sending gain=%i to controller", adc_config->gain);
  logger.Log(Info, msg_ptr);
  free(msg_ptr);

  // configure command to send gain to controller
  uint8_t set_gain[2] = { 0x04, adc_config->hex_cmd };
  uint8_t response[18];
  int x = write(serial_port, &set_gain, sizeof(set_gain));

  // log write errors
  if (x != 2) {
    asprintf(&msg_ptr, "serialPollingTask() write gain expected %i bytes, sent %i", 2, x);
    logger.Log(Error, msg_ptr);
    free(msg_ptr);

    // set the hardware alarm bit
    logger.Log(Warn, "serialPollingTask() hardware alarm enabled");
    mb_mapping->tab_input_bits[1] = 1;

    // -1 means system error
    if (x == -1) {
      try {
        char err_buffer[256];
        char* err_msg = strerror_r(errno, err_buffer, 256);
        asprintf(&msg_ptr, "serialPollingTask() Error(%i) %s", errno, err_msg);
        logger.Log(Fatal, msg_ptr);
        close(serial_port);
        throw std::runtime_error(msg_ptr);
      } catch (const std::runtime_error &ex) {
        _exceptionPtr = std::current_exception();
      }
    }
  }

  // read the response from the controller
  x = read(serial_port, &response, sizeof(response));
  if (x < 18) {
    asprintf(&msg_ptr, "serialPollingTask() controller response expected %i bytes, received %i", 18, x);
    logger.Log(Error, msg_ptr);
    free(msg_ptr);

    // set the hardware alarm bit
    logger.Log(Warn, "serialPollingTask() hardware alarm enabled");
    mb_mapping->tab_input_bits[1] = 1;

    // -1 means system error
    if (x == -1) {
      try {
        char err_buffer[256];
        char* err_msg = strerror_r(errno, err_buffer, 256);
        asprintf(&msg_ptr, "serialPollingTask() Error(%i) %s", errno, err_msg);
        logger.Log(Fatal, msg_ptr);
        close(serial_port);
        throw std::runtime_error(msg_ptr);
      } catch (const std::runtime_error &ex) {
        _exceptionPtr = std::current_exception();
      }
    }
  }

  if (x == 18) {
    std::string response_msg = "serialPollingTask() controller response";
    for (int i=0; i < 18; i++) {
      char* byte_ptr;
      asprintf(&byte_ptr, " [%02X]", response[i]);
      response_msg += (std::string)byte_ptr;
      free(byte_ptr);
    }
    logger.Log(Info, &response_msg[0]);
  }


  uint8_t *bme_temp = reinterpret_cast<uint8_t *>(malloc(8 * sizeof(uint8_t)));
  uint8_t *bme_hum = reinterpret_cast<uint8_t *>(malloc(10 * sizeof(uint8_t)));
  uint8_t *bme_press = reinterpret_cast<uint8_t *>(malloc(19 * sizeof(uint8_t)));

  uint16_t cycle_counter = 0;
  high_resolution_clock::time_point current_time = high_resolution_clock::now();
  high_resolution_clock::time_point last_updated;
  duration<double> timeout_period;
  mb_mapping->tab_input_bits[0] = 0;  // cycle_counter_alarm

  int MAX_DATA_BYTES = 30720;
  int MAX_DATA_POINTS = 1920;
  int MAX_READ_SIZE = 128;
  int MAX_SERIAL_ATTEMPTS = 10;
  int ser_fails = 0;

  // main polling loop
  while (true) {
    // reset measurement valid
    mb_mapping->tab_input_bits[8] = 1;

    int r = 0;
    int total_bytes = 0;

    last_updated = high_resolution_clock::now();

    uint8_t write_buf[1] = { 0x01 };

    // write a 1 to the serial buffer to signal ADS data
    r = write(serial_port, &write_buf, sizeof(write_buf));
    if (r < 1) {
      asprintf(&msg_ptr, "serialPollingTask() write 0x01, expected %i bytes, sent %i", 1, r);
      logger.Log(Warn, msg_ptr);
      free(msg_ptr);
      ser_fails++;

      // -1 means a system error occured
      if (r == -1) {
        try {
          char err_buffer[256];
          char* err_msg = strerror_r(errno, err_buffer, 256);
          asprintf(&msg_ptr, "serialPollingTask() Error(%i) %s", errno, err_msg);
          logger.Log(Fatal, msg_ptr);
          close(serial_port);
          throw std::runtime_error(msg_ptr);
        } catch (const std::runtime_error &ex) {
          _exceptionPtr = std::current_exception();
          break;
        }
      }
    }

    uint8_t read_buf[MAX_READ_SIZE];     // serial read buffer
    uint8_t data_frame[MAX_DATA_BYTES];  // raw bytes from serial buffer

    // read until data buffer is full
    while (total_bytes < MAX_DATA_BYTES) {
      r = read(serial_port, &read_buf, sizeof(read_buf));

      // catch read errors
      if (r < MAX_READ_SIZE) {
        asprintf(&msg_ptr, "serialPollingTask() data stream expected %i bytes, received %i", MAX_READ_SIZE, r);
        logger.Log(Warn, msg_ptr);
        free(msg_ptr);
        ser_fails++;
        asprintf(&msg_ptr, "serialPollingTask() Failed serial comm attempt %i of %i", ser_fails, MAX_SERIAL_ATTEMPTS);

        // -1 means there was a system error
        if (r == -1) {
          try {
            char err_buffer[256];
            char* err_msg = strerror_r(errno, err_buffer, 256);
            asprintf(&msg_ptr, "serialPollingTask() Error(%i) %s", errno, err_msg);
            logger.Log(Fatal, msg_ptr);
            close(serial_port);
            throw std::runtime_error(msg_ptr);
          } catch (const std::runtime_error &ex) {
            _exceptionPtr = std::current_exception();
            break;
          }
        }
      }

      // copy data bytes if buffer filled
      if (r == MAX_READ_SIZE) {
        memcpy(&data_frame[total_bytes], read_buf, r);
        total_bytes += r;
      }

      // break if max serial failures reached
      if (ser_fails >= MAX_SERIAL_ATTEMPTS) {
        logger.Log(Warn, "serialPollingTask() Max failed attempts to read ADC data reached");
        break;
      }

    }

    // write a 2 to the serial buffer to signal for temperature data
    write_buf[0] = 0x02;
    r = write(serial_port, &write_buf, sizeof(write_buf));
    if (r < 1) {
      asprintf(&msg_ptr, "serialPollingTask() write 0x02, expected %i bytes, sent %i", 1, r);
      logger.Log(Warn, msg_ptr);
      free(msg_ptr);
      ser_fails++;

      // -1 means a system error occured
      if (r == -1) {
        try {
          char err_buffer[256];
          char* err_msg = strerror_r(errno, err_buffer, 256);
          asprintf(&msg_ptr, "serialPollingTask() Error(%i) %s", errno, err_msg);
          logger.Log(Fatal, msg_ptr);
          close(serial_port);
          throw std::runtime_error(msg_ptr);
        } catch (const std::runtime_error &ex) {
          _exceptionPtr = std::current_exception();
          break;
        }
      }
    }

    //  attempt to read temperature data from serial
    r = read(serial_port, &read_buf, sizeof(read_buf));
    if (r < 8) {
      asprintf(&msg_ptr, "serialPollingTask() temperature expected %i bytes, received %i", 8, r);
      logger.Log(Warn, msg_ptr);
      free(msg_ptr);
      ser_fails++;

      // -1 means a system error occured
      if (r == -1) {
        try {
          char err_buffer[256];
          char* err_msg = strerror_r(errno, err_buffer, 256);
          asprintf(&msg_ptr, "serialPollingTask() Error(%i) %s", errno, err_msg);
          logger.Log(Fatal, msg_ptr);
          close(serial_port);
          throw std::runtime_error(msg_ptr);
        } catch (const std::runtime_error &ex) {
          _exceptionPtr = std::current_exception();
          break;
        }
      }
    }
    if (r == 8) {
      // 8 bytes comes from controller on temp read
      memcpy(&bme_temp[0], read_buf, r);
    }

    // write a 3 to serial buffer to request humidity data
    write_buf[0] = 0x03;
    r = write(serial_port, &write_buf, sizeof(write_buf));
    if (r < 1) {
      asprintf(&msg_ptr, "serialPollingTask() write 0x03, expected %i bytes, sent %i", 1, r);
      logger.Log(Warn, msg_ptr);
      free(msg_ptr);
      ser_fails++;

      // -1 means a system error occured
      if (r == -1) {
        try {
          char err_buffer[256];
          char* err_msg = strerror_r(errno, err_buffer, 256);
          asprintf(&msg_ptr, "serialPollingTask() Error(%i) %s", errno, err_msg);
          logger.Log(Fatal, msg_ptr);
          close(serial_port);
          throw std::runtime_error(msg_ptr);
        } catch (const std::runtime_error &ex) {
          _exceptionPtr = std::current_exception();
          break;
        }
      }
    }

    // read humidity data from serial buffer
    r = read(serial_port, &read_buf, sizeof(read_buf));
    if (r < 10) {
      asprintf(&msg_ptr, "serialPollingTask() humidity expected %i bytes, received %i", 10, r);
      logger.Log(Warn, msg_ptr);
      free(msg_ptr);
      ser_fails++;

      // -1 means a system error occured
      if (r == -1) {
        try {
          char err_buffer[256];
          char* err_msg = strerror_r(errno, err_buffer, 256);
          asprintf(&msg_ptr, "serialPollingTask() Error(%i) %s", errno, err_msg);
          logger.Log(Fatal, msg_ptr);
          close(serial_port);
          throw std::runtime_error(msg_ptr);
        } catch (const std::runtime_error &ex) {
          _exceptionPtr = std::current_exception();
          break;
        }
      }
    }

    if (r == 10) {
      // 10 bytes come from BME humidity sensor
      memcpy(&bme_hum[0], read_buf, r);
    }


    // write a 5 to serial buffer to request pressure data
    write_buf[0] = 0x05;
    r = write(serial_port, &write_buf, sizeof(write_buf));
    if (r < 1) {
      asprintf(&msg_ptr, "serialPollingTask() write 0x05, expected %i bytes, sent %i", 1, r);
      logger.Log(Warn, msg_ptr);
      free(msg_ptr);
      ser_fails++;

      // -1 means a system error occured
      if (r == -1) {
        try {
          char err_buffer[256];
          char* err_msg = strerror_r(errno, err_buffer, 256);
          asprintf(&msg_ptr, "serialPollingTask() Error(%i) %s", errno, err_msg);
          logger.Log(Fatal, msg_ptr);
          close(serial_port);
          throw std::runtime_error(msg_ptr);
        } catch (const std::runtime_error &ex) {
          _exceptionPtr = std::current_exception();
          break;
        }
      }
    }

    // read pressurer data from serial buffer
    r = read(serial_port, &read_buf, sizeof(read_buf));
    if (r < 10) {
      asprintf(&msg_ptr, "serialPollingTask() pressure expected %i bytes, received %i", 19, r);
      logger.Log(Warn, msg_ptr);
      free(msg_ptr);
      ser_fails++;

      // -1 means a system error occured
      if (r == -1) {
        try {
          char err_buffer[256];
          char* err_msg = strerror_r(errno, err_buffer, 256);
          asprintf(&msg_ptr, "serialPollingTask() Error(%i) %s", errno, err_msg);
          logger.Log(Fatal, msg_ptr);
          close(serial_port);
          throw std::runtime_error(msg_ptr);
        } catch (const std::runtime_error &ex) {
          _exceptionPtr = std::current_exception();
          break;
        }
      }
    }

    if (r == 19) {
      // 19 bytes come from BME humidity sensor
      memcpy(&bme_press[0], read_buf, r);
    }

    // exit if max serial read fails reached
    if (ser_fails >= MAX_SERIAL_ATTEMPTS) {
        try {
          logger.Log(Fatal, "serialPollingTask() Max serial communication failures reached");
          throw std::runtime_error("serialPollingTask() Max serial communication failures reached");
        } catch (const std::runtime_error &ex) {
          _exceptionPtr = std::current_exception();
          break;
        }
    }

    // skip rest of loop if buffer is
    // not completely full
    if (total_bytes < MAX_DATA_BYTES) {
      logger.Log(Warn, "serialPollingTask() Data buffer not filled, continuing data acquisition");
      ser_fails++;
      asprintf(&msg_ptr, "serialPollingTask() Failed serial comm attempt %i of %i", ser_fails, MAX_SERIAL_ATTEMPTS);
      logger.Log(Fatal, msg_ptr);
      free(msg_ptr);
      continue;
    }

    int n = 0;
    int idx = 0;

    // store the peak values for each position
    double peaks3[32];
    double peaks5[32];
    double peaks6[32];

    // keep track of how many peaks occur in each position
    int n3 = 0;
    int n5 = 0;
    int n6 = 0;

    bool start = false;
    bool measure = false;
    bool above_zero = false;
    double pmax = -10;
    double pmin = 10;
    int peak_pos;
    double data_arr[MAX_DATA_POINTS];    // ADC floating point data
    uint8_t pos_arr[MAX_DATA_POINTS];

    // convert raw bytes to floating point data
    while (n < 7680) {  // 3841
      // read 3 bytes of ADC signal (MSB first)
      int32_t upperByte = (int32_t) data_frame[n] << 24;
      n++;
      int32_t middleByte = (int32_t) data_frame[n] << 16;
      n++;
      int32_t lowerByte = (int32_t) data_frame[n] << 8;
      n++;

      // read the position byte
      pos_arr[idx] = data_frame[n];
      n++;

      // right shift ch0 to get sign bit
      int32_t ch0 = (upperByte | middleByte | lowerByte) >> 8;

      // scale adc value then divide by HW gain
      double ch0_f = (adc_config->scale_factor * ch0) / HARDWARE_GAIN;

      // back-calculate the detector resistance
      ch0_f = (-ch0_f * (698000.0)) / (ch0_f - 2.5);

      // invert value to put filter values on top
      // half of signal
      data_arr[idx] = -ch0_f;

      idx++;
      if (idx >= MAX_DATA_POINTS) break;
    }

    // mean-center the data (find mean value as zero-point)
    double mean_center = 0.0;
    for (int i = 0; i < MAX_DATA_POINTS; i++) {
      mean_center += data_arr[i];
    }
    mean_center = mean_center/MAX_DATA_POINTS;

    // calculate the peak-to-peak intensities
    for (int i = 0; i < MAX_DATA_POINTS; i++) {
      // don't do anything until the signal drops below 0
      if (data_arr[i] < mean_center && !start) {
        start = true;
      }

      // don't start measuring until the first positive zero-crossing
      if (data_arr[i] > mean_center && data_arr[i - 1] <= mean_center && start && !measure) {
        measure = true;
      }

      // wait until at least 1 value in data array
      // and starting flag active
      if (i > 0 && measure) {
        // look for zero-crossing in positive direction
        if (data_arr[i] > mean_center && data_arr[i - 1] <= mean_center) {
          if (pmin != 10 && n3 < 32 && n5 < 32 && n6 < 32) {
            switch (peak_pos) {
              case 0x03:
                peaks3[n3] = pmax - pmin;
                n3++;
                break;
              case 0x05:
                peaks5[n5] = pmax - pmin;
                n5++;
                break;
              case 0x06:
                peaks6[n6] = pmax - pmin;
                n6++;
                break;
              default:
                break;
            }
          }
          above_zero = true;
          pmax = -10;
        }

        // look for zero-crossing in negative direction
        if (data_arr[i] < mean_center && data_arr[i - 1] >= mean_center) {
          above_zero = false;
          pmin = 10;
        }

        if (above_zero) {
          // look for the max
          if (data_arr[i] > pmax) {
            pmax = data_arr[i];
            peak_pos = pos_arr[i];
          }
        } else if (!above_zero) {
          // look for min

          if (data_arr[i] < pmin) {
            pmin = data_arr[i];
          }
        }
      }
    }

    // calculate average of each peak/dark
    double avg_p3 = 0;
    double avg_p5 = 0;
    double avg_p6 = 0;

    for (int i = 0; i < n3; i++) {
      avg_p3 += peaks3[i];
    }
    for (int i = 0; i < n5; i++) {
      avg_p5 += peaks5[i];
    }
    for (int i = 0; i < n6; i++) {
      avg_p6 += peaks6[i];
    }
    avg_p3 = avg_p3 / n3;
    avg_p5 = avg_p5 / n5;
    avg_p6 = avg_p6 / n6;

    // ND
    uint64_t u;
    memcpy(&u, &avg_p5, sizeof(double));
    mb_mapping->tab_input_registers[0] = (uint16_t) (u >> 48) | 0;
    mb_mapping->tab_input_registers[1] = (uint16_t) (u >> 32) | 0;
    mb_mapping->tab_input_registers[2] = (uint16_t) (u >> 16) | 0;
    mb_mapping->tab_input_registers[3] = (uint16_t) u | 0;

    // MOE 1
    memcpy(&u, &avg_p6, sizeof(double));
    mb_mapping->tab_input_registers[4] = (uint16_t) (u >> 48) | 0;
    mb_mapping->tab_input_registers[5] = (uint16_t) (u >> 32) | 0;
    mb_mapping->tab_input_registers[6] = (uint16_t) (u >> 16) | 0;
    mb_mapping->tab_input_registers[7] = (uint16_t) u | 0;

    // MOE 2
    memcpy(&u, &avg_p3, sizeof(double));
    mb_mapping->tab_input_registers[8] = (uint16_t) (u >> 48) | 0;
    mb_mapping->tab_input_registers[9] = (uint16_t) (u >> 32) | 0;
    mb_mapping->tab_input_registers[10] = (uint16_t) (u >> 16) | 0;
    mb_mapping->tab_input_registers[11] = (uint16_t) u | 0;

    // write the temp to modbus
    double temp_c = convert_temp(bme_temp);
    uint64_t t;
    memcpy(&t, &temp_c, sizeof(double));
    mb_mapping->tab_input_registers[16] = (uint16_t) (t >> 48) | 0;
    mb_mapping->tab_input_registers[17] = (uint16_t) (t >> 32) | 0;
    mb_mapping->tab_input_registers[18] = (uint16_t) (t >> 16) | 0;
    mb_mapping->tab_input_registers[19] = (uint16_t) t | 0;

    // Humidity reading
    double hum = convert_humidity(bme_hum, temp_c);
    uint64_t h;
    memcpy(&h, &hum, sizeof(double));
    mb_mapping->tab_input_registers[20] = (uint16_t) (h >> 48) | 0;
    mb_mapping->tab_input_registers[21] = (uint16_t) (h >> 32) | 0;
    mb_mapping->tab_input_registers[22] = (uint16_t) (h >> 16) | 0;
    mb_mapping->tab_input_registers[23] = (uint16_t) h | 0;

    // pressure reading
    double press = convert_pressure(bme_press, temp_c);
    // convert from Pascal to PSI
    press = press * 0.000145038;
    uint64_t pr;
    memcpy(&pr, &press, sizeof(double));
    mb_mapping->tab_input_registers[24] = (uint16_t) (pr >> 48) | 0;
    mb_mapping->tab_input_registers[25] = (uint16_t) (pr >> 32) | 0;
    mb_mapping->tab_input_registers[26] = (uint16_t) (pr >> 16) | 0;
    mb_mapping->tab_input_registers[27] = (uint16_t) pr | 0;


    double H2O_conc = 0.0;
    if (pred_mode == 1 || pred_mode == 2) {
      // single beam
      // moe1/2/ndf slope & y-int at standard 30-C temp
      double moe1_std_2nd, moe1_std_slope, moe1_std_yint;
      moe1_std_2nd = calculate_second_order(moe1_cal, std_cal_temp);
      moe1_std_slope = calculate_slope(moe1_cal, std_cal_temp);
      moe1_std_yint = calculate_intercept(moe1_cal, std_cal_temp);

      double moe2_std_2nd, moe2_std_slope, moe2_std_yint;
      moe2_std_2nd = calculate_second_order(moe2_cal, std_cal_temp);
      moe2_std_slope = calculate_slope(moe2_cal, std_cal_temp);
      moe2_std_yint = calculate_intercept(moe2_cal, std_cal_temp);

      double ndf_std_2nd, ndf_std_slope, ndf_std_yint;
      ndf_std_2nd = calculate_second_order(ndf_cal, std_cal_temp);
      ndf_std_slope = calculate_slope(ndf_cal, std_cal_temp);
      ndf_std_yint = calculate_intercept(ndf_cal, std_cal_temp);

      // calculate temp correction coefficients for each MOE and NDF
      double moe1_2nd, moe1_slope, moe1_yint;
      moe1_2nd = calculate_second_order(moe1_cal, temp_c);
      moe1_slope = calculate_slope(moe1_cal, temp_c);
      moe1_yint = calculate_intercept(moe1_cal, temp_c);

      double moe2_2nd, moe2_slope, moe2_yint;
      moe2_2nd = calculate_second_order(moe2_cal, temp_c);
      moe2_slope = calculate_slope(moe2_cal, temp_c);
      moe2_yint = calculate_intercept(moe2_cal, temp_c);

      double ndf_2nd, ndf_slope, ndf_yint;
      ndf_2nd = calculate_second_order(ndf_cal, temp_c);
      ndf_slope = calculate_slope(ndf_cal, temp_c);
      ndf_yint = calculate_intercept(ndf_cal, temp_c);

      // scale the raw signals channels
      double moe1_scaled, moe2_scaled, ndf_scaled;
      moe1_scaled = avg_p6 / 18408.79121;
      moe2_scaled = avg_p3 / 18408.79121;
      ndf_scaled = avg_p5 / 18408.79121;

      // solve the quadratics for each filter channel
      double moe1_pred_abs, moe2_pred_abs, ndf_pred_abs;
      moe1_pred_abs = quadratic_solver(moe1_2nd, moe1_slope, (moe1_yint - moe1_scaled));
      moe2_pred_abs = quadratic_solver(moe2_2nd, moe2_slope, (moe2_yint - moe2_scaled));
      ndf_pred_abs = quadratic_solver(ndf_2nd, ndf_slope, (ndf_yint - ndf_scaled));

      // calculate corrected ratios @ 30C standard temp
      double moe1_corr = (moe1_std_2nd * pow(moe1_pred_abs, 2)) + (moe1_std_slope * moe1_pred_abs) + moe1_std_yint;
      double moe2_corr = (moe2_std_2nd * pow(moe2_pred_abs, 2)) + (moe2_std_slope * moe2_pred_abs) + moe2_std_yint;
      double ndf_corr = (ndf_std_2nd * pow(ndf_pred_abs, 2)) + (ndf_std_slope * ndf_pred_abs) + ndf_std_yint;

      if (pred_mode == 1) {
        // calculated the predicted concentration
        H2O_conc = prediction_model(h20_pred_model, moe1_corr, moe2_corr, ndf_corr);
      }

      if (pred_mode == 2) {
        if (ndf_corr != 0.0) {
          double r_corr1, r_corr2;
          r_corr1 = moe1_corr / ndf_corr;
          r_corr2 = moe2_corr / ndf_corr;
          H2O_conc = prediction_model(h20_pred_model, r_corr1, r_corr2, 0.0);
        }
      }

    } else {
      // otherwise, use ratio mode
      // moe1/2 slope & y-int at standard temp (33.34C)
      double moe1_std_slope, moe1_std_yint, moe2_std_slope, moe2_std_yint;
      moe1_std_slope = calculate_slope(moe1_cal, std_cal_temp);
      moe1_std_yint = calculate_intercept(moe1_cal, std_cal_temp);

      moe2_std_slope = calculate_slope(moe2_cal, std_cal_temp);
      moe2_std_yint = calculate_intercept(moe2_cal, std_cal_temp);

      // calculate temp correction coefficients for each MOE
      double moe1_slope, moe1_yint, moe2_slope, moe2_yint;

      moe1_slope = calculate_slope(moe1_cal, temp_c);
      moe1_yint = calculate_intercept(moe1_cal, temp_c);

      moe2_slope = calculate_slope(moe2_cal, temp_c);
      moe2_yint = calculate_intercept(moe2_cal, temp_c);

      // calculate MOE/ND ratios
      double R1, R2;
      R1 = avg_p6 / avg_p5;
      R2 = avg_p3 / avg_p5;

      // calculate predicted absorbance for each moe at measured temp
      double moe1_pred_abs, moe2_pred_abs;
      moe1_pred_abs = (R1 - moe1_yint) / moe1_slope;
      moe2_pred_abs = (R2 - moe2_yint) / moe2_slope;

      // calculate corrected ratios @ 33.34C standard temp
      double r_corr1, r_corr2;
      r_corr1 = (moe1_std_slope * moe1_pred_abs) + moe1_std_yint;
      r_corr2 = (moe2_std_slope * moe2_pred_abs) + moe2_std_yint;

      // compute the final predicted concentration
      H2O_conc = prediction_model(h20_pred_model, r_corr1, r_corr2, 0.0);
    }

    // write concentration to modbus
    uint64_t conc;
    memcpy(&conc, &H2O_conc, sizeof(double));
    mb_mapping->tab_input_registers[12] = (uint16_t) (conc >> 48) | 0;
    mb_mapping->tab_input_registers[13] = (uint16_t) (conc >> 32) | 0;
    mb_mapping->tab_input_registers[14] = (uint16_t) (conc >> 16) | 0;
    mb_mapping->tab_input_registers[15] = (uint16_t) conc | 0;

    /* Check if Signal within range */
    // check high signal (MOE2 is the highest signal)
    if (avg_p3 > 0.85 * adc_config->max_ohms) {
      logger.Log(Warn, "serialPollingTask() detector is saturated");
      mb_mapping->tab_input_bits[9] = 1;
    } else {
      if (mb_mapping->tab_input_bits[9] == 1) {
        mb_mapping->tab_input_bits[9] = 0;
        logger.Log(Info, "serialPollingTask() detector signal in range");
      }
    }

    // check low signal
    if (avg_p5 < .2 * adc_config->max_ohms) {
      logger.Log(Warn, "serialPollingTask() low light level");
      mb_mapping->tab_input_bits[10] = 1;
    } else {
      if (mb_mapping->tab_input_bits[10] == 1) {
        mb_mapping->tab_input_bits[10] = 0;
        logger.Log(Info, "serialPollingTask() detector signal in range");
      }
    }

    /* check if Temp within range */
    if (temp_c > 45.0) {
      logger.Log(Warn, "serialPollingTask() temperature out of range - high");
      mb_mapping->tab_input_bits[5] = 1;
    } else {
      if (mb_mapping->tab_input_bits[5] == 1) {
        mb_mapping->tab_input_bits[5] = 0;
        logger.Log(Info, "serialPollingTask() temperature in range");
      }
    }

    /* check if Temp within range */
    if (temp_c < 20.0) {
      logger.Log(Warn, "serialPollingTask() temperature out of range - low");
      mb_mapping->tab_input_bits[6] = 1;
    } else {
      if (mb_mapping->tab_input_bits[6] == 1) {
        mb_mapping->tab_input_bits[6] = 0;
        logger.Log(Info, "serialPollingTask() temperature in range");
      }
    }

    /* Check if humidity within range */
    if (hum > 10.0) {
      logger.Log(Warn, "serialPollingTask() humidity out of range");
      mb_mapping->tab_input_bits[7] = 1;
    } else {
      if (mb_mapping->tab_input_bits[7] == 1) {
        mb_mapping->tab_input_bits[7] = 0;
        logger.Log(Info, "serialPollingTask() humidity in range");
      }
    }

    /* VALIDATE MEASUREMENT BY CHECKING ALARM BITS */
    if (mb_mapping->tab_input_bits[9] == 1 ||
        mb_mapping->tab_input_bits[10] == 1 ||
        mb_mapping->tab_input_bits[11] == 0 ||
        mb_mapping->tab_input_bits[5] == 1 ||
        mb_mapping->tab_input_bits[6] == 1 ||
        mb_mapping->tab_input_bits[7] == 1 ||
        mb_mapping->tab_input_bits[1] == 1) {
      // trigger measurement invalid
      mb_mapping->tab_input_bits[8] = 0;
    }

    // reset the hardware alarm
    if (mb_mapping->tab_input_bits[1] == 1) {
      logger.Log(Warn, "serialPollingTask() hardware alarm reset");
      mb_mapping->tab_input_bits[1] = 0;
    }

    // measure the timeout period
    current_time = high_resolution_clock::now();
    timeout_period = duration_cast<duration<double>>(
      current_time - last_updated);
    double prd = timeout_period.count();
    if (prd < 5.0) {
      // increment the cycle counter if within timeout_period
      mb_mapping->tab_input_registers[28] = cycle_counter;
      cycle_counter = cycle_counter < 32767 ? cycle_counter + 1 : 0;
      if (cycle_counter == 0) {
        logger.Log(Info, "serialPollingTask() cycle counter reset");
      }
      mb_mapping->tab_input_bits[0] = 0;
    } else {
      // trigger cycle_counter_alarm if timeout_period exceeded
      asprintf(&msg_ptr, "serialPollingTask() measurement cycle %i timeout: %f s", cycle_counter, prd);
      logger.Log(Warn, msg_ptr);
      free(msg_ptr);
      mb_mapping->tab_input_bits[0] = 1;
    }


    asprintf(
      &data_msg_ptr,
      "%f,%f,%f,%f,%f,%f,%f,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i",
      avg_p6, avg_p3, avg_p5, H2O_conc, temp_c, hum, press,
      mb_mapping->tab_input_bits[0],  // cycle_counter_alarm
      mb_mapping->tab_input_bits[1],  // hw_alarm
      mb_mapping->tab_input_bits[2],  // motor_off
      mb_mapping->tab_input_bits[3],  // lightsource_off
      mb_mapping->tab_input_bits[4],  // detector_off
      mb_mapping->tab_input_bits[5],  // temp_high_alarm
      mb_mapping->tab_input_bits[6],  // temp_low_alarm
      mb_mapping->tab_input_bits[7],  // hum_high_alarm
      mb_mapping->tab_input_bits[8],  // H2O_conc_pred_valid
      mb_mapping->tab_input_bits[9],  // saturated alarm
      mb_mapping->tab_input_bits[10], // low-signal alarm
      mb_mapping->tab_input_bits[11]  // low-pressure alarm
    );
    dataLogger.Log(Info, data_msg_ptr);
    free(data_msg_ptr);

  }

  close(serial_port);
}

void checkLowPressureAlarm() {
    int f;
    char fname[] = "/dev/i2c-0";
    try {
      if ((f = open(fname, O_RDWR)) < 0) {

        logger.Log(Warn, "checkLowPressureAlarm() hardware alarm enabled");
        mb_mapping->tab_input_bits[1] = 1;

        char err_buffer[256];
        char* err_msg = strerror_r(errno, err_buffer, 256);
        asprintf(&msg_ptr, "checkLowPressureAlarm() Error(%i) %s", errno, err_msg);
        logger.Log(Fatal, msg_ptr);
        free(msg_ptr);
        throw std::runtime_error(msg_ptr);
      }
    } catch (const std::runtime_error &ex) {
      _exceptionPtr = std::current_exception();
    }

    /* To use this properly, zero pad the address on the left and store it as 0b00101001.
    The calls to read and write after the ioctl will automatically set the proper read
    and write bit when signaling the peripheral.*/
    int addr = 0x21;
    try {
      if (ioctl(f, I2C_SLAVE, addr) < 0) {

        logger.Log(Warn, "checkLowPressureAlarm() hardware alarm enabled");
        mb_mapping->tab_input_bits[1] = 1;

        char err_buffer[256];
        char* err_msg = strerror_r(errno, err_buffer, 256);
        asprintf(&msg_ptr, "checkLowPressureAlarm() Error(%i) %s", errno, err_msg);
        logger.Log(Fatal, msg_ptr);
        free(msg_ptr);
        throw std::runtime_error(msg_ptr);
      }
    } catch (const std::runtime_error &ex) {
      _exceptionPtr = std::current_exception();
    }

    // to write to i2c bus, first send 7-bit start condidition with
    // target addres + 0 for write
    // ---> 01000000
    //char buf[10] = {0};
    //char start_cond[10] = {0b01000010};
    //char reg_addr[10] = {0b00000011};  // 0x03 --> configuration (0=output, 1=input)
    //char config[10] = {0b111000};  // set pins 4-7 as outputs
    //buf[0] = 0b00000011;  // register address
    //buf[1] = 0b11110000;  // pin output configuration
    //buf[2] = 0b00000001;  // output port address
    //buf[2] = 0b11110000;  // set pins 4-7 to logic low

    // write pin config to device
    // configure bit [0] as input
    __u8 reg = 0x03;
    __u8 pinconfig = 0b00000001;
    int res;
    try {
      if ((res = i2c_smbus_write_byte_data(f, reg, pinconfig)) == -1) {

        logger.Log(Warn, "checkLowPressureAlarm() hardware alarm enabled");
        mb_mapping->tab_input_bits[1] = 1;

        char err_buffer[256];
        char* err_msg = strerror_r(errno, err_buffer, 256);
        asprintf(&msg_ptr, "checkLowPressureAlarm() Error(%i) %s", errno, err_msg);
        logger.Log(Fatal, msg_ptr);
        free(msg_ptr);
        throw std::runtime_error(msg_ptr);
      }
    } catch (const std::runtime_error &ex) {
      _exceptionPtr = std::current_exception();
    }

    logger.Log(Info, "checkLowPressureAlarm() low pressure relay configured");

    while (1) {

      // set pin levels
      // __u8 pinlogic = 0b00000000;
      // reg = 0x01;
      // if ((res = i2c_smbus_write_byte_data(f, reg, pinlogic)) == -1) {
      //     std::cout << "[ERR " << errno << "] --->  " << std::strerror(errno) << std::endl;
      // }
      // std::cout << "Set Pin Logic --> " << std::bitset<8>(pinlogic) << std::endl;

      // read pins
      int out_word;
      reg = 0x00;
      try {
        if ((out_word = i2c_smbus_read_word_data(f, reg)) == -1) {

          logger.Log(Warn, "checkLowPressureAlarm() hardware alarm enabled");
          mb_mapping->tab_input_bits[1] = 1;

          char err_buffer[256];
          char* err_msg = strerror_r(errno, err_buffer, 256);
          asprintf(&msg_ptr, "checkLowPressureAlarm() Error(%i) %s", errno, err_msg);
          logger.Log(Fatal, msg_ptr);
          free(msg_ptr);
          throw std::runtime_error(msg_ptr);
        }

        // update the low pressure alarm signal
        mb_mapping->tab_input_bits[11] = std::bitset<8>(out_word)[0];

        if (std::bitset<8>(out_word)[0] == 1) {
          logger.Log(Warn, "checkLowPressureAlarm() hardware alarm enabled");
          mb_mapping->tab_input_bits[1] = 1;

          logger.Log(Warn, "checkLowPressureAlarm() low pressure alarm enabled");
        }

      } catch (const std::runtime_error &ex) {
        _exceptionPtr = std::current_exception();
      }

      sleep(2);
    }
    close(f);
}
