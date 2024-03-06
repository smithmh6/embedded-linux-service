/* Copyright Thorlabs 2023
 *
 * Definitions for Optical Engine system
 */

#ifndef DOW_GEN3_EMBEDDED_INC_DEFINITIONS_H
#define DOW_GEN3_EMBEDDED_INC_DEFINITIONS_H

#include <stdint.h>
#include <math.h>

#define NB_CONNECTION 5  // allowed number of modbus connections

#define HARDWARE_GAIN 37.36

struct adc_settings {
  int gain;
  double max_ohms;
  double scale_factor;
  uint8_t hex_cmd;
};

/*  Hardware Gain = 37.36x
 *  Max (V)   SW Gain   Max (Ohms)
 *  2.4	      1 	    18408.79121
 *  1.2	      2 	    9084.598698
 *  0.6	      4 	    4512.931034
 *  0.3	      8 	    2249.194415
 *  0.15	    16  	  1122.788204
 *  0.075	    32  	  560.9429413
 *  0.0375	  64  	  280.3588164
 *  0.01875	  128 	  140.1512616
 */

static adc_settings adc_config_table[] = {
  {1, 18408.79121, 1.430511475e-07, 0x00},
  {2, 9084.598698, 7.152557373e-08, 0x01},
  {4, 4512.931034, 3.576278687e-08, 0x02},
  {8, 2249.194415, 1.788139343e-08, 0x03},
  {16, 1122.788204, 8.940696716e-09, 0x04},
  {32, 560.9429413, 4.470348358e-09, 0x05},
  {64, 280.3588164, 2.235174179e-09, 0x06},
  {128, 140.1512616, 1.117587090e-09, 0x07}
};

const unsigned int adc_entries =
  sizeof(adc_config_table) / sizeof(adc_config_table[0]);

struct calibration_settings {
  double HC2;  // 2nd order coefficient C2
  double HC1;  // 2nd order coefficient C1
  double HC0;  // 2nd order coefficient C0
  double SC2;  // slope coefficient C2
  double SC1;  // slope coefficient C1
  double SC0;  // slope coefficient C0
  double IC2;  // intercept coefficient C2
  double IC1;  // intercept coefficient C1
  double IC0;  // intercept coefficient C0
};

struct prediction_settings {
  double C9;  // a^2
  double C8;  // b^2
  double C7;  // n^2
  double C6;  // a
  double C5;  // b
  double C4;  // n
  double C3;  // a*b
  double C2;  // a*n
  double C1;  // b*n
  double C0;  // offset
};

double convert_temp(uint8_t *data) {
  double var1, var2, t_fine, temp_comp;

  // uint8_t *out = (uint8_t *)malloc(8 * sizeof(uint8_t));
  // out[0] = p1data[1];   // par_t1 MSB
  // out[1] = p1data[0];   // par_t1 LSB
  // out[2] = p23data[1];  // par_t2 MSB
  // out[3] = p23data[0];  // par_t2 LSB
  // out[4] = p23data[2];  // par_t3
  // out[5] = data[0];     // temp MSB
  // out[6] = data[1];     // temp LSB
  // out[7] = data[2];     // temp X-LSB

  uint16_t par_t1, par_t2;
  uint8_t par_t3;

  par_t1 = data[0];
  par_t1 = par_t1 << 8 | data[1];
  par_t2 = data[2];
  par_t2 = par_t2 << 8 | data[3];
  par_t3 = data[4];

  int32_t temp = data[5];
  temp = temp << 8 | data[6];
  temp = temp << 8 | data[7];
  temp = temp >> 4;

  var1 = ((static_cast<double>(temp) / 16384.0) -
          (static_cast<double>(par_t1) / 1024.0)) *
          static_cast<double>(par_t2);

  var2 = (((static_cast<double>(temp) / 131072.0) -
          (static_cast<double>(par_t1) / 8192.0)) *
          ((static_cast<double>(temp) / 131072.0) -
          (static_cast<double>(par_t1) / 8162.0))) *
          (static_cast<double>(par_t3) * 16.0);

  t_fine = var1 + var2;
  temp_comp = t_fine / 5120.0;

  return temp_comp;
}

double convert_humidity(uint8_t *data, double temp_comp) {
  // uint8_t *out = (uint8_t *)malloc(10 * sizeof(uint8_t));
  // out[0] = parh1h2_data[2];  // par_h1 MSB
  // out[1] = parh1h2_data[0];  // par_h2 MSB
  // out[2] = parh1h2_data[1];  // <7:4> par_h2 LSB, <3:0> par_h1 LSB
  // out[3] = parh3h7_data[0];  // par_h3
  // out[4] = parh3h7_data[1];  // par_h4
  // out[5] = parh3h7_data[2];  // par_h5
  // out[6] = parh3h7_data[3];  // par_h6
  // out[7] = parh3h7_data[4];  // par_h7
  // out[8] = data[0];          // hum MSB
  // out[9] = data[1];          // hum LSB

  int16_t hum_adc;
  uint16_t par_h1, par_h2;
  int8_t par_h3, par_h4, par_h5, par_h7;
  uint8_t par_h6;

  // (par_h1) left shift data[0] and bitwise or with
  // extracted lower 4-bits of data[2] (bitwise AND with 0x0F)
  par_h1 = (uint16_t)(((uint16_t)data[0] << 4) | (data[2] & 0x0F));

  // par_h2 left shift data[1] and bitwise OR with
  // upper 4 bits of data[2] (right shift 4 bits)
  par_h2 = (uint16_t)(((uint16_t)data[1] << 4) | (data[2] >> 4));

  // par_h3 - par_h7
  par_h3 = data[3];
  par_h4 = data[4];
  par_h5 = data[5];
  par_h7 = data[6];
  par_h6 = data[7];

  // humidity bytes
  hum_adc = data[8];
  hum_adc = (hum_adc << 8) | data[9];

  // calculate the compensated humidity
  double var1, var2, var3, var4,  hum_comp;

  var1 = (static_cast<double>(hum_adc) -
          ((static_cast<double>(par_h1) * 16.0) +
            ((static_cast<double>(par_h3) / 2.0) * temp_comp)));
  var2 = var1 * ((static_cast<double>(par_h2) / 262144.0)
                  * (1.0 + ((static_cast<double>(par_h4) / 16384.0)
                  * temp_comp) + ((static_cast<double>(par_h5) / 1048576.0)
                  * temp_comp * temp_comp)));
  var3 = static_cast<double>(par_h6) / 16384.0;
  var4 = static_cast<double>(par_h7) / 2097152.0;

  hum_comp = var2 + ((var3 + (var4 * temp_comp)) * var2 * var2);

  if (hum_comp < 0.0) return 0.0;

  if (hum_comp > 100.0) return 100.0;

  return hum_comp;
}

double convert_pressure(uint8_t *data, double temp_comp) {
  // computes pressure in Pascal
  double t_fine = temp_comp * 5120.0;

  double var1, var2, var3, press_comp;
  uint16_t p1;
  int16_t p2, p4, p5, p8, p9;
  int8_t p3, p6, p7, p10;

  int32_t press_adc = data[18];
  press_adc = press_adc << 8 | data[17];
  press_adc = press_adc << 8 | data[16];
  press_adc = press_adc >> 4;

  p1 = data[1];
  p1 = (p1 << 8) | data[0];
  p2 = data[3];
  p2 = (p2 << 8) | data[2];
  p3 = data[4];
  p4 = data[6];
  p4 = (p4 << 8) | data[5];
  p5 = data[8];
  p5 = (p5 << 8) | data[7];
  p6 = data[10];
  p7 = data[9];
  p8 = data[12];
  p8 = (p8 << 8) | data[11];
  p9 = data[14];
  p9 = (p9 << 8) | data[13];
  p10 = data[15];

  var1 = (t_fine / 2.0) - 64000.0;
  var2 = var1 * var1 * ((static_cast<double>(p6)) / (131072.0));
  var2 = var2 + (var1 * (static_cast<double>(p5)) * 2.0);
  var2 = (var2 / 4.0) + ((static_cast<double>(p4)) * 65536.0);
  var1 = ((((static_cast<double>(p3) * var1 * var1) / 16384.0) + (static_cast<double>(p2) * var1)) / 524288.0);
  var1 = ((1.0 + (var1 / 32768.0)) * (static_cast<double>(p1)));
  press_comp = (1048576.0 - (static_cast<double>(press_adc)));

  /* Avoid exception caused by division by zero */
  if ((int)var1 != 0)
  {
      press_comp = (((press_comp - (var2 / 4096.0)) * 6250.0) / var1);
      var1 = ((static_cast<double>(p9)) * press_comp * press_comp) / 2147483648.0;
      var2 = press_comp * ((static_cast<double>(p8)) / 32768.0);
      var3 = ((press_comp / 256.0) * (press_comp / 256.0) * (press_comp / 256.0) * (p10 / 131072.0));
      press_comp = (press_comp + (var1 + var2 + var3 + (static_cast<double>(p7) * 128.0)) / 16.0);
  }
  else
  {
      press_comp = 0;
  }
  return press_comp;
}

double quadratic_poly(double C2, double C1, double C0, double x) {
  return (C2 * pow(x, 2)) + (C1 * x) + C0;
}

double quadratic_solver(double a, double b, double c) {
    // b^2 - 4*a*c
    double var1 = pow(b, 2) - (4 * a * c);
    var1 = var1 < 0.0 ? 0.0 : var1;  // set to 0 if less than 0

    // [-b +/- sqrt(var1)] / 2 *a
    if (a == 0.0) return 0.0;
    double plus_term = (-b + sqrt(var1)) / (2 * a);
    double minus_term = (-b - sqrt(var1)) / (2 * a);

    // return the number with the smallest absolute value
    return abs(plus_term) <= abs(minus_term) ? plus_term : minus_term;
}

double calculate_second_order(calibration_settings *cal, double temp) {
  return quadratic_poly(cal->HC2, cal->HC1, cal->HC0, temp);
}

double calculate_slope(calibration_settings *cal, double temp) {
  return quadratic_poly(cal->SC2, cal->SC1, cal->SC0, temp);
}

double calculate_intercept(calibration_settings *cal, double temp) {
  return quadratic_poly(cal->IC2, cal->IC1, cal->IC0, temp);
}

double prediction_model(prediction_settings *pred, double a, double b, double n) {
  double var1, var2, var3, offset;
  var1 = (pred->C9 * pow(a, 2)) + (pred->C8 * pow(b, 2)) + (pred->C7 * pow(n, 2));
  var2 = (pred->C6 * a) + (pred->C5 * b) + (pred->C4 * n);
  var3 = (pred->C3 * a * b) + (pred->C2 * a * n) + (pred->C1 * b * n);
  offset = pred->C0;
  return var1 + var2 + var3 + offset;
}

#endif  // DOW_GEN3_EMBEDDED_INC_DEFINITIONS_H