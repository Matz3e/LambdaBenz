/*
  Lambda Tool für den Controlduino 01S01V00 - Mit Free-Air-Calibration auf 20.9% O2
  DEBUG_LEVEL: 0=off, 1=basic, 2=verbose
*/

// ---------------- Configuration ----------------
#define DEBUG_LEVEL 0     // *** ÄNDERE HIER: 0=aus, 1=basic, 2=verbose ***
#define ENABLE_SERIAL 1   // *** ÄNDERE HIER: 0=Serial komplett aus, 1=an ***

// Debug macros
#if DEBUG_LEVEL >= 1
  #define DBG1(x)   do { if (ENABLE_SERIAL) Serial.print(x); } while(0)
  #define DBGLN1(x) do { if (ENABLE_SERIAL) Serial.println(x); } while(0)
#else
  #define DBG1(x)
  #define DBGLN1(x)
#endif

#if DEBUG_LEVEL >= 2
  #define DBG2(x)   do { if (ENABLE_SERIAL) Serial.print(x); } while(0)
  #define DBGLN2(x) do { if (ENABLE_SERIAL) Serial.println(x); } while(0)
#else
  #define DBG2(x)
  #define DBGLN2(x)
#endif

// Serielle Ausgabe für Status (unabhängig von DEBUG_LEVEL)
#if ENABLE_SERIAL
  #define SERIAL_PRINT(x)   Serial.print(x)
  #define SERIAL_PRINTLN(x) Serial.println(x)
#else
  #define SERIAL_PRINT(x)
  #define SERIAL_PRINTLN(x)
#endif

// Supply threshold in Volts (start condition)
#define UBAT_MIN_V           9.0f

// Divider gain for your UB network: (Rtop+Rbot)/Rbot = (100k+10k)/10k = 11.0
#define UBAT_DIVIDER_GAIN    11.0f

// Set this to your measured 5V reference if needed (e.g., 4.98f). Leave 5.00f if unknown.
static float VREF_VOLTS = 5.046f;

// ---------------- Includes ----------------
#include <SPI.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>

// ---------------- LCD ----------------
LiquidCrystal_I2C lcd(0x27, 16, 2); // adjust to 0x3F if needed

// ---------------- CJ125 registers ----------------
#define CJ125_IDENT_REG_REQUEST         0x4800
#define CJ125_DIAG_REG_REQUEST          0x7800
#define CJ125_INIT_REG1_REQUEST         0x6C00
#define CJ125_INIT_REG2_REQUEST         0x7E00
#define CJ125_INIT_REG1_MODE_CALIBRATE  0x569D
#define CJ125_INIT_REG1_MODE_NORMAL_V8  0x5688
#define CJ125_INIT_REG1_MODE_NORMAL_V17 0x5689
#define CJ125_DIAG_REG_STATUS_OK        0x28FF
#define CJ125_DIAG_REG_STATUS_NOPOWER   0x2855
#define CJ125_DIAG_REG_STATUS_NOSENSOR  0x287F
#define CJ125_INIT_REG1_STATUS_0        0x2888
#define CJ125_INIT_REG1_STATUS_1        0x2889

// ---------------- Pins ----------------
#define CJ125_CS_PIN          10
#define HEATER_OUTPUT_PIN     6
#define ANALOG_OUTPUT_PIN     3
#define UB_ANALOG_INPUT_PIN   2
#define UR_ANALOG_INPUT_PIN   1
#define UA_ANALOG_INPUT_PIN   0

#define CAL_BUTTON_PIN        4   // Free-Air-Cal-Taster an D4 (gegen GND, INPUT_PULLUP)

// ---------------- Parameters ----------------
#define SERIAL_RATE           1    // Hz (1-100)

// ---------------- Globals ----------------
int adcValue_UA = 0;
int adcValue_UR = 0;
int adcValue_UB = 0;
int adcValue_UA_Optimal = 0;
int adcValue_UR_Optimal = 0;
int HeaterOutput = 0;
int serial_counter = 0;
int CJ125_Status = 0;

// Free-Air-Calibration Offset
int UA_Offset = 0;
bool ua_offset_valid = false;

// EEPROM Layout
const uint16_t EEPROM_MAGIC          = 0x4C53; // 'LS'
const int      EEPROM_ADDR_MAGIC     = 0;
const int      EEPROM_ADDR_UA_OFFSET = 2;

// PID variables
int dState;
int iState;
const int iMax = 250;
const int iMin = -250;
const float pGain = 120;
const float iGain = 0.8;
const float dGain = 10;

// ---------------- Lookup tables ----------------
const PROGMEM float Lambda_Conversion[753] = {
  0.750, 0.751, 0.752, 0.752, 0.753, 0.754, 0.755, 0.755, 0.756, 0.757, 0.758, 0.758, 0.759, 0.760, 0.761, 0.761, 0.762, 0.763, 0.764, 0.764,
  0.765, 0.766, 0.766, 0.767, 0.768, 0.769, 0.769, 0.770, 0.771, 0.772, 0.772, 0.773, 0.774, 0.774, 0.775, 0.776, 0.777, 0.777, 0.778, 0.779,
  0.780, 0.780, 0.781, 0.782, 0.782, 0.783, 0.784, 0.785, 0.785, 0.786, 0.787, 0.787, 0.788, 0.789, 0.790, 0.790, 0.791, 0.792, 0.793, 0.793,
  0.794, 0.795, 0.796, 0.796, 0.797, 0.798, 0.799, 0.799, 0.800, 0.801, 0.802, 0.802, 0.803, 0.804, 0.805, 0.805, 0.806, 0.807, 0.808, 0.808,
  0.809, 0.810, 0.811, 0.811, 0.812, 0.813, 0.814, 0.815, 0.815, 0.816, 0.817, 0.818, 0.819, 0.820, 0.820, 0.821, 0.822, 0.823, 0.824, 0.825,
  0.825, 0.826, 0.827, 0.828, 0.829, 0.830, 0.830, 0.831, 0.832, 0.833, 0.834, 0.835, 0.836, 0.837, 0.837, 0.838, 0.839, 0.840, 0.841, 0.842,
  0.843, 0.844, 0.845, 0.846, 0.846, 0.847, 0.848, 0.849, 0.850, 0.851, 0.852, 0.853, 0.854, 0.855, 0.855, 0.856, 0.857, 0.858, 0.859, 0.860,
  0.861, 0.862, 0.863, 0.864, 0.865, 0.865, 0.866, 0.867, 0.868, 0.869, 0.870, 0.871, 0.872, 0.873, 0.874, 0.875, 0.876, 0.877, 0.878, 0.878,
  0.879, 0.880, 0.881, 0.882, 0.883, 0.884, 0.885, 0.886, 0.887, 0.888, 0.889, 0.890, 0.891, 0.892, 0.893, 0.894, 0.895, 0.896, 0.897, 0.898,
  0.899, 0.900, 0.901, 0.902, 0.903, 0.904, 0.905, 0.906, 0.907, 0.908, 0.909, 0.910, 0.911, 0.912, 0.913, 0.915, 0.916, 0.917, 0.918, 0.919,
  0.920, 0.921, 0.922, 0.923, 0.924, 0.925, 0.926, 0.927, 0.928, 0.929, 0.931, 0.932, 0.933, 0.934, 0.935, 0.936, 0.937, 0.938, 0.939, 0.940,
  0.941, 0.942, 0.944, 0.945, 0.946, 0.947, 0.948, 0.949, 0.950, 0.951, 0.952, 0.953, 0.954, 0.955, 0.957, 0.958, 0.959, 0.960, 0.961, 0.962,
  0.963, 0.965, 0.966, 0.967, 0.969, 0.970, 0.971, 0.973, 0.974, 0.976, 0.977, 0.979, 0.980, 0.982, 0.983, 0.985, 0.986, 0.987, 0.989, 0.990,
  0.991, 0.992, 0.994, 0.995, 0.996, 0.998, 0.999, 1.001, 1.003, 1.005, 1.008, 1.010, 1.012, 1.015, 1.017, 1.019, 1.022, 1.024, 1.026, 1.028,
  1.030, 1.032, 1.035, 1.037, 1.039, 1.041, 1.043, 1.045, 1.048, 1.050, 1.052, 1.055, 1.057, 1.060, 1.062, 1.064, 1.067, 1.069, 1.072, 1.075,
  1.077, 1.080, 1.082, 1.085, 1.087, 1.090, 1.092, 1.095, 1.098, 1.100, 1.102, 1.105, 1.107, 1.110, 1.112, 1.115, 1.117, 1.120, 1.122, 1.124,
  1.127, 1.129, 1.132, 1.135, 1.137, 1.140, 1.142, 1.145, 1.148, 1.151, 1.153, 1.156, 1.159, 1.162, 1.165, 1.167, 1.170, 1.173, 1.176, 1.179,
  1.182, 1.185, 1.188, 1.191, 1.194, 1.197, 1.200, 1.203, 1.206, 1.209, 1.212, 1.215, 1.218, 1.221, 1.224, 1.227, 1.230, 1.234, 1.237, 1.240,
  1.243, 1.246, 1.250, 1.253, 1.256, 1.259, 1.262, 1.266, 1.269, 1.272, 1.276, 1.279, 1.282, 1.286, 1.289, 1.292, 1.296, 1.299, 1.303, 1.306,
  1.310, 1.313, 1.317, 1.320, 1.324, 1.327, 1.331, 1.334, 1.338, 1.342, 1.345, 1.349, 1.352, 1.356, 1.360, 1.364, 1.367, 1.371, 1.375, 1.379,
  1.382, 1.386, 1.390, 1.394, 1.398, 1.401, 1.405, 1.409, 1.413, 1.417, 1.421, 1.425, 1.429, 1.433, 1.437, 1.441, 1.445, 1.449, 1.453, 1.457,
  1.462, 1.466, 1.470, 1.474, 1.478, 1.483, 1.487, 1.491, 1.495, 1.500, 1.504, 1.508, 1.513, 1.517, 1.522, 1.526, 1.531, 1.535, 1.540, 1.544,
  1.549, 1.554, 1.558, 1.563, 1.568, 1.572, 1.577, 1.582, 1.587, 1.592, 1.597, 1.601, 1.606, 1.611, 1.616, 1.621, 1.627, 1.632, 1.637, 1.642,
  1.647, 1.652, 1.658, 1.663, 1.668, 1.674, 1.679, 1.684, 1.690, 1.695, 1.701, 1.707, 1.712, 1.718, 1.724, 1.729, 1.735, 1.741, 1.747, 1.753,
  1.759, 1.764, 1.770, 1.776, 1.783, 1.789, 1.795, 1.801, 1.807, 1.813, 1.820, 1.826, 1.832, 1.839, 1.845, 1.852, 1.858, 1.865, 1.872, 1.878,
  1.885, 1.892, 1.898, 1.905, 1.912, 1.919, 1.926, 1.933, 1.940, 1.947, 1.954, 1.961, 1.968, 1.975, 1.983, 1.990, 1.997, 2.005, 2.012, 2.020,
  2.027, 2.035, 2.042, 2.050, 2.058, 2.065, 2.073, 2.081, 2.089, 2.097, 2.105, 2.113, 2.121, 2.129, 2.137, 2.145, 2.154, 2.162, 2.171, 2.179,
  2.188, 2.196, 2.205, 2.214, 2.222, 2.231, 2.240, 2.249, 2.258, 2.268, 2.277, 2.286, 2.295, 2.305, 2.314, 2.324, 2.333, 2.343, 2.353, 2.363,
  2.373, 2.383, 2.393, 2.403, 2.413, 2.424, 2.434, 2.444, 2.455, 2.466, 2.476, 2.487, 2.498, 2.509, 2.520, 2.532, 2.543, 2.554, 2.566, 2.577,
  2.589, 2.601, 2.613, 2.625, 2.637, 2.649, 2.662, 2.674, 2.687, 2.699, 2.712, 2.725, 2.738, 2.751, 2.764, 2.778, 2.791, 2.805, 2.819, 2.833,
  2.847, 2.861, 2.875, 2.890, 2.904, 2.919, 2.934, 2.949, 2.964, 2.979, 2.995, 3.010, 3.026, 3.042, 3.058, 3.074, 3.091, 3.107, 3.124, 3.141,
  3.158, 3.175, 3.192, 3.209, 3.227, 3.245, 3.263, 3.281, 3.299, 3.318, 3.337, 3.355, 3.374, 3.394, 3.413, 3.433, 3.452, 3.472, 3.492, 3.513,
  3.533, 3.554, 3.575, 3.597, 3.618, 3.640, 3.662, 3.684, 3.707, 3.730, 3.753, 3.776, 3.800, 3.824, 3.849, 3.873, 3.898, 3.924, 3.950, 3.976,
  4.002, 4.029, 4.056, 4.084, 4.112, 4.140, 4.169, 4.198, 4.228, 4.258, 4.288, 4.319, 4.350, 4.382, 4.414, 4.447, 4.480, 4.514, 4.548, 4.583,
  4.618, 4.654, 4.690, 4.726, 4.764, 4.801, 4.840, 4.879, 4.918, 4.958, 4.999, 5.040, 5.082, 5.124, 5.167, 5.211, 5.255, 5.299, 5.345, 5.391,
  5.438, 5.485, 5.533, 5.582, 5.632, 5.683 ,5.735, 5.788, 5.841, 5.896, 5.953, 6.010, 6.069, 6.129, 6.190, 6.253, 6.318, 6.384, 6.452, 6.521,
  6.592, 6.665, 6.740, 6.817, 6.896, 6.976, 7.059, 7.144, 7.231, 7.320, 7.412, 7.506, 7.602, 7.701, 7.803, 7.906, 8.013, 8.122, 8.234, 8.349,
  8.466, 8.587, 8.710, 8.837, 8.966, 9.099, 9.235, 9.374, 9.516, 9.662, 9.811, 9.963, 10.119
};

const PROGMEM float Oxygen_Conversion[548] = {
  00.00, 00.04, 00.08, 00.13, 00.17, 00.21, 00.25, 00.30, 00.34, 00.38, 00.42, 00.47, 00.51, 00.55, 00.59, 00.64, 00.68, 00.72, 00.76, 00.81,
  00.85, 00.89, 00.93, 00.98, 01.02, 01.06, 01.10, 01.15, 01.19, 01.23, 01.27, 01.31, 01.36, 01.40, 01.44, 01.48, 01.53, 01.57, 01.61, 01.65,
  01.70, 01.74, 01.78, 01.82, 01.86, 01.91, 01.95, 01.99, 02.03, 02.08, 02.12, 02.16, 02.20, 02.24, 02.29, 02.33, 02.37, 02.41, 02.45, 02.50,
  02.54, 02.58, 02.62, 02.66, 02.71, 02.75, 02.79, 02.83, 02.87, 02.92, 02.96, 03.00, 03.04, 03.08, 03.13, 03.17, 03.21, 03.25, 03.29, 03.33,
  03.38, 03.42, 03.46, 03.50, 03.54, 03.58, 03.63, 03.67, 03.71, 03.75, 03.79, 03.83, 03.88, 03.92, 03.96, 04.00, 04.04, 04.08, 04.12, 04.17,
  04.21, 04.25, 04.29, 04.33, 04.37, 04.41, 04.45, 04.50, 04.54, 04.58, 04.62, 04.66, 04.70, 04.74, 04.78, 04.82, 04.86, 04.91, 04.95, 04.99,
  05.03, 05.07, 05.11, 05.15, 05.19, 05.23, 05.27, 05.31, 05.35, 05.39, 05.44, 05.48, 05.52, 05.56, 05.60, 05.64, 05.68, 05.72, 05.76, 05.80,
  05.84, 05.88, 05.92, 05.96, 06.00, 06.04, 06.08, 06.12, 06.16, 06.20, 06.24, 06.28, 06.32, 06.36, 06.40, 06.44, 06.48, 06.52, 06.56, 06.60,
  06.64, 06.68, 06.72, 06.76, 06.80, 06.84, 06.88, 06.92, 06.96, 07.00, 07.03, 07.07, 07.11, 07.15, 07.19, 07.23, 07.27, 07.31, 07.35, 07.39,
  07.43, 07.47, 07.51, 07.55, 07.59, 07.62, 07.66, 07.70, 07.74, 07.78, 07.82, 07.86, 07.90, 07.94, 07.98, 08.02, 08.06, 08.09, 08.13, 08.17,
  08.21, 08.25, 08.29, 08.33, 08.37, 08.41, 08.45, 08.49, 08.52, 08.56, 08.60, 08.64, 08.68, 08.72, 08.76, 08.80, 08.84, 08.88, 08.91, 08.95,
  08.99, 09.03, 09.07, 09.11, 09.15, 09.19, 09.23, 09.26, 09.30, 09.34, 09.38, 09.42, 09.46, 09.50, 09.54, 09.57, 09.61, 09.65, 09.69, 09.73,
  09.77, 09.81, 09.85, 09.89, 09.92, 09.96, 10.00, 10.04, 10.08, 10.12, 10.16, 10.19, 10.23, 10.27, 10.31, 10.35, 10.39, 10.43, 10.47, 10.50,
  10.54, 10.58, 10.62, 10.66, 10.70, 10.73, 10.77, 10.81, 10.85, 10.89, 10.93, 10.97, 11.00, 11.04, 11.08, 11.12, 11.16, 11.20, 11.23, 11.27,
  11.31, 11.35, 11.39, 11.43, 11.46, 11.50, 11.54, 11.58, 11.62, 11.66, 11.69, 11.73, 11.77, 11.81, 11.85, 11.89, 11.92, 11.96, 12.00, 12.04,
  12.08, 12.11, 12.15, 12.19, 12.23, 12.27, 12.30, 12.34, 12.38, 12.42, 12.46, 12.49, 12.53, 12.57, 12.61, 12.65, 12.68, 12.72, 12.76, 12.80,
  12.84, 12.87, 12.91, 12.95, 12.99, 13.03, 13.06, 13.10, 13.14, 13.18, 13.21, 13.25, 13.29, 13.33, 13.36, 13.40, 13.44, 13.48, 13.51, 13.55,
  13.59, 13.63, 13.67, 13.70, 13.74, 13.78, 13.82, 13.85, 13.89, 13.93, 13.96, 14.00, 14.04, 14.08, 14.11, 14.15, 14.19, 14.23, 14.26, 14.30,
  14.34, 14.38, 14.41, 14.45, 14.49, 14.52, 14.56, 14.60, 14.64, 14.67, 14.71, 14.75, 14.78, 14.82, 14.86, 14.90, 14.93, 14.97, 15.01, 15.04,
  15.08, 15.12, 15.15, 15.19, 15.23, 15.26, 15.30, 15.34, 15.37, 15.41, 15.45, 15.48, 15.52, 15.56, 15.59, 15.63, 15.67, 15.70, 15.74, 15.78,
  15.81, 15.85, 15.89, 15.92, 15.96, 16.00, 16.03, 16.07, 16.11, 16.14, 16.18, 16.22, 16.25, 16.29, 16.32, 16.36, 16.40, 16.43, 16.47, 16.51,
  16.54, 16.58, 16.61, 16.65, 16.69, 16.72, 16.76, 16.79, 16.83, 16.87, 16.90, 16.94, 16.97, 17.01, 17.05, 17.08, 17.12, 17.15, 17.19, 17.22,
  17.26, 17.30, 17.33, 17.37, 17.40, 17.44, 17.47, 17.51, 17.55, 17.58, 17.62, 17.65, 17.69, 17.72, 17.76, 17.79, 17.83, 17.86, 17.90, 17.94,
  17.97, 18.01, 18.04, 18.08, 18.11, 18.15, 18.18, 18.22, 18.25, 18.29, 18.32, 18.36, 18.39, 18.43, 18.46, 18.50, 18.53, 18.57, 18.60, 18.64,
  18.67, 18.71, 18.74, 18.78, 18.81, 18.85, 18.88, 18.92, 18.95, 18.98, 19.02, 19.05, 19.09, 19.12, 19.16, 19.19, 19.23, 19.26, 19.30, 19.33,
  19.36, 19.40, 19.43, 19.47, 19.50, 19.54, 19.57, 19.60, 19.64, 19.67, 19.71, 19.74, 19.77, 19.81, 19.84, 19.88, 19.91, 19.94, 19.98, 20.01,
  20.05, 20.08, 20.11, 20.15, 20.18, 20.22, 20.25, 20.28, 20.32, 20.35, 20.38, 20.42, 20.45, 20.48, 20.52, 20.55, 20.58, 20.62, 20.65, 20.68,
  20.72, 20.75, 20.78, 20.82, 20.85, 20.88, 20.92, 20.95
};

// ---------------- SPI ----------------
uint16_t COM_SPI(uint16_t TX_data) {
  SPI.setDataMode(SPI_MODE1);
  SPI.setClockDivider(SPI_CLOCK_DIV128);
  digitalWrite(CJ125_CS_PIN, LOW);
  uint16_t Response = SPI.transfer16(TX_data);
  digitalWrite(CJ125_CS_PIN, HIGH);
  return Response;
}

// ---------------- Helpers ----------------
inline float ubat_from_adc(int adcUB) {
  delayMicroseconds(50);
  adcUB = analogRead(UB_ANALOG_INPUT_PIN);
  float vadc = ((float)adcUB / 1023.0f) * VREF_VOLTS;
  return vadc * UBAT_DIVIDER_GAIN;
}

// EEPROM-Helpers
void load_calibration() {
  uint16_t magic;
  EEPROM.get(EEPROM_ADDR_MAGIC, magic);

  if (magic == EEPROM_MAGIC) {
    EEPROM.get(EEPROM_ADDR_UA_OFFSET, UA_Offset);
    ua_offset_valid = true;
    DBGLN1("EEPROM: UA_Offset loaded");
    DBG1("EEPROM: UA_Offset="); DBGLN1(UA_Offset);
  } else {
    ua_offset_valid = false;
    UA_Offset = 0;
    DBGLN1("EEPROM: no valid cal, UA_Offset=0");
  }
}

void save_calibration() {
  EEPROM.put(EEPROM_ADDR_UA_OFFSET, UA_Offset);
  uint16_t magic = EEPROM_MAGIC;
  EEPROM.put(EEPROM_ADDR_MAGIC, magic);
  ua_offset_valid = true;

  DBGLN1("EEPROM: UA_Offset saved");
  DBG1("EEPROM: UA_Offset="); DBGLN1(UA_Offset);
}

// ---------------- PID ----------------
int Heater_PID_Control(int input) {
  int error = adcValue_UR_Optimal - input;
  int position = input;

  float pTerm = -pGain * error;
  iState += error;
  if (iState > iMax) iState = iMax;
  if (iState < iMin) iState = iMin;
  float iTerm = -iGain * iState;

  float dTerm = -dGain * (dState - position);
  dState = position;

  int RegulationOutput = pTerm + iTerm + dTerm;
  if (RegulationOutput > 255) RegulationOutput = 255;
  if (RegulationOutput < 0.0) RegulationOutput = 0;
  return RegulationOutput;
}

// ---------------- AFR analog 0-1V ----------------
void UpdateAnalogOutput() {
  const float AirFuelRatioOctane = 14.70;
  const int maximumOutput = 51; /* 1V */
  const int minimumOutput = 0;  /* 0V */
  int analogOutput = 0;
  float lambdaAFR = Lookup_Lambda(adcValue_UA) * AirFuelRatioOctane;
  analogOutput = map(lambdaAFR * 100, 2000, 1000, minimumOutput, maximumOutput);
  if (analogOutput > maximumOutput) analogOutput = maximumOutput;
  if (analogOutput < minimumOutput) analogOutput = minimumOutput;
  analogWrite(ANALOG_OUTPUT_PIN, analogOutput);
}

// ---------------- Lookups ----------------
float Lookup_Lambda(int Input_ADC) {
  float LAMBDA_VALUE = 0;
  if (Input_ADC >= 39 && Input_ADC <= 791) {
    LAMBDA_VALUE = pgm_read_float_near(Lambda_Conversion + (Input_ADC - 39));
  }
  if (Input_ADC > 791) LAMBDA_VALUE = 10.119;
  if (Input_ADC < 39)  LAMBDA_VALUE = 0.750;
  return LAMBDA_VALUE;
}

float Lookup_Oxygen(int Input_ADC) {
  float OXYGEN_CONTENT = 0;
  if (Input_ADC > 854) Input_ADC = 854;
  if (Input_ADC >= 307 && Input_ADC <= 854) {
    OXYGEN_CONTENT = pgm_read_float_near(Oxygen_Conversion + (Input_ADC - 307));
  }
  return OXYGEN_CONTENT;
}

// ---------------- LCD helpers ----------------
void lcd_print_boot(const char* line2) {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Lambda Shield");
  lcd.setCursor(0,1);
  lcd.print(line2);
}

// ohne lcd.clear() um Flackern zu reduzieren
void lcd_print_two(float ub_v, int pwm, float lambda, float o2, bool lambda_valid, bool o2_valid) {
  lcd.setCursor(0,0);
  lcd.print("UB ");
  lcd.print(ub_v, 1);
  lcd.print("V AFR");

  const float AirFuelRatioOctane = 14.70;
  float afr = Lookup_Lambda(adcValue_UA) * AirFuelRatioOctane;
  if (lambda_valid) {
    lcd.print(afr, 1);
  } else {
    lcd.print("  - ");
  }

  lcd.setCursor(0,1);
  lcd.print("L");
  if (lambda_valid) {
    lcd.print(lambda, 3);
  } else {
    lcd.print("-");
  }
  lcd.print(" O2 ");
  if (o2_valid) {
    lcd.print(o2, 1);
    lcd.print("%");
  } else {
    lcd.print("-");
  }
}

// ---------------- Free-Air-Calibration ----------------
void calibrate_free_air() {
  DBGLN1("FREE AIR CAL: Starting calibration on 20.9% O2");
  lcd_print_boot("Free Air Cal...");

  DBGLN1("FREE AIR CAL: Waiting for sensor to stabilize (30s)");
  for (int i = 0; i < 30; i++) {
    adcValue_UR = analogRead(UR_ANALOG_INPUT_PIN);
    HeaterOutput = Heater_PID_Control(adcValue_UR);
    analogWrite(HEATER_OUTPUT_PIN, HeaterOutput);
    delay(1000);

    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Stabilizing...");
    lcd.setCursor(0,1);
    lcd.print("Wait: ");
    lcd.print(30 - i);
    lcd.print("s");
  }

  long ua_sum = 0;
  for (int i = 0; i < 10; i++) {
    ua_sum += analogRead(UA_ANALOG_INPUT_PIN);
    delay(100);
  }
  int ua_average = ua_sum / 10;

  const int TARGET_ADC_20_9_PERCENT = 854;
  UA_Offset = TARGET_ADC_20_9_PERCENT - ua_average;

  DBG1("FREE AIR CAL: Measured UA_ADC="); DBGLN1(ua_average);
  DBG1("FREE AIR CAL: Target UA_ADC="); DBGLN1(TARGET_ADC_20_9_PERCENT);
  DBG1("FREE AIR CAL: Calculated Offset="); DBGLN1(UA_Offset);

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Cal Complete!");
  lcd.setCursor(0,1);
  lcd.print("Offset: ");
  lcd.print(UA_Offset);
  delay(3000);

  // im EEPROM speichern
  save_calibration();
}

// ---------------- Start sequence ----------------
void start() {
  DBGLN1("WAIT: Supply & CJ125 OK check...");
  lcd_print_boot("Wait CJ125/UB");

  unsigned long start_ms = millis();
  const unsigned long wait_timeout_ms = 10000; // 10s

  while (true) {
    CJ125_Status = COM_SPI(CJ125_DIAG_REG_REQUEST);

    adcValue_UB = analogRead(UB_ANALOG_INPUT_PIN);
    float UB_V = ubat_from_adc(adcValue_UB);

    DBG2("WAIT: CJ125=0x"); DBG2(String(CJ125_Status, HEX));
    DBG2(", UB_ADC="); DBG2(adcValue_UB);
    DBG2(", UB_V="); DBGLN2(String(UB_V, 2));

    if (CJ125_Status != CJ125_DIAG_REG_STATUS_OK) {
      DBGLN1("WARN: CJ125 not OK yet");
      lcd_print_boot("CJ125 not OK");
    }
    if (UB_V < UBAT_MIN_V) {
      DBGLN1("WARN: Supply below UBAT_MIN_V");
      lcd_print_boot("Low supply UB");
      delay(500);
      continue;
    }
    if (CJ125_Status == CJ125_DIAG_REG_STATUS_OK && UB_V >= UBAT_MIN_V) break;

    if (millis() - start_ms > wait_timeout_ms) {
      DBGLN1("ERROR: CJ125/UB wait timeout");
      lcd_print_boot("CJ125/UB timeout");
      delay(2000);
      start_ms = millis(); // erneut versuchen
    }

    delay(500);
  }

  DBGLN1("READY: Supply and CJ125 OK");
  lcd_print_boot("CJ125 OK");

  DBGLN1("CAL: Enter CJ125 calibration mode");
  COM_SPI(CJ125_INIT_REG1_MODE_CALIBRATE);
  lcd_print_boot("Calibrate...");
  delay(500);

  adcValue_UA_Optimal = analogRead(UA_ANALOG_INPUT_PIN);
  adcValue_UR_Optimal = analogRead(UR_ANALOG_INPUT_PIN);
  adcValue_UA = adcValue_UA_Optimal;
  UpdateAnalogOutput();

  DBG1("CAL: UA_Optimal ADC="); DBGLN1(adcValue_UA_Optimal);
  DBG1("CAL: UR_Optimal ADC="); DBGLN1(adcValue_UR_Optimal);
  DBG1("CAL: UA_Optimal λ="); DBGLN1(String(Lookup_Lambda(adcValue_UA_Optimal), 3));

  DBGLN1("MODE: Enter normal operation (V=17)");
  COM_SPI(CJ125_INIT_REG1_MODE_NORMAL_V17);
  lcd_print_boot("Mode V=17");

  DBGLN1("HEAT: Start heating (condensation phase)");
  float UB_V_now = ubat_from_adc(adcValue_UB);
  int CondensationPWM = (int)((2.0f / UB_V_now) * 255.0f);
  if (CondensationPWM < 0) CondensationPWM = 0;
  if (CondensationPWM > 255) CondensationPWM = 255;
  analogWrite(HEATER_OUTPUT_PIN, CondensationPWM);
  lcd_print_boot("Heat: condens");

  int t = 0;
  while (t < 5 && ubat_from_adc(adcValue_UB) >= UBAT_MIN_V) {
    DBGLN2("HEAT: Condensation tick");
    delay(500);
    delay(500);
    t += 1;
  }

  DBGLN1("HEAT: Ramp-up +0.4V/s from 8.5V until 100% or 12V");
  lcd_print_boot("Heat: ramp");
  float UHeater = 8.5f;
  while (UHeater < 12.0f && ubat_from_adc(adcValue_UB) >= UBAT_MIN_V) {
    UB_V_now = ubat_from_adc(adcValue_UB);
    CondensationPWM = (int)((UHeater / UB_V_now) * 255.0f);
    if (CondensationPWM > 255) CondensationPWM = 255;
    if (CondensationPWM < 0) CondensationPWM = 0;
    analogWrite(HEATER_OUTPUT_PIN, CondensationPWM);
    DBG2("HEAT: Ramp U="); DBG2(UHeater); DBG2("V, PWM="); DBGLN2(CondensationPWM);
    delay(500);
    delay(500);
    UHeater += 0.4f;
  }

  DBGLN1("HEAT: Warm until UR <= UR_Optimal (hot)");
  lcd_print_boot("Heat: warmup");
  while (analogRead(UR_ANALOG_INPUT_PIN) > adcValue_UR_Optimal && ubat_from_adc(adcValue_UB) >= UBAT_MIN_V) {
    int ur = analogRead(UR_ANALOG_INPUT_PIN);
    DBG2("HEAT: UR now "); DBGLN2(ur);
    delay(500);
    delay(500);
  }

  DBGLN1("HEAT: Heating phase complete, hand over to PID");
  lcd_print_boot("PID active");
  analogWrite(HEATER_OUTPUT_PIN, 0);

  // Keine automatische Free-Air-Cal hier.
  // UA_Offset wird aus EEPROM verwendet.
}

// ---------------- Setup ----------------
void setup() {
  #if ENABLE_SERIAL
    Serial.begin(9600);
    delay(50);
  #endif

  DBGLN1("BOOT: Device reset");
  DBG2("BOOT: DEBUG_LEVEL="); DBGLN2(DEBUG_LEVEL);
  DBG2("BOOT: VREF_VOLTS="); DBGLN2(VREF_VOLTS);

  pinMode(CAL_BUTTON_PIN, INPUT_PULLUP);

  Wire.begin();
  lcd.init();
  lcd.backlight();
  lcd_print_boot("Boot...");

  SPI.begin();
  SPI.setBitOrder(MSBFIRST);

  pinMode(CJ125_CS_PIN, OUTPUT);
  pinMode(HEATER_OUTPUT_PIN, OUTPUT);

  digitalWrite(CJ125_CS_PIN, HIGH);
  analogWrite(HEATER_OUTPUT_PIN, 0);
  analogWrite(ANALOG_OUTPUT_PIN, 0);

  load_calibration();

  if (ua_offset_valid) {
    lcd_print_boot("Using cal");
    delay(1000);
  } else {
    lcd_print_boot("No cal yet");
    delay(1000);
  }

  DBGLN1("INIT: Starting main sequence");
  start();
}

// ---------------- Loop ----------------
void loop() {
  CJ125_Status = COM_SPI(CJ125_DIAG_REG_REQUEST);

  int raw_UA = analogRead(UA_ANALOG_INPUT_PIN);
  adcValue_UA = raw_UA + UA_Offset;
  if (adcValue_UA < 0) adcValue_UA = 0;
  if (adcValue_UA > 1023) adcValue_UA = 1023;

  adcValue_UR = analogRead(UR_ANALOG_INPUT_PIN);
  adcValue_UB = analogRead(UB_ANALOG_INPUT_PIN);

  float UB_V_now = ubat_from_adc(adcValue_UB);

  // Kalibrier-Taster abfragen (Flankenerkennung)
  static bool cal_button_last = HIGH;
  bool cal_button_now = digitalRead(CAL_BUTTON_PIN);

  if (cal_button_last == HIGH && cal_button_now == LOW) {
    DBGLN1("BUTTON: Free-Air-Cal requested");

    if (CJ125_Status == CJ125_DIAG_REG_STATUS_OK && UB_V_now >= UBAT_MIN_V) {
      lcd_print_boot("Free Air Cal");
      calibrate_free_air();
    } else {
      DBGLN1("BUTTON: Cal denied (CJ125/UB not OK)");
      lcd_print_boot("Cal denied");
      delay(1000);
    }
  }
  cal_button_last = cal_button_now;

  // PID control nur bei gültiger UR_Optimal und ausreichender Spannung
  bool pid_enabled = (adcValue_UR_Optimal > 0) && (UB_V_now >= UBAT_MIN_V);

  if (pid_enabled) {
    HeaterOutput = Heater_PID_Control(adcValue_UR);
    analogWrite(HEATER_OUTPUT_PIN, HeaterOutput);
    DBG2("PID: UR="); DBG2(adcValue_UR);
    DBG2(", URopt="); DBG2(adcValue_UR_Optimal);
    DBG2(", PWM="); DBGLN2(HeaterOutput);
  } else {
    HeaterOutput = 0;
    analogWrite(HEATER_OUTPUT_PIN, HeaterOutput);
    DBGLN2("PID: Disabled (preconditions not met)");
  }

  if (UB_V_now < UBAT_MIN_V) {
    DBGLN1("ERROR: Low power -> restart sequence");
    lcd_print_boot("Low UB -> restart");
    HeaterOutput = 0;
    analogWrite(HEATER_OUTPUT_PIN, HeaterOutput);
    start();
    return;
  }

  if ((100 / SERIAL_RATE) == serial_counter) {
    serial_counter = 0;

    float LAMBDA_VALUE = Lookup_Lambda(adcValue_UA);
    float OXYGEN_CONTENT = Lookup_Oxygen(adcValue_UA);
    UpdateAnalogOutput();

    if (CJ125_Status == CJ125_DIAG_REG_STATUS_OK) {
      String txString = "Measuring, CJ125: 0x";
      txString += String(CJ125_Status, HEX);
      txString += ", UA_ADC: ";
      txString += String(adcValue_UA, DEC);
      txString += " (raw: ";
      txString += String(raw_UA, DEC);
      txString += ", offset: ";
      txString += String(UA_Offset, DEC);
      txString += "), UR_ADC: ";
      txString += String(adcValue_UR, DEC);
      txString += ", UB_V: ";
      txString += String(UB_V_now, 2);

      if (adcValue_UA >= 39 && adcValue_UA <= 791) {
        txString += ", Lambda: ";
        txString += String(LAMBDA_VALUE, 3);
      } else {
        txString += ", Lambda: -";
      }

      if (adcValue_UA >= 307) {
        txString += ", Oxygen: ";
        txString += String(OXYGEN_CONTENT, 2);
        txString += "%";
      } else {
        txString += ", Oxygen: -";
      }

      SERIAL_PRINTLN(txString);

      if (DEBUG_LEVEL >= 2) {
        DBG2("STAT: CJ125=0x"); DBG2(String(CJ125_Status, HEX));
        DBG2(", UA="); DBG2(adcValue_UA);
        DBG2(", UR="); DBG2(adcValue_UR);
        DBG2(", UB_V="); DBG2(String(UB_V_now, 2));
        DBG2(", λ="); DBG2(String(LAMBDA_VALUE, 3));
        DBG2(", O2%="); DBGLN2(String(OXYGEN_CONTENT, 2));
      }

      bool lambda_valid = (adcValue_UA >= 39 && adcValue_UA <= 791);
      bool o2_valid = (adcValue_UA >= 307);
      lcd_print_two(UB_V_now, HeaterOutput, LAMBDA_VALUE, OXYGEN_CONTENT, lambda_valid, o2_valid);
    } else {
      switch (CJ125_Status) {
        case CJ125_DIAG_REG_STATUS_NOPOWER:
          DBGLN1("DIAG: CJ125 No Power (0x2855)");
          lcd_print_boot("CJ125: No Power");
        break;
        case CJ125_DIAG_REG_STATUS_NOSENSOR:
          DBGLN1("DIAG: CJ125 No Sensor (0x287F)");
          lcd_print_boot("CJ125: No Sensor");
        break;
        default:
          DBG1("DIAG: CJ125 status=0x"); DBGLN1(String(CJ125_Status, HEX));
          lcd_print_boot("CJ125 diag...");
        break;
      }
    }
  }

  serial_counter++;
  delay(10);
}
