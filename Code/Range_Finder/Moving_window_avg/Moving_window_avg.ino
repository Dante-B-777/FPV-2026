#include <Wire.h>
#include "Adafruit_VL53L0X.h"

// ================================================================
// ESP32 I2C PINS
// ================================================================

#define I2C_SDA 22   // D22
#define I2C_SCL 21   // D21

// ================================================================
// SENSOR OBJECT
// ================================================================

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// ================================================================
// SETTINGS
// ================================================================

const int AVG_SAMPLES = 5;

const uint16_t MIN_VALID_MM = 30;
const uint16_t MAX_VALID_MM = 2200;

// Normal outlier rejection around previous average
const uint16_t OUTLIER_WINDOW_MM = 300;

// If all readings are rejected, but they are consistent with each other,
// accept the new cluster and reset the average.
const uint16_t CLUSTER_WINDOW_MM = 80;

// Timing budget.
// 20000 us = faster, noisier
// 33000 us = more stable, slower
const uint32_t TIMING_BUDGET_US = 20000;

// Maximum time spent trying to collect samples per update
const unsigned long SAMPLE_TIMEOUT_MS = 150;

// ================================================================
// DATA
// ================================================================

uint16_t last_raw_mm = 0;
uint8_t last_status = 255;

float avg_raw_mm = -1.0;
float avg_raw_m = -1.0;

int valid_count = 0;
int bad_count = 0;
int outlier_count = 0;
int reset_count = 0;

unsigned long lastPrint = 0;
const unsigned long PRINT_INTERVAL_MS = 20;

// ================================================================
// SETUP
// ================================================================

void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.println();
  Serial.println("VL53L0X Fast Raw Average With Outlier Unlock");
  Serial.println("No calibration. Rejects >2200 mm and handles big real jumps.");
  Serial.println();

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  Serial.println("Initializing VL53L0X...");

  if (!lox.begin()) {
    Serial.println("ERROR: VL53L0X not detected.");
    Serial.println("Check VIN, GND, SDA, SCL.");
    while (1) {
      delay(1000);
    }
  }

  lox.configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_LONG_RANGE);
  lox.setMeasurementTimingBudgetMicroSeconds(TIMING_BUDGET_US);

  Serial.println("VL53L0X OK.");
  Serial.println();
  Serial.println("Serial Plotter labels:");
  Serial.println("avg_raw_mm,avg_raw_m,last_raw_mm,valid_count,bad_count,outlier_count,reset_count,status");
}

// ================================================================
// READ ONE SAMPLE
// ================================================================

bool readOneRange(uint16_t &mm_out, uint8_t &status_out) {
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);

  uint16_t mm = measure.RangeMilliMeter;
  uint8_t status = measure.RangeStatus;

  mm_out = mm;
  status_out = status;

  if (status == 4) {
    return false;
  }

  if (mm < MIN_VALID_MM || mm > MAX_VALID_MM) {
    return false;
  }

  return true;
}

// ================================================================
// SIMPLE SORT
// ================================================================

void sortArray(uint16_t arr[], int n) {
  for (int i = 0; i < n - 1; i++) {
    for (int j = i + 1; j < n; j++) {
      if (arr[j] < arr[i]) {
        uint16_t temp = arr[i];
        arr[i] = arr[j];
        arr[j] = temp;
      }
    }
  }
}

// ================================================================
// FAST AVERAGE WITH UNLOCK
// ================================================================

void updateFastAverage() {
  uint16_t accepted[AVG_SAMPLES];
  uint16_t rejected[AVG_SAMPLES * 3];

  int accepted_count = 0;
  int rejected_count = 0;

  valid_count = 0;
  bad_count = 0;
  outlier_count = 0;
  reset_count = 0;

  unsigned long start_time = millis();

  while (accepted_count < AVG_SAMPLES) {
    uint16_t mm = 0;
    uint8_t status = 255;

    bool basic_ok = readOneRange(mm, status);

    last_raw_mm = mm;
    last_status = status;

    if (!basic_ok) {
      bad_count++;
    } else {
      bool accept = false;

      // First run: no previous average, accept any valid reading
      if (avg_raw_mm < 0) {
        accept = true;
      } else {
        float lower_limit = avg_raw_mm - OUTLIER_WINDOW_MM;
        float upper_limit = avg_raw_mm + OUTLIER_WINDOW_MM;

        if (mm >= lower_limit && mm <= upper_limit) {
          accept = true;
        }
      }

      if (accept) {
        accepted[accepted_count] = mm;
        accepted_count++;
      } else {
        outlier_count++;

        if (rejected_count < AVG_SAMPLES * 3) {
          rejected[rejected_count] = mm;
          rejected_count++;
        }
      }
    }

    if (millis() - start_time > SAMPLE_TIMEOUT_MS) {
      break;
    }
  }

  // Normal case: average accepted readings
  if (accepted_count > 0) {
    long sum = 0;

    for (int i = 0; i < accepted_count; i++) {
      sum += accepted[i];
    }

    avg_raw_mm = (float)sum / accepted_count;
    avg_raw_m = avg_raw_mm / 1000.0;
    valid_count = accepted_count;
    return;
  }

  // Unlock case:
  // If everything got rejected, check whether rejected values form a tight cluster.
  // Example: old avg = 1865, new readings = 90, 92, 94, 91.
  if (rejected_count >= 3) {
    sortArray(rejected, rejected_count);

    uint16_t low = rejected[0];
    uint16_t high = rejected[rejected_count - 1];

    if ((high - low) <= CLUSTER_WINDOW_MM) {
      long sum = 0;

      for (int i = 0; i < rejected_count; i++) {
        sum += rejected[i];
      }

      avg_raw_mm = (float)sum / rejected_count;
      avg_raw_m = avg_raw_mm / 1000.0;

      valid_count = rejected_count;
      reset_count = 1;
    }
  }
}

// ================================================================
// PRINT DATA
// ================================================================

void printData() {
  if (millis() - lastPrint < PRINT_INTERVAL_MS) {
    return;
  }

  lastPrint = millis();

  Serial.print("avg_raw_mm:");
  Serial.print(avg_raw_mm, 1);

  Serial.print(",avg_raw_m:");
  Serial.print(avg_raw_m, 3);

  Serial.print(",last_raw_mm:");
  Serial.print(last_raw_mm);

  Serial.print(",valid_count:");
  Serial.print(valid_count);

  Serial.print(",bad_count:");
  Serial.print(bad_count);

  Serial.print(",outlier_count:");
  Serial.print(outlier_count);

  Serial.print(",reset_count:");
  Serial.print(reset_count);

  Serial.print(",status:");
  Serial.println(last_status);
}

// ================================================================
// MAIN LOOP
// ================================================================

void loop() {
  updateFastAverage();
  printData();
}