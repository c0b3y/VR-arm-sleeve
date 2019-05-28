/*3sensors.cpp*/
#include "SparkFunLSM9DS1.h"
#include <time.h>
#include <stdint.h>
#include <stdio.h>
#include <cstddef>
#include <math.h>
#include <inttypes.h>
//////////////////////////
// LSM9DS1 Library Init //
//////////////////////////
// Use the LSM9DS1 class to create an object. [imu] can be
// named anything, we'll refer to that throught the sketch.
LSM9DS1 imu1(0x6b, 0x1e, 1);
LSM9DS1 imu2(0x6a, 0x1c, 1);
LSM9DS1 imu3(0x6b, 0x1e, 0);
///////////////////////
// Example I2C Setup //
///////////////////////
// SDO_XM and SDO_G are both pulled high, so our addresses are:
#define LSM9DS1_M       0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG      0x6B // Would be 0x6A if SDO_AG is LOW
#define PI 3.141596
////////////////////////////
// Sketch Output Settings //
////////////////////////////
#define PRINT_SPEED 250 // 250 ms between prints
static unsigned long lastPrint = 0; // Keep track of print time

// Earth's magnetic field varies by location. Add or subtract
// a declination to get a more accurate heading. Calculate
// your's here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION -11.50 // Declination (degrees) in Boulder, CO.

void printGyro()
{
    // Now we can use the gx, gy, and gz variables as we please.
    // Either print them as raw ADC values, or calculated in DPS.
    printf("G: ");
    // If you want to print calculated values, you can use the
    // calcGyro helper function to convert a raw ADC value to
    // DPS. Give the function the value that you want to convert.
    printf("%.2f, %.2f, %.2f ", imu1.calcGyro(imu1.gx), imu1.calcGyro(imu1.gy), imu1.calcGyro(imu1.gz));
    printf("%.2f, %.2f, %.2f ", imu2.calcGyro(imu2.gx), imu1.calcGyro(imu2.gy), imu2.calcGyro(imu2.gz));
    printf("%.2f, %.2f, %.2f ", imu3.calcGyro(imu3.gx), imu1.calcGyro(imu3.gy), imu3.calcGyro(imu3.gz));
    printf(" deg/s\n");
}

void printAccel()
{
    // Now we can use the ax, ay, and az variables as we please.
    // Either print them as raw ADC values, or calculated in g's.
    printf("A: ");
    // If you want to print calculated values, you can use the
    // calcAccel helper function to convert a raw ADC value to
    // g's. Give the function the value that you want to convert.
    printf("%.2f, %.2f, %.2f ", imu1.calcAccel(imu1.ax), imu1.calcAccel(imu1.ay), imu1.calcAccel(imu1.az));
    printf("%.2f, %.2f, %.2f ", imu2.calcAccel(imu2.ax), imu2.calcAccel(imu2.ay), imu2.calcAccel(imu2.az));
    printf("%.2f, %.2f, %.2f ", imu3.calcAccel(imu3.ax), imu3.calcAccel(imu3.ay), imu3.calcAccel(imu3.az));
    printf(" g\n");
}

void printMag()
{
    // Now we can use the mx, my, and mz variables as we please.
    // Either print them as raw ADC values, or calculated in Gauss.
    printf("M: ");
    // If you want to print calculated values, you can use the
    // calcMag helper function to convert a raw ADC value to
    // Gauss. Give the function the value that you want to convert.
    printf("%.2f, %.2f, %.2f ", imu1.calcMag(imu1.mx), imu1.calcMag(imu1.my), imu1.calcMag(imu1.mz));
    printf("%.2f, %.2f, %.2f ", imu2.calcMag(imu2.mx), imu2.calcMag(imu2.my), imu2.calcMag(imu2.mz));
    printf("%.2f, %.2f, %.2f ", imu3.calcMag(imu3.mx), imu3.calcMag(imu3.my), imu3.calcMag(imu3.mz));
    printf(" gauss\n");
}

void printAttitude(float ax, float ay, float az, float mx, float my, float mz)
{
    float roll = atan2(ay, az);
    float pitch = atan2(-ax, sqrt(ay * ay + az * az));

    float heading;
    if (my == 0)
        heading = (mx < 0) ? PI : 0;
    else
        heading = atan2(mx, my);

    heading -= DECLINATION * PI / 180;

    if (heading > PI) heading -= (2 * PI);
    else if (heading < -PI) heading += (2 * PI);

    // Convert everything from radians to degrees:
    heading *= 180.0 / PI;
    pitch *= 180.0 / PI;
    roll  *= 180.0 / PI;

    printf("Pitch, Roll: %.2f, %.2f; Heading: %.2f\n", pitch, roll, heading);
}

int main() {
    int count = 0;

    imu1.settings.device.commInterface = IMU_MODE_I2C;
    imu1.settings.device.mAddress = 0x1e;
    imu1.settings.device.agAddress = 0x6b;

    imu2.settings.device.commInterface = IMU_MODE_I2C;
    imu2.settings.device.mAddress = 0x1c;
    imu2.settings.device.agAddress = 0x6a;

    imu3.settings.device.commInterface = IMU_MODE_I2C;
    imu3.settings.device.mAddress = 0x1e;
    imu3.settings.device.agAddress = 0x6b;



    if (!imu1.begin() || !imu2.begin() || !imu3.begin()){
        printf("Failed to communicate with LSM9DS1.\n");
        printf("Double-check wiring.\n");
        printf("Default settings in this sketch will work for an out of the box LSM9DS1 Breakout, but may need to be modified if the board jumpers are.\n");
        return 0;
   }

    while(1){
        if ( imu1.gyroAvailable() && imu2.gyroAvailable() && imu3.gyroAvailable() ){
        // To read from the gyroscope,  first call the
        // readGyro() function. When it exits, it'll update the
        // gx, gy, and gz variables with the most current data.
            imu1.readGyro();
            imu2.readGyro();
            imu3.readGyro();
        }
        if ( imu1.accelAvailable() && imu2.accelAvailable() ){
        // To read from the accelerometer, first call the
        // readAccel() function. When it exits, it'll update the
        // ax, ay, and az variables with the most current data.
            imu1.readAccel();
            imu2.readAccel();
            imu3.readAccel();
        }
        if ( imu1.magAvailable() && imu2.magAvailable() && imu3.magAvailable() ){
        // To read from the magnetometer, first call the
        // readMag() function. When it exits, it'll update the
        // mx, my, and mz variables with the most current data.
            imu1.readMag();
            imu2.readMag();
            imu3.readMag();
        }
        printGyro();
        printAccel();
        printMag();
        printAttitude(imu1.ax, imu1.ay, imu1.az, -imu1.my, -imu1.mx, imu1.mz);
        printAttitude(imu2.ax, imu2.ay, imu2.az, -imu2.my, -imu2.mx, imu2.mz);
        printAttitude(imu3.ax, imu3.ay, imu3.az, -imu3.my, -imu3.mx, imu3.mz);
        printf("\n");
    }
}
