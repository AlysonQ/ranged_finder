#include <ros/ros.h>
#include "std_msgs/Int16.h"
#include <stdlib.h>
#include <stdio.h>
#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"
#include <unistd.h>
extern "C" {  
    #include "jetsonGPIO.h"  
}  

#define DEV1_I2C_ADDR 0x28 
#define DEV2_I2C_ADDR 0x29
#define SensorOneADDR 0x16
#define SensorTwoADDR 0x18
#define SensorThreeADDR 0x20

//VL53L0X_Error Status = VL53L0X_ERROR_NONE;
VL53L0X_Error SensorOneStatus = VL53L0X_ERROR_NONE;
VL53L0X_Error SensorTwoStatus = VL53L0X_ERROR_NONE;
//VL53L0X_Dev_t MyDevice;
VL53L0X_Dev_t SensorOne;
VL53L0X_Dev_t SensorTwo;
//VL53L0X_Dev_t *pMyDevice = &MyDevice;
VL53L0X_Dev_t *pSensorOne = &SensorOne;
VL53L0X_Dev_t *pSensorTwo = &SensorTwo;
//VL53L0X_Version_t                   Version;
VL53L0X_Version_t                   SensorOneVersion;
VL53L0X_Version_t                   SensorTwoVersion;
//VL53L0X_Version_t                  *pVersion   = &Version;
VL53L0X_Version_t                  *pSensorOneVersion   = &SensorOneVersion;
VL53L0X_Version_t                  *pSensorTwoVersion   = &SensorTwoVersion;
//VL53L0X_DeviceInfo_t                DeviceInfo;
VL53L0X_DeviceInfo_t                SensorOneDeviceInfo;
VL53L0X_DeviceInfo_t                SensorTwoDeviceInfo;

VL53L0X_RangingMeasurementData_t    SensorOneRangingMeasurementData;
VL53L0X_RangingMeasurementData_t    SensorTwoRangingMeasurementData;
uint32_t refSpadCount;
uint8_t isApertureSpads;
uint8_t VhvSettings;
uint8_t PhaseCal;

int main(int argc, char **argv)
{


    //Setting GPIO ---------- Alyson ----START
    ROS_INFO("GPIO Setting 01 :: Init GPIO");
    jetsonGPIO SensorOneGPIO = gpio165 ;
    jetsonGPIO SensorTwoGPIO = gpio166 ;
    usleep(1000000);
    ROS_INFO("GPIO Setting 02 :: Export GPIO");
    gpioExport(SensorOneGPIO) ;
    gpioExport(SensorTwoGPIO) ;
    usleep(1000000);
    ROS_INFO("GPIO Setting 03 :: Set Direction = output");
    gpioSetDirection(SensorOneGPIO,outputPin) ;
    gpioSetDirection(SensorTwoGPIO,outputPin) ;
    usleep(1000000);
    ROS_INFO("GPIO Setting 04  :: Set Value = off");
    gpioSetValue(SensorOneGPIO, off);
    usleep(1000000);
    gpioSetValue(SensorTwoGPIO, off);
    usleep(1000000);

    //Setting GPIO ---------- Alyson ----END

    int32_t status_int;
	
    ros::init(argc, argv, "laser_ros");

    ros::NodeHandle nh;
    //ros::Publisher rang_pub = nh.advertise<std_msgs::Int16>("mavros/laser_ranging", 1);

    //GPIO Init---------- Alyson ----START
    //Step 1 :: Set Sensor One
    ROS_INFO("11111 ===============================================");
    gpioSetValue(SensorOneGPIO, on);
    usleep(1000000);
    pSensorOne->I2cDevAddr      = DEV2_I2C_ADDR;
    pSensorOne->fd = VL53L0X_i2c_init("/dev/i2c-1", pSensorOne->I2cDevAddr);
    ROS_INFO("DataInit----------------------------------------- >START");
    VL53L0X_DataInit(&SensorOne);
    VL53L0X_StaticInit(&SensorOne);
    VL53L0X_PerformRefCalibration(pSensorOne,
        		&VhvSettings, &PhaseCal);
    VL53L0X_PerformRefSpadManagement(pSensorOne,
        		&refSpadCount, &isApertureSpads);
    ROS_INFO("DataInit----------------------------------------- >END");
    usleep(1000000);
    //gpioSetValue(SensorOneGPIO,off);
    usleep(1000000);
    ROS_INFO("Changing Address 01 :: SetAddress 0x16 :: GPIO_ONE = OFF :: GPIO_TWO = ON");
    VL53L0X_SetDeviceAddress(&SensorOne,SensorOneADDR);
    usleep(1000000);
    ROS_INFO("DataInit----------------------------------------- >START");
    pSensorOne->I2cDevAddr      = SensorOneADDR;
    pSensorOne->fd = VL53L0X_i2c_init("/dev/i2c-1", pSensorOne->I2cDevAddr);
    VL53L0X_DataInit(&SensorOne);
    VL53L0X_StaticInit(&SensorOne);
    VL53L0X_PerformRefCalibration(pSensorOne,
        		&VhvSettings, &PhaseCal);
    VL53L0X_PerformRefSpadManagement(pSensorOne,
        		&refSpadCount, &isApertureSpads);
    ROS_INFO("DataInit----------------------------------------- >END");
    //Step 2 :: Set Sensor Two
    ROS_INFO("22222 ===============================================");
    gpioSetValue(SensorTwoGPIO, on);
    usleep(1000000);
    pSensorTwo->I2cDevAddr      = DEV2_I2C_ADDR;
    pSensorTwo->fd = VL53L0X_i2c_init("/dev/i2c-1", pSensorTwo->I2cDevAddr);
    ROS_INFO("DataInit----------------------------------------- >START");
    VL53L0X_DataInit(&SensorTwo);
    VL53L0X_StaticInit(&SensorTwo);
    VL53L0X_PerformRefCalibration(pSensorTwo,
        		&VhvSettings, &PhaseCal);
    VL53L0X_PerformRefSpadManagement(pSensorTwo,
        		&refSpadCount, &isApertureSpads);
    ROS_INFO("DataInit----------------------------------------- >END");
    //gpioSetValue(SensorTwoGPIO,off);
    usleep(1000000);
    ROS_INFO("Changing Address 02 :: Set Address 0x18 ::GPIO_ONE = OFF :: GPIO_TWO = OFF");
    VL53L0X_SetDeviceAddress(&SensorTwo,SensorTwoADDR);
    usleep(1000000); 
    ROS_INFO("DataInit----------------------------------------- >START");
    pSensorTwo->I2cDevAddr      = SensorTwoADDR;
    pSensorTwo->fd = VL53L0X_i2c_init("/dev/i2c-1", pSensorTwo->I2cDevAddr);
    VL53L0X_DataInit(&SensorTwo);
    VL53L0X_StaticInit(&SensorTwo);
    VL53L0X_PerformRefCalibration(pSensorTwo,
        		&VhvSettings, &PhaseCal);
    VL53L0X_PerformRefSpadManagement(pSensorTwo,
        		&refSpadCount, &isApertureSpads);
    ROS_INFO("DataInit----------------------------------------- >END");
    //GPIO Init---------- Alyson ----END
    //Step 3 :: Get Sensor Value
    ROS_INFO("3 ===============================================");


    ROS_INFO("Changing Address Complete :: START read data");
   VL53L0X_SetDeviceMode(pSensorOne, VL53L0X_DEVICEMODE_SINGLE_RANGING);
   VL53L0X_SetDeviceMode(pSensorTwo, VL53L0X_DEVICEMODE_SINGLE_RANGING);
    //for (int count=1 ;count <=100 ;count++){ //Alyson Debug 
    //while (ros::ok()){

            VL53L0X_PerformSingleRangingMeasurement(pSensorOne,
                		&SensorOneRangingMeasurementData);
            ROS_INFO ("Sensor One distance = %d cm\n",SensorOneRangingMeasurementData.RangeMilliMeter*1000);
            VL53L0X_PerformSingleRangingMeasurement(pSensorTwo,
                		&SensorTwoRangingMeasurementData);
            ROS_INFO ("Sensor Two distance = %d cm\n",SensorTwoRangingMeasurementData.RangeMilliMeter*1000);
     //   }

    ROS_INFO("CHECK===============================================");
    usleep(5000000);
    //Step 4 :: Close ALL
    ROS_INFO("GPIO Setting 04  :: Unexport");

    VL53L0X_i2c_close();
    gpioUnexport(SensorOneGPIO);
    gpioUnexport(SensorTwoGPIO);

    return (0);
}

