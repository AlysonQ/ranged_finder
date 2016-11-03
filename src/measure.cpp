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
VL53L0X_Error SensorThreeStatus = VL53L0X_ERROR_NONE;
//VL53L0X_Dev_t MyDevice;
VL53L0X_Dev_t SensorOne;
VL53L0X_Dev_t SensorTwo;
VL53L0X_Dev_t SensorThree;
//VL53L0X_Dev_t *pMyDevice = &MyDevice;
VL53L0X_Dev_t *pSensorOne = &SensorOne;
VL53L0X_Dev_t *pSensorTwo = &SensorTwo;
VL53L0X_Dev_t *pSensorThree = &SensorThree;
//VL53L0X_Version_t                   Version;
VL53L0X_Version_t                   SensorOneVersion;
VL53L0X_Version_t                   SensorTwoVersion;
VL53L0X_Version_t                   SensorThreeVersion;
//VL53L0X_Version_t                  *pVersion   = &Version;
VL53L0X_Version_t                  *pSensorOneVersion   = &SensorOneVersion;
VL53L0X_Version_t                  *pSensorTwoVersion   = &SensorTwoVersion;
VL53L0X_Version_t                  *pSensorThreeVersion   = &SensorThreeVersion;
//VL53L0X_DeviceInfo_t                DeviceInfo;
VL53L0X_DeviceInfo_t                SensorOneDeviceInfo;
VL53L0X_DeviceInfo_t                SensorTwoDeviceInfo;
VL53L0X_DeviceInfo_t                SensorThreeDeviceInfo;

VL53L0X_RangingMeasurementData_t    SensorOneRangingMeasurementData;
VL53L0X_RangingMeasurementData_t    SensorTwoRangingMeasurementData;
VL53L0X_RangingMeasurementData_t    SensorThreeRangingMeasurementData;
uint32_t refSpadCount;
uint8_t isApertureSpads;
uint8_t VhvSettings;
uint8_t PhaseCal;

int main(int argc, char **argv)
{


    //Setting GPIO ---------- Alyson ----START
    ROS_INFO("GPIO Setting 01 :: Init GPIO");
    jetsonGPIO SensorOneGPIO = gpio164 ;
    jetsonGPIO SensorTwoGPIO = gpio165 ;
    jetsonGPIO SensorThreeGPIO = gpio166 ;
    usleep(1000000);
    ROS_INFO("GPIO Setting 02 :: Export GPIO");
    gpioExport(SensorOneGPIO) ;
    gpioExport(SensorTwoGPIO) ;
    gpioExport(SensorThreeGPIO) ;
    usleep(1000000);
    ROS_INFO("GPIO Setting 03 :: Set Direction = output");
    gpioSetDirection(SensorOneGPIO,outputPin) ;
    gpioSetDirection(SensorTwoGPIO,outputPin) ;
    gpioSetDirection(SensorThreeGPIO,outputPin) ;
    usleep(1000000);
    ROS_INFO("GPIO Setting 04  :: Set Value = off");
    gpioSetValue(SensorOneGPIO, off);
    usleep(1000000);
    gpioSetValue(SensorTwoGPIO, off);
    usleep(1000000);
    gpioSetValue(SensorThreeGPIO, off);
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
    ROS_INFO("Changing Address 01 :: SetAddress 0x16 ");
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
    usleep(1000000);
    ROS_INFO("Changing Address 02 :: Set Address 0x18 ");
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

    //Step 3 :: Set Sensor Three
    ROS_INFO("333333 ===============================================");
    gpioSetValue(SensorThreeGPIO, on);
    usleep(1000000);
    pSensorThree->I2cDevAddr      = DEV2_I2C_ADDR;
    pSensorThree->fd = VL53L0X_i2c_init("/dev/i2c-1", pSensorThree->I2cDevAddr);
    ROS_INFO("DataInit----------------------------------------- >START");
    VL53L0X_DataInit(&SensorThree);
    VL53L0X_StaticInit(&SensorThree);
    VL53L0X_PerformRefCalibration(pSensorThree,
        		&VhvSettings, &PhaseCal);
    VL53L0X_PerformRefSpadManagement(pSensorThree,
        		&refSpadCount, &isApertureSpads);
    ROS_INFO("DataInit----------------------------------------- >END");
    usleep(1000000);
    ROS_INFO("Changing Address 03 :: SetAddress 0x20 ");
    VL53L0X_SetDeviceAddress(&SensorThree,SensorThreeADDR);
    usleep(1000000);
    ROS_INFO("DataInit----------------------------------------- >START");
    pSensorThree->I2cDevAddr      = SensorThreeADDR;
    pSensorThree->fd = VL53L0X_i2c_init("/dev/i2c-1", pSensorThree->I2cDevAddr);
    VL53L0X_DataInit(&SensorThree);
    VL53L0X_StaticInit(&SensorThree);
    VL53L0X_PerformRefCalibration(pSensorThree,
        		&VhvSettings, &PhaseCal);
    VL53L0X_PerformRefSpadManagement(pSensorThree,
        		&refSpadCount, &isApertureSpads);
    ROS_INFO("DataInit----------------------------------------- >END");
    //GPIO Init---------- Alyson ----END

    //Step 4 :: Get Sensor Value
    ROS_INFO("4444 ===============================================");
  

    ROS_INFO("Changing Address Complete :: START read data");
    VL53L0X_SetDeviceMode(pSensorOne, VL53L0X_DEVICEMODE_SINGLE_RANGING);
    VL53L0X_SetDeviceMode(pSensorTwo, VL53L0X_DEVICEMODE_SINGLE_RANGING);
    VL53L0X_SetDeviceMode(pSensorThree, VL53L0X_DEVICEMODE_SINGLE_RANGING);
    //for (int count=1 ;count <=100 ;count++){ //Alyson Debug 
    while (ros::ok()){
    ROS_INFO("Reading Sensors Data =================================");
            VL53L0X_PerformSingleRangingMeasurement(pSensorOne,
                		&SensorOneRangingMeasurementData);
            ROS_INFO ("Sensor One distance = %d cm\n",SensorOneRangingMeasurementData.RangeMilliMeter*1000);
            VL53L0X_PerformSingleRangingMeasurement(pSensorTwo,
                		&SensorTwoRangingMeasurementData);
            ROS_INFO ("Sensor Two distance = %d cm\n",SensorTwoRangingMeasurementData.RangeMilliMeter*1000);
            VL53L0X_PerformSingleRangingMeasurement(pSensorThree,
                		&SensorThreeRangingMeasurementData);
            ROS_INFO ("Sensor Three distance = %d cm\n",SensorThreeRangingMeasurementData.RangeMilliMeter*1000);
            usleep(500000);
    }

    ROS_INFO("CHECK===============================================");
    usleep(5000000);
    //Step 4 :: Close ALL
    ROS_INFO("GPIO Setting 04  :: Unexport");

    VL53L0X_i2c_close();
    gpioUnexport(SensorOneGPIO);
    gpioUnexport(SensorTwoGPIO);
    gpioUnexport(SensorThreeGPIO);
    return (0);
}

