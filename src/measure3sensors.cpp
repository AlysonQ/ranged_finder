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
#include "ranged_finder/getSensorData.h"
#include "ranged_finder/RangedFinder.h"

 
#define DEV2_I2C_ADDR 0x29
#define SensorOneADDR 0x16
#define SensorTwoADDR 0x18
#define SensorThreeADDR 0x20

#define sleeptime  100000

jetsonGPIO SensorOneGPIO = gpio164 ;
jetsonGPIO SensorTwoGPIO = gpio165 ;
jetsonGPIO SensorThreeGPIO = gpio166 ;

VL53L0X_Dev_t SensorOne;
VL53L0X_Dev_t SensorTwo;
VL53L0X_Dev_t SensorThree;

VL53L0X_Dev_t *pSensorOne = &SensorOne;
VL53L0X_Dev_t *pSensorTwo = &SensorTwo;
VL53L0X_Dev_t *pSensorThree = &SensorThree;

VL53L0X_RangingMeasurementData_t    SensorOneRangingMeasurementData;
VL53L0X_RangingMeasurementData_t    SensorTwoRangingMeasurementData;
VL53L0X_RangingMeasurementData_t    SensorThreeRangingMeasurementData;

uint32_t refSpadCount;
uint8_t isApertureSpads;
uint8_t VhvSettings;
uint8_t PhaseCal;

ranged_finder::RangedFinder sensorData;

bool getSensorData(ranged_finder::getSensorData::Request &req,
                   ranged_finder::getSensorData::Response &res)
{
    ROS_INFO("[Alyson] getSensorDataService !!\n");
    res.Left = SensorOneRangingMeasurementData.RangeMilliMeter;
    res.Middle = SensorTwoRangingMeasurementData.RangeMilliMeter;
    res.Right = SensorThreeRangingMeasurementData.RangeMilliMeter;
    //ROS_INFO("res.Left=%d \n",res.Left);
    //ROS_INFO("res.Middle=%d \n",res.Middle);
    //ROS_INFO("res.Right=%d \n",res.Right);
    return true;
}

void GPIO_Setup()
{
    printf("\n\n>>>>>>>>>>>>>>>  GPIO  Setup <<<<<<<<<<<<<<<<<\n\n");
    ROS_INFO("GPIO Setting  :: Export GPIO");
    gpioExport(SensorOneGPIO) ;
    gpioExport(SensorTwoGPIO) ;
    gpioExport(SensorThreeGPIO) ;
    usleep(sleeptime);

    ROS_INFO("GPIO Setting  :: Set Direction = output");
    gpioSetDirection(SensorOneGPIO,outputPin) ;
    gpioSetDirection(SensorTwoGPIO,outputPin) ;
    gpioSetDirection(SensorThreeGPIO,outputPin) ;
    usleep(sleeptime);

    ROS_INFO("GPIO Setting  :: Set Value = off");
    gpioSetValue(SensorOneGPIO, off);
    usleep(sleeptime);
    gpioSetValue(SensorTwoGPIO, off);
    usleep(sleeptime);
    gpioSetValue(SensorThreeGPIO, off);
    usleep(sleeptime);
}

void Sensor_Setup()
{
    printf("\n\n>>>>>>>>>>>>>>>  Sensor Setup <<<<<<<<<<<<<<<<<\n\n");
    //Step 1 :: Set Sensor One
    //ROS_INFO(">>>>>>>>>>>>>>> [Setting Sensor One]---------- >START");
    gpioSetValue(SensorOneGPIO, on);
    usleep(sleeptime);
    pSensorOne->I2cDevAddr      = DEV2_I2C_ADDR;
    pSensorOne->fd = VL53L0X_i2c_init("/dev/i2c-1", pSensorOne->I2cDevAddr);
    //ROS_INFO("DataInit----------------------------------------- >START");
    VL53L0X_DataInit(&SensorOne);
    VL53L0X_StaticInit(&SensorOne);
    VL53L0X_PerformRefCalibration(pSensorOne,
        		&VhvSettings, &PhaseCal);
    VL53L0X_PerformRefSpadManagement(pSensorOne,
        		&refSpadCount, &isApertureSpads);
    //ROS_INFO("DataInit----------------------------------------- >END");
    usleep(sleeptime);
    ROS_INFO("Changing Address 01 :: SetAddress 0x16 ");
    VL53L0X_SetDeviceAddress(&SensorOne,SensorOneADDR);
    usleep(sleeptime);
    //ROS_INFO("DataInit----------------------------------------- >START");
    pSensorOne->I2cDevAddr      = SensorOneADDR;
    pSensorOne->fd = VL53L0X_i2c_init("/dev/i2c-1", pSensorOne->I2cDevAddr);
    VL53L0X_DataInit(&SensorOne);
    VL53L0X_StaticInit(&SensorOne);
    VL53L0X_PerformRefCalibration(pSensorOne,
        		&VhvSettings, &PhaseCal);
    VL53L0X_PerformRefSpadManagement(pSensorOne,
        		&refSpadCount, &isApertureSpads);
    //ROS_INFO("DataInit----------------------------------------- >END");
    //ROS_INFO(">>>>>>>>>>>>>>> [Setting Sensor One]---------- >END");

    //Step 2 :: Set Sensor Two
    //ROS_INFO(">>>>>>>>>>>>>>> [Setting Sensor Two]---------- >START");
    gpioSetValue(SensorTwoGPIO, on);
    usleep(sleeptime);
    pSensorTwo->I2cDevAddr      = DEV2_I2C_ADDR;
    pSensorTwo->fd = VL53L0X_i2c_init("/dev/i2c-1", pSensorTwo->I2cDevAddr);
    //ROS_INFO("DataInit----------------------------------------- >START");
    VL53L0X_DataInit(&SensorTwo);
    VL53L0X_StaticInit(&SensorTwo);
    VL53L0X_PerformRefCalibration(pSensorTwo,
        		&VhvSettings, &PhaseCal);
    VL53L0X_PerformRefSpadManagement(pSensorTwo,
        		&refSpadCount, &isApertureSpads);
    //ROS_INFO("DataInit----------------------------------------- >END");
    usleep(sleeptime);
    ROS_INFO("Changing Address 02 :: Set Address 0x18 ");
    VL53L0X_SetDeviceAddress(&SensorTwo,SensorTwoADDR);
    usleep(sleeptime); 
    //ROS_INFO("DataInit----------------------------------------- >START");
    pSensorTwo->I2cDevAddr      = SensorTwoADDR;
    pSensorTwo->fd = VL53L0X_i2c_init("/dev/i2c-1", pSensorTwo->I2cDevAddr);
    VL53L0X_DataInit(&SensorTwo);
    VL53L0X_StaticInit(&SensorTwo);
    VL53L0X_PerformRefCalibration(pSensorTwo,
        		&VhvSettings, &PhaseCal);
    VL53L0X_PerformRefSpadManagement(pSensorTwo,
        		&refSpadCount, &isApertureSpads);
    //ROS_INFO("DataInit----------------------------------------- >END");
    //ROS_INFO(">>>>>>>>>>>>>>> [Setting Sensor Two]---------- >END");

    //Step 3 :: Set Sensor Three
    //ROS_INFO(">>>>>>>>>>>>>>> [Setting Sensor Three]---------- >START");
    gpioSetValue(SensorThreeGPIO, on);
    usleep(sleeptime);
    pSensorThree->I2cDevAddr      = DEV2_I2C_ADDR;
    pSensorThree->fd = VL53L0X_i2c_init("/dev/i2c-1", pSensorThree->I2cDevAddr);
    //ROS_INFO("DataInit----------------------------------------- >START");
    VL53L0X_DataInit(&SensorThree);
    VL53L0X_StaticInit(&SensorThree);
    VL53L0X_PerformRefCalibration(pSensorThree,
        		&VhvSettings, &PhaseCal);
    VL53L0X_PerformRefSpadManagement(pSensorThree,
        		&refSpadCount, &isApertureSpads);
    //ROS_INFO("DataInit----------------------------------------- >END");
    usleep(sleeptime);
    ROS_INFO("Changing Address 03 :: SetAddress 0x20 ");
    VL53L0X_SetDeviceAddress(&SensorThree,SensorThreeADDR);
    usleep(sleeptime);
    //ROS_INFO("DataInit----------------------------------------- >START");
    pSensorThree->I2cDevAddr      = SensorThreeADDR;
    pSensorThree->fd = VL53L0X_i2c_init("/dev/i2c-1", pSensorThree->I2cDevAddr);
    VL53L0X_DataInit(&SensorThree);
    VL53L0X_StaticInit(&SensorThree);
    VL53L0X_PerformRefCalibration(pSensorThree,
        		&VhvSettings, &PhaseCal);
    VL53L0X_PerformRefSpadManagement(pSensorThree,
        		&refSpadCount, &isApertureSpads);
    //ROS_INFO("DataInit----------------------------------------- >END");
    //ROS_INFO(">>>>>>>>>>>>>>> [Setting Sensor Three]---------- >END");
}

void Sensor_Calibration (VL53L0X_Dev_t *pDevice)
{
    VL53L0X_SetDeviceMode(pDevice, VL53L0X_DEVICEMODE_SINGLE_RANGING);
    VL53L0X_SetLimitCheckEnable(pDevice,
        		    VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
    VL53L0X_SetLimitCheckEnable(pDevice,
        		    VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
    VL53L0X_SetLimitCheckValue(pDevice,
        		    VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
        		    (FixPoint1616_t)(0.1*65536));
    VL53L0X_SetLimitCheckValue(pDevice,
        		    VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
        		    (FixPoint1616_t)(60*65536));
    VL53L0X_SetMeasurementTimingBudgetMicroSeconds(pDevice,
        		    33000);
    VL53L0X_SetVcselPulsePeriod(pDevice, 
		            VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
    VL53L0X_SetVcselPulsePeriod(pDevice, 
		            VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);

}


void Start_Ranging()
{
    //printf("\n\n***************Start ranging***************\n\n");
    VL53L0X_PerformSingleRangingMeasurement(pSensorOne,
                		&SensorOneRangingMeasurementData);
    //ROS_INFO ("Sensor One distance = %d \n",SensorOneRangingMeasurementData.RangeMilliMeter);
    VL53L0X_PerformSingleRangingMeasurement(pSensorTwo,
                		&SensorTwoRangingMeasurementData);
    //ROS_INFO ("Sensor Two distance = %d \n",SensorTwoRangingMeasurementData.RangeMilliMeter);
    VL53L0X_PerformSingleRangingMeasurement(pSensorThree,
                		&SensorThreeRangingMeasurementData);
    //ROS_INFO ("Sensor Three distance = %d \n",SensorThreeRangingMeasurementData.RangeMilliMeter);
    sensorData.Left = SensorOneRangingMeasurementData.RangeMilliMeter;
    sensorData.Middle = SensorTwoRangingMeasurementData.RangeMilliMeter;
    sensorData.Right = SensorThreeRangingMeasurementData.RangeMilliMeter;
    
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "ranged_finder");
    ros::NodeHandle nh;
    ros::Publisher rang_pub = nh.advertise<ranged_finder::RangedFinder>("/ranged_finder_data", 1);
    ros::ServiceServer getSensorDataService = nh.advertiseService("get_sensor_data_service" ,getSensorData);
   
    GPIO_Setup();
    Sensor_Setup();

    //Step 4 :: Calibration Sensor and Get Sensor Value 
    Sensor_Calibration(pSensorOne);
    Sensor_Calibration(pSensorTwo);
    Sensor_Calibration(pSensorThree);

    printf("\n\n***************Start ranging***************\n\n");
    while (ros::ok()){
        Start_Ranging();
        rang_pub.publish(sensorData);
        ros::spinOnce();
    }

    ROS_INFO("GPIO Unexport");
    VL53L0X_i2c_close();
    gpioUnexport(SensorOneGPIO);
    gpioUnexport(SensorTwoGPIO);
    gpioUnexport(SensorThreeGPIO);
    return (0);
}

