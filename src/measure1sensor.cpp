#include <ros/ros.h>
#include "std_msgs/Int16.h"
#include <stdlib.h>
#include <stdio.h>
#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"


#define DEV1_I2C_ADDR 0x28 
#define DEV2_I2C_ADDR 0x29


void print_pal_error(VL53L0X_Error Status){
    char buf[VL53L0X_MAX_STRING_LENGTH];
    VL53L0X_GetPalErrorString(Status, buf);
    ROS_INFO("API Status: %i : %s\n", Status, buf);
}

void print_range_status(VL53L0X_RangingMeasurementData_t* pRangingMeasurementData){
    char buf[VL53L0X_MAX_STRING_LENGTH];
    uint8_t RangeStatus;

    /*
     * New Range Status: data is valid when pRangingMeasurementData->RangeStatus = 0
     */

    RangeStatus = pRangingMeasurementData->RangeStatus;

    VL53L0X_GetRangeStatusString(RangeStatus, buf);
    ROS_INFO("Range Status: %i : %s\n", RangeStatus, buf);

}


VL53L0X_Error rangingTest(VL53L0X_Dev_t *pMyDevice, ros::NodeHandle nh)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    VL53L0X_RangingMeasurementData_t    RangingMeasurementData;
    int i;
    uint32_t refSpadCount;
    uint8_t isApertureSpads;
    uint8_t VhvSettings;
    uint8_t PhaseCal;

	ros::Publisher rang_pub = nh.advertise<std_msgs::Int16>("mavros/laser_ranging", 1);
		
	std_msgs::Int16 dist;

    if(Status == VL53L0X_ERROR_NONE)
    {
        ROS_INFO ("Call of VL53L0X_StaticInit\n");
        Status = VL53L0X_StaticInit(pMyDevice); // Device Initialization
        print_pal_error(Status);
    }
    
    if(Status == VL53L0X_ERROR_NONE)
    {
        ROS_INFO ("Call of VL53L0X_PerformRefCalibration\n");
        Status = VL53L0X_PerformRefCalibration(pMyDevice,
        		&VhvSettings, &PhaseCal); // Device Initialization
        print_pal_error(Status);
    }

    if(Status == VL53L0X_ERROR_NONE)
    {
        ROS_INFO ("Call of VL53L0X_PerformRefSpadManagement\n");
        Status = VL53L0X_PerformRefSpadManagement(pMyDevice,
        		&refSpadCount, &isApertureSpads); // Device Initialization
        ROS_INFO ("refSpadCount = %d, isApertureSpads = %d\n", refSpadCount, isApertureSpads);
        print_pal_error(Status);
    }

    if(Status == VL53L0X_ERROR_NONE)
    {

        // no need to do this when we use VL53L0X_PerformSingleRangingMeasurement
        ROS_INFO ("Call of VL53L0X_SetDeviceMode\n");
        Status = VL53L0X_SetDeviceMode(pMyDevice, VL53L0X_DEVICEMODE_SINGLE_RANGING); // Setup in single ranging mode
        print_pal_error(Status);
    }

    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckEnable(pMyDevice,
        		VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
    }
    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckEnable(pMyDevice,
        		VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
    }
				
    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckValue(pMyDevice,
        		VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
        		(FixPoint1616_t)(0.1*65536));
	}			
    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckValue(pMyDevice,
        		VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
        		(FixPoint1616_t)(60*65536));			
    }
    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(pMyDevice,
        		33000);
	}
	
    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetVcselPulsePeriod(pMyDevice, 
		        VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
    }
    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetVcselPulsePeriod(pMyDevice, 
		        VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);
    }


    /*
     *  Step  4 : Test ranging mode
     */

	ros::Rate loop_rate(15);

	ROS_WARN ("\n\n***************Start ranging***************\n\n");

    if(Status == VL53L0X_ERROR_NONE)
    {
        while (ros::ok()){
            //ROS_INFO ("Call of VL53L0X_PerformSingleRangingMeasurement\n");
            Status = VL53L0X_PerformSingleRangingMeasurement(pMyDevice,
            		&RangingMeasurementData);

            //print_pal_error(Status);
            //print_range_status(&RangingMeasurementData);

           ROS_INFO ("distance = %d cm\n",RangingMeasurementData.RangeMilliMeter/10);
            //if (Status != VL53L0X_ERROR_NONE) break;

            //ROS_INFO("Measured distance: %i\n\n", RangingMeasurementData.RangeMilliMeter);
			dist.data = RangingMeasurementData.RangeMilliMeter;
			dist.data = (dist.data > 2500) ? 2500 : dist.data;
			
			rang_pub.publish(dist);
			loop_rate.sleep();
		}
    }
    return Status;
}

int main(int argc, char **argv)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    VL53L0X_Dev_t MyDevice;
    VL53L0X_Dev_t *pMyDevice = &MyDevice;
    VL53L0X_Version_t                   Version;
    VL53L0X_Version_t                  *pVersion   = &Version;
    VL53L0X_DeviceInfo_t                DeviceInfo;

    int32_t status_int;
	
	ros::init(argc, argv, "laser_ros");

	ros::NodeHandle nh;
	//ros::Publisher rang_pub = nh.advertise<std_msgs::Int16>("mavros/laser_ranging", 1);
	
	uint32_t addr = DEV2_I2C_ADDR; //(argv[1][2] - '0')*16 + (argv[1][3] - '0');

    // Initialize Comms
    pMyDevice->I2cDevAddr      = addr;

    pMyDevice->fd = VL53L0X_i2c_init("/dev/i2c-1", pMyDevice->I2cDevAddr);
    
	if (MyDevice.fd<0) {
        Status = VL53L0X_ERROR_CONTROL_INTERFACE;
        ROS_ERROR ("Failed to init\n");
    }
	
    if(Status == VL53L0X_ERROR_NONE)
    {
        ROS_INFO ("Call of VL53L0X_DataInit\n");
        Status = VL53L0X_DataInit(&MyDevice); // Data initialization
        print_pal_error(Status);
    }

    if(Status == VL53L0X_ERROR_NONE)
    {
        Status = VL53L0X_GetDeviceInfo(&MyDevice, &DeviceInfo);
        if(Status == VL53L0X_ERROR_NONE)
        {
            ROS_INFO("VL53L0X_GetDeviceInfo:\n");
            ROS_INFO("Device Name : %s\n", DeviceInfo.Name);
            ROS_INFO("Device Type : %s\n", DeviceInfo.Type);
            ROS_INFO("Device ID : %s\n", DeviceInfo.ProductId);
            ROS_INFO("ProductRevisionMajor : %d\n", DeviceInfo.ProductRevisionMajor);
            ROS_INFO("ProductRevisionMinor : %d\n", DeviceInfo.ProductRevisionMinor);

        if ((DeviceInfo.ProductRevisionMinor != 1) && (DeviceInfo.ProductRevisionMinor != 1)) {
        	ROS_ERROR("Error expected cut 1.1 but found cut %d.%d\n",
                       DeviceInfo.ProductRevisionMajor, DeviceInfo.ProductRevisionMinor);
                Status = VL53L0X_ERROR_NOT_SUPPORTED;
            }
        }
        print_pal_error(Status);
    }

    print_pal_error(Status);

    if(Status == VL53L0X_ERROR_NONE)
    {
        Status = rangingTest(pMyDevice, nh);
    }

    print_pal_error(Status);
    
    // Implementation specific

    /*
     *  Disconnect comms - part of VL53L0X_platform.c
     */

    ROS_WARN ("Close Comms\n");
    VL53L0X_i2c_close();

    print_pal_error(Status);
	
    return (0);
}

