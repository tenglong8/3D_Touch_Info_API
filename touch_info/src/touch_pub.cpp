#include <touch_info/touch_pub.hpp>
#include <stdio.h>
#include <assert.h>
#include <HD/hd.h>
#include <HDU/hduVector.h>
#include <HDU/hduError.h>


typedef struct 
{
    HDboolean m_buttonState;       /* Has the device button has been pressed. */
    HDboolean m_buttonState2;
    hduVector3Dd m_devicePosition; /* Current device coordinates. */
    hduVector3Dd m_deviceOrientation;
    hduVector3Dd m_deviceOrientation2;
    HDErrorInfo m_error;
} DeviceData;

static DeviceData gServoDeviceData;

/*******************************************************************************
 Checks the state of the gimbal button and gets the position of the device.
*******************************************************************************/
HDCallbackCode HDCALLBACK updateDeviceCallback(void *pUserData)
{   
    int nButtons = 0;

    hdBeginFrame(hdGetCurrentDevice());

    /* Retrieve the current button(s). */
    hdGetIntegerv(HD_CURRENT_BUTTONS, &nButtons);
    
    /* In order to get the specific button 1 state, we use a bitmask to
       test for the HD_DEVICE_BUTTON_1 bit. */
    gServoDeviceData.m_buttonState = 
        (nButtons & HD_DEVICE_BUTTON_1) ? HD_TRUE : HD_FALSE;
    
    gServoDeviceData.m_buttonState2 = 
        (nButtons & HD_DEVICE_BUTTON_2) ? HD_TRUE : HD_FALSE;
    /* Get the current location of the device (HD_GET_CURRENT_POSITION)
       We declare a vector of three doubles since hdGetDoublev returns 
       the information in a vector of size 3. */
    hdGetDoublev(HD_CURRENT_POSITION, gServoDeviceData.m_devicePosition);
    hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, gServoDeviceData.m_deviceOrientation);
    hdGetDoublev(HD_CURRENT_JOINT_ANGLES, gServoDeviceData.m_deviceOrientation2);
    /* Also check the error state of HDAPI. */
    gServoDeviceData.m_error = hdGetError();

    /* Copy the position into our device_data tructure. */
    hdEndFrame(hdGetCurrentDevice());

    return HD_CALLBACK_CONTINUE;    
}


/*******************************************************************************
 Checks the state of the gimbal button and gets the position of the device.
*******************************************************************************/
HDCallbackCode HDCALLBACK copyDeviceDataCallback(void *pUserData)
{
    DeviceData *pDeviceData = (DeviceData *) pUserData;

    memcpy(pDeviceData, &gServoDeviceData, sizeof(DeviceData));

    return HD_CALLBACK_DONE;
}


namespace touch{

   publisher::publisher(const std::string& name) :
       Node( name ){
        touch_pub = create_publisher<touch_info::msg::Touch>("Touch_msg", 200);
       
       }

  void publisher::publish( double x, double y, double z, double roll, double pitch, double yaw, bool button1, bool button2 ){
      touch_info::msg::Touch msg;
      msg.twist.linear.x = x;
      msg.twist.linear.y = y;
      msg.twist.linear.z = z;
      msg.twist.angular.x = roll;
      msg.twist.angular.y = pitch;
      msg.twist.angular.z = yaw;
      
      msg.button1 = button1;
      msg.button2 = button2; 
      touch_pub->publish( msg );


}
}

int main( int argc, char** argv ){

  rclcpp::init( argc, argv );
  touch::publisher publisher( "touch_info" );
  rclcpp::Rate rate(200);
  
  HDSchedulerHandle hUpdateHandle = 0;
       HDErrorInfo error;
       
    /* Initialize the device, must be done before attempting to call any hd 
       functions. */
       HHD hHD = hdInitDevice(HD_DEFAULT_DEVICE);
       hUpdateHandle = hdScheduleAsynchronous(
       updateDeviceCallback, 0, HD_MAX_SCHEDULER_PRIORITY);

    /* Start the servo loop scheduler. */
       hdStartScheduler();
          
    /* For cleanup, unschedule callbacks and stop the servo loop. */
       static const int kTerminateCount = 1000;
       int buttonHoldCount = 0;
       
    /* Instantiate the structure used to capture data from the device. */
       DeviceData currentData;
       DeviceData prevData;

    /* Perform a synchronous call to copy the most current device state. */
       hdScheduleSynchronous(copyDeviceDataCallback, 
       &currentData, HD_MIN_SCHEDULER_PRIORITY);
  
  
  
  
  while(rclcpp::ok()){
    hdScheduleSynchronous(copyDeviceDataCallback, &currentData, HD_MIN_SCHEDULER_PRIORITY);
       
    publisher.publish( currentData.m_devicePosition[0], currentData.m_devicePosition[1], currentData.m_devicePosition[2], 
                      currentData.m_deviceOrientation[0]-currentData.m_deviceOrientation2[0], currentData.m_deviceOrientation[1]+currentData.m_deviceOrientation2[2], currentData.m_deviceOrientation[2],
                      currentData.m_buttonState, currentData.m_buttonState2);
    
  
  }

 rclcpp::shutdown();
 
 return 0;
}
