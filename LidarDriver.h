#pragma once

#include "Arduino.h"
#include "lidarTypes.h"

class LidarDriver
{
public:
  //class constructor
  LidarDriver();
  //class destructor
  ~LidarDriver();
  //begins serial communication with lidar
  bool begin(HardwareSerial &serialobj);
  //ends serial communication with lidar
  void end();
  //returns 1 if serial communication established, otherwise 0
  bool isOpen();
  // initiates the reading and deserialization of 1 full lidar rotation scan, returns true if successful, 0 if error occured
  bool scan(); 
  //stores scan data prosecced during scan()
  LidarScan scanData;

  ErrorCode error;

private:
  //clears outdated buffer cache
  void clearSerialBuffer();
  //reads and processes 1 serial commuication frame, returns processed frame
  LidarFrame readFrame(uint16_t timeout = READ_FRAME_TIMEOUT);
  //reads and processes n serial communication frames, returns pointer to array of processed frames
  LidarFrame* readContiguousFrames(uint16_t n = SCAN_STEPS);
  //pointer to hardware serial instance used to communicate with lidar
  HardwareSerial * _serial;
};
