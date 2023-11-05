#pragma once

#include <stdint.h>

#define MAX_PARAM_LENGTH        100

#define LIDAR_ANGULAR_RESOLUTION  0.72
#define MAX_POINTS_PER_SCAN     int(360/0.72)

#define SCAN_STEPS          16
#define ANGLE_FACTOR        0.01
#define RPM_FACTOR          6
#define RANGE_SCALE         0.00025 //to meters

typedef struct{
  float distance;
  float angle;
  bool quality;  
} LidarPoint;

typedef struct{
  uint16_t numPoints;
  LidarPoint points[MAX_POINTS_PER_SCAN];
  uint16_t numFrames;
} LidarScan;


typedef struct{
  float rpm;
  float startAngle;
  uint16_t sampleCount;
  uint8_t frameIndex;
} FrameData;

enum class ErrorCode{
  Success,
  HeaderFailed,
  ProtocolVersionFailed,
  FrameTypeFailed,
  ParamLengthFailed,
  ChecksumFailed,
  TimeoutOccured,
  MemoryAllocationFailed
};

typedef struct{
      uint8_t header;
      uint16_t length;
      uint8_t protocolVersion;
      uint8_t type;
      uint8_t commandWord;
      uint16_t parameterLength;
      uint8_t parameters[MAX_PARAM_LENGTH];
      uint16_t checksum;
} LidarFrame;

#define READ_FRAME_TIMEOUT        200
#define LIDAR_BAUDRATE            115200

#define FRAME_HEADER              0xAA
#define FRAME_TYPE                0x61
#define PROTOCOL_VERSION          0x01
