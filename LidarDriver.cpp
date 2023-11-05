#include <sys/unistd.h>
#include "LidarDriver.h"

LidarDriver::LidarDriver()
  : _serial(NULL){};

LidarDriver::~LidarDriver(){
    end();
}

void LidarDriver::end(){
  if (isOpen()) {
     _serial->end();
     _serial = NULL;
  }
}

bool LidarDriver::begin(HardwareSerial &serialobj){
    if (isOpen()) {
      end(); 
    }
    _serial = &serialobj;
    _serial->end();
    _serial->begin(LIDAR_BAUDRATE);
    return 1;
};

bool LidarDriver::isOpen(){
  return _serial?true:false;
}

void LidarDriver::clearSerialBuffer(){
  while(_serial->available()){
    _serial->read();
  }
}

LidarFrame LidarDriver::readFrame(uint16_t timeout){
  LidarFrame frame;
  uint8_t status = 0;
  uint8_t paramCtr = 0;
  uint16_t checksum = 0;
  
  uint8_t by;

  clearSerialBuffer();

  uint32_t startMillis = millis();
  while(millis() - startMillis < READ_FRAME_TIMEOUT){
    if(_serial->available() > 0){
      by = _serial->read();
      switch(status){
        case 0:
          checksum = 0;
          paramCtr = 0;
          frame.header = by;
          if(frame.header == FRAME_HEADER){status = 1;}
          else{status = 0;}
          break;
        case 1:
          frame.length = by << 8;
          status = 2;
          break;
        case 2:
          frame.length += by;
          status = 3;
          break;
        case 3:
          frame.protocolVersion = by;
          if(frame.protocolVersion == PROTOCOL_VERSION){status = 4;}
          else{status = 0;}
          break;
        case 4:
          frame.type = by;
          if(frame.type == FRAME_TYPE){status = 5;}
          else{status = 0;}
          break;
        case 5:
          frame.commandWord = by;
          status = 6;
          break;
        case 6:
          frame.parameterLength = by << 8;
          status = 7;
          break;
        case 7:
          frame.parameterLength += by;
          if(frame.parameterLength > 0 && frame.parameterLength <= MAX_PARAM_LENGTH){status = 8;}
          else{status = 0;}
          break;
        case 8:
          frame.parameters[paramCtr] = by;
          paramCtr += 1;
          if(paramCtr == frame.parameterLength){status = 9;} 
          break;
        case 9:
          frame.checksum = by << 8;
          status = 10;
          break;
        case 10:
          frame.checksum += by;
          if(checksum == frame.checksum){return frame;}
          status = 0;
          break;
        }
      if(status < 10){checksum = (checksum + by) % 0xFFFF;}
    }
  }
  return frame;
}

LidarFrame* LidarDriver::readContiguousFrames(uint16_t n){
 LidarFrame* frames = new LidarFrame[n];

 if (!frames) {
   Serial.println("Memory allocation failed.");
   return nullptr;
 }

 for(int i = 0; i < n; i++){
   frames[i] = readFrame();
 }

 return frames;
}

bool LidarDriver::scan(){
  uint16_t pointCtr = 0;

  LidarFrame* frames = readContiguousFrames();

  for(int i = 0; i < SCAN_STEPS; i++){

    scanData.numFrames = SCAN_STEPS;    
    
    if(frames[i].commandWord == 0xAD){
      float rpm = frames[i].parameters[0]*RPM_FACTOR;
      float startAngle = ((frames[i].parameters[3]<<8)+frames[i].parameters[4])*ANGLE_FACTOR;
      int sampleCount = (frames[i].parameterLength-5)/3;
      int frameIndex = (startAngle/(360.0/SCAN_STEPS));

      for(int j = 0; j < sampleCount; j++){
        scanData.points[pointCtr].distance = ((frames[i].parameters[5 +(j*3)+1]<<8) + frames[i].parameters[5+(j*3)+2])*RANGE_SCALE;;
        scanData.points[pointCtr].angle = -1 * (((360.0/SCAN_STEPS)/sampleCount)*j + startAngle);
        scanData.points[pointCtr].quality = frames[i].parameters[5 + (j * 3)];
        pointCtr+=1;
      }
    }
    else{scanData.numPoints = 0; delete frames; return 0;}
  }
  scanData.numPoints = pointCtr;
  delete frames;
  return 1;
}
