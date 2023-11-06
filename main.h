#pragma once

#define APP_WINDOWS
//#define APP_LINUX

#define BUFLEN 1248
#define PORT 2368

const int MAX_NUMS = 12;
const int MAX_CHANNEL = 32;

struct Channel {
    float       distance;
    uint8_t     intensity;
    float       azimuth;
    uint8_t     laser_id;
};

struct Datablock {
    uint16_t    flag;
    float       azimuth;
    Channel     channel[MAX_CHANNEL];
};

struct DataFrame {
    Datablock   gelenpaket[MAX_NUMS];
    uint32_t    timestamp;
};

struct GLWidget {
    bool mouseLeftDown;
    bool mouseRightDown;
    bool mouseMiddleDown;
    float mouseX, mouseY;
    float cameraDistanceX;
    float cameraDistanceY;
    float cameraAngleX;
    float cameraAngleY;
};

struct TinyIMU {
    float       pitch = 0.0f;
    float       roll = 0.0f;
    float       yaw = 0.0f;
    float       timestamp = 0.0f;
    int         frequency = 0;
};

struct PointInfo {
    int     point_amount;
    float   max_distance;
    int     frame_rate;
};