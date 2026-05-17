#pragma once

// -----------------------------------------------------------------------------
// Copy this file to `include/credentials.h` and fill in your local credentials.
// The `include/credentials.h` file is ignored by git.
// -----------------------------------------------------------------------------

// WiFi credentials
#define WIFI_SSID     "sdfasfsdfsdfsdf"
#define WIFI_PASSWORD "sdfdsfdafagdgdfg"

// UDP control settings
// - UDP_LISTEN_PORT: local port the robot listens on for control packets
#define UDP_LISTEN_PORT       12345

// Max UDP packet size for control packets
#define UDP_MAX_PACKET_SIZE 64

// ProtocolComm device ID. The handler addresses this robot by this id
// (PROTO_ID_BROADCAST = 0xFF is also always accepted).
#define ID_DEVICE  0x01
