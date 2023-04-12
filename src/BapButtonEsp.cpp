#include <Arduino.h>
#include <NTPClient.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <PubSubClient.h>
#include <Bounce2.h>
#include <MsgPack.h>
#include "DebugUtils.h"
#include "LittleFS.h"
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>
#include "WifiTerm/WiFiTerm.h"
#include <Battery.h>
#include <NeoPixelBrightnessBus.h> // instead of NeoPixelBus.h
#include <RTCVars.h>
#include <EEPROM.h>

Battery battery(2600, 6600, A0);

#define DEBUG 1
#define NUMROWS (8)
#define NUMCOLS (8)
#define NUMPIXELS (NUMROWS * NUMCOLS)
String versionId = "1.0.4";
int buttonSettingsAddress = 0;

const boolean showCustomImageDebug = false;
const RgbColor black(0, 0, 0);
RTCVars state;
int shutdownInProgress;
boolean testingMode = false;
short currentTestPattern = 0;

const uint32 bapLogo[64] = {
    0,
    0,
    0,
    16771899,
    16771899,
    16761095,
    0,
    0,
    51283,
    0,
    0,
    16771899,
    0,
    16761095,
    0,
    0,
    51283,
    0,
    0,
    16771899,
    16761095,
    16761095,
    0,
    0,
    51283,
    0,
    0,
    16771899,
    0,
    16761095,
    240116,
    240116,
    51283,
    51283,
    51283,
    16771899,
    0,
    16761095,
    0,
    2712319,
    51283,
    0,
    0,
    51283,
    0,
    2201331,
    2201331,
    2712319,
    51283,
    0,
    0,
    51283,
    0,
    2201331,
    0,
    0,
    58998,
    58998,
    58998,
    58998,
    0,
    2201331,
    0,
    0};

const byte builtinPatterns[255][8] = {
    // Zero means no pattern
    {B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000},
    // 1
    {B00000000, B00011000, B00011000, B00111000, B00011000, B00011000, B00011000, B01111110},
    // 2
    {B00000000, B00111100, B01100110, B00000110, B00001100, B00110000, B01100000, B01111110},
    // 3
    {B00000000, B00111100, B01100110, B00000110, B00011100, B00000110, B01100110, B00111100},
    // 4
    {B00000000, B00001100, B00011100, B00101100, B01001100, B01111110, B00001100, B00001100},
    // 5
    {B00000000, B01111110, B01100000, B01111100, B00000110, B00000110, B01100110, B00111100},
    // 6
    {B00000000, B00111100, B01100110, B01100000, B01111100, B01100110, B01100110, B00111100},
    // 7
    {B00000000, B01111110, B01100110, B00001100, B00001100, B00011000, B00011000, B00011000},
    // 8
    {B00000000, B00111100, B01100110, B01100110, B00111100, B01100110, B01100110, B00111100},
    // 9
    {B00000000, B00111100, B01100110, B01100110, B00111110, B00000110, B01100110, B00111100},
    // 10
    {B00000000, B01001111, B01001001, B01001001, B01001001, B01001001, B01001001, B01001111},
    // 11
    {B00000000, B01001000, B01001000, B01001000, B01001000, B01001000, B01001000, B01001000},
    // 12
    {B00000000, B01001111, B01000001, B01000001, B01001111, B01001000, B01001000, B01001111},
    // 13
    {B00000000, B01001111, B01000001, B01000001, B01001111, B01000001, B01000001, B01001111},
    // 14
    {B00000000, B01001001, B01001001, B01001001, B01001111, B01000001, B01000001, B01000001},
    // 15
    {B00000000, B01001111, B01001000, B01001000, B01001111, B01000001, B01000001, B01001111},
    // 16
    {B00000000, B01001000, B01001000, B01001000, B01001111, B01001001, B01001001, B01001111},
    // 17
    {B00000000, B01001111, B01000001, B01000001, B01000001, B01000001, B01000001, B01000001},
    // 18
    {B00000000, B01001111, B01001001, B01001001, B01001111, B01001001, B01001001, B01001111},
    // 19
    {B00000000, B01001111, B01001001, B01001001, B01001111, B01000001, B01000001, B01000001},
    // 20
    {B00000000, B11101111, B00101001, B00101001, B11101001, B10001001, B10001001, B11101111},
    // 21
    {B00000000, B11100100, B00100100, B00100100, B11100100, B10000100, B10000100, B11100100},
    // 22
    {B00000000, B11101111, B00100001, B00100001, B11101111, B10001000, B10001000, B11101111},
    // 23
    {B00000000, B11101111, B00100001, B00100001, B11101111, B10000001, B10000001, B11101111},
    // 24
    {B00000000, B11101001, B00101001, B00101001, B11101111, B10000001, B10000001, B11100001},
    // 25
    {B00000000, B11101111, B00101000, B00101000, B11101111, B10000001, B10000001, B11101111},
    // 26
    {B00000000, B11101000, B00101000, B00101000, B11101111, B10001001, B10001001, B11101111},
    // 27
    {B00000000, B11101111, B00100001, B00100001, B11100001, B10000001, B10000001, B11100001},
    // 28
    {B00000000, B11101111, B00101001, B00101001, B11101111, B10001001, B10001001, B11101111},
    // 29
    {B00000000, B11101111, B00101001, B00101001, B11101111, B10000001, B10000001, B11100001},
    // 30
    {B00000000, B11101111, B00101001, B00101001, B11101001, B00101001, B00101001, B11101111},
    // 31
    {B00000000, B11100100, B00100100, B00100100, B11100100, B00100100, B00100100, B11100100},
    // 32
    {B00000000, B11101111, B00100001, B00100001, B11100111, B00101000, B00101000, B11101111},
    // 33
    {B00000000, B11101110, B00100010, B00100010, B11101110, B00100010, B00100010, B11101110},
    // 34
    {B00000000, B11101001, B00101001, B00101001, B11101111, B00100001, B00100001, B11100001},
    // 35
    {B00000000, B11101111, B00101000, B00101000, B11101111, B00100001, B00100001, B11101111},
    // 36
    {B00000000, B11101000, B00101000, B00101000, B11101111, B00101001, B00101001, B11101111},
    // 37
    {B00000000, B11101111, B00100001, B00100001, B11100001, B00100001, B00100001, B11100001},
    // 38
    {B00000000, B11101111, B00101001, B00101001, B11101111, B00101001, B00101001, B11101111},
    // 39
    {B00000000, B11101111, B00101001, B00101001, B11101111, B00100001, B00100001, B11100001},
    // 40
    {B00000000, B10101111, B10101001, B10101001, B11101001, B00101001, B00101001, B00101111},
    // 41
    {B00000000, B10100100, B10100100, B10100100, B11100100, B00100100, B00100100, B00100100},
    // 42
    {B00000000, B10101111, B10100001, B10100001, B11101111, B00101000, B00101000, B00101111},
    // 43
    {B00000000, B10101111, B10100001, B10100001, B11101111, B00100001, B00100001, B00101111},
    // 44
    {B00000000, B10101001, B10101001, B10101001, B11101111, B00100001, B00100001, B00100001},
    // 45
    {B00000000, B10101111, B10101000, B10101000, B11101111, B00100001, B00100001, B00101111},
    // 46
    {B00000000, B10101000, B10101000, B10101000, B11101111, B00101001, B00101001, B00101111},
    // 47
    {B00000000, B10101111, B10100001, B10100001, B11100001, B00100001, B00100001, B00100001},
    // 48
    {B00000000, B10101111, B10101001, B10101001, B11101111, B00101001, B00101001, B00101111},
    // 49
    {B00000000, B10101111, B10101001, B10101001, B11101111, B00100001, B00100001, B00100001},
    // 50
    {B00000000, B11101111, B10001001, B10001001, B11101001, B00101001, B00101001, B11101111},
    // 51
    {B00000000, B11100100, B10000100, B10000100, B11100100, B00100100, B00100100, B11100100},
    // 52
    {B00000000, B11101111, B10000001, B10000001, B11101111, B00101000, B00101000, B11101111},
    // 53
    {B00000000, B11101111, B10000001, B10000001, B11101111, B00100001, B00100001, B11101111},
    // 54
    {B00000000, B11101001, B10001001, B10001001, B11101111, B00100001, B00100001, B11100001},
    // 55
    {B00000000, B11101111, B10001000, B10001000, B11101111, B00100001, B00100001, B11101111},
    // 56
    {B00000000, B11101000, B10001000, B10001000, B11101111, B00101001, B00101001, B11101111},
    // 57
    {B00000000, B11101111, B10000001, B10000001, B11100001, B00100001, B00100001, B11100001},
    // 58
    {B00000000, B11101111, B10001001, B10001001, B11101111, B00101001, B00101001, B11101111},
    // 59
    {B00000000, B11101111, B10001001, B10001001, B11101111, B00100001, B00100001, B11100001},
    // 60
    {B00000000, B10001111, B10001001, B10001001, B11101001, B10101001, B10101001, B11101111},
    // 61
    {B00000000, B10000100, B10000100, B10000100, B11100100, B10100100, B10100100, B11100100},
    // 62
    {B00000000, B10001111, B10000001, B10000001, B11101111, B10101000, B10101000, B11101111},
    // 63
    {B00000000, B10001111, B10000001, B10000001, B11101111, B10100001, B10100001, B11101111},
    // 64
    {B00000000, B10001001, B10001001, B10001001, B11101111, B10100001, B10100001, B11100001},
    // 65
    {B00000000, B10001111, B10001000, B10001000, B11101111, B10100001, B10100001, B11101111},
    // 66
    {B00000000, B10001000, B10001000, B10001000, B11101111, B10101001, B10101001, B11101111},
    // 67
    {B00000000, B10001111, B10000001, B10000001, B11100001, B10100001, B10100001, B11100001},
    // 68
    {B00000000, B10001111, B10001001, B10001001, B11101111, B10101001, B10101001, B11101111},
    // 69
    {B00000000, B10001111, B10001001, B10001001, B11101111, B10100001, B10100001, B11100001},
    // 70
    {B00000000, B11101111, B00101001, B00101001, B00101001, B00101001, B00101001, B00101111},
    // 71
    {B00000000, B11100100, B00100100, B00100100, B00100100, B00100100, B00100100, B00100100},
    // 72
    {B00000000, B11101111, B00100001, B00100001, B00101111, B00101000, B00101000, B00101111},
    // 73
    {B00000000, B11101111, B00100001, B00100001, B00101111, B00100001, B00100001, B00101111},
    // 74
    {B00000000, B11101001, B00101001, B00101001, B00101111, B00100001, B00100001, B00100001},
    // 75
    {B00000000, B11101111, B00101000, B00101000, B00101111, B00100001, B00100001, B00101111},
    // 76
    {B00000000, B11101000, B00101000, B00101000, B00101111, B00101001, B00101001, B00101111},
    // 77
    {B00000000, B11101111, B00100001, B00100001, B00100001, B00100001, B00100001, B00100001},
    // 78
    {B00000000, B11101111, B00101001, B00101001, B00101111, B00101001, B00101001, B00101111},
    // 79
    {B00000000, B11101111, B00101001, B00101001, B00101111, B00100001, B00100001, B00100001},
    // 80
    {B00000000, B11101111, B10101001, B10101001, B11101001, B10101001, B10101001, B11101111},
    // 81
    {B00000000, B11100100, B10100100, B10100100, B11100100, B10100100, B10100100, B11100100},
    // 82
    {B00000000, B11101111, B10100001, B10100001, B11101111, B10101000, B10101000, B11101111},
    // 83
    {B00000000, B11101111, B10100001, B10100001, B11101111, B10100001, B10100001, B11101111},
    // 84
    {B00000000, B11101001, B10101001, B10101001, B11101111, B10100001, B10100001, B11100001},
    // 85
    {B00000000, B11101111, B10101000, B10101000, B11101111, B10100001, B10100001, B11101111},
    // 86
    {B00000000, B11101000, B10101000, B10101000, B11101111, B10101001, B10101001, B11101111},
    // 87
    {B00000000, B11101111, B10100001, B10100001, B11100001, B10100001, B10100001, B11100001},
    // 88
    {B00000000, B11101111, B10101001, B10101001, B11101111, B10101001, B10101001, B11101111},
    // 89
    {B00000000, B11101111, B10101001, B10101001, B11101111, B10100001, B10100001, B11100001},
    // 90
    {B00000000, B11101111, B10101001, B10101001, B11101001, B00101001, B00101001, B00101111},
    // 91
    {B00000000, B11100100, B10100100, B10100100, B11100100, B00100100, B00100100, B00100100},
    // 92
    {B00000000, B11101111, B10100001, B10100001, B11101111, B00101000, B00101000, B00101111},
    // 93
    {B00000000, B11101111, B10100001, B10100001, B11101111, B00100001, B00100001, B00101111},
    // 94
    {B00000000, B11101001, B10101001, B10101001, B11101111, B00100001, B00100001, B00100001},
    // 95
    {B00000000, B11101111, B10101000, B10101000, B11101111, B00100001, B00100001, B00101111},
    // 96
    {B00000000, B11101000, B10101000, B10101000, B11101111, B00101001, B00101001, B00101111},
    // 97
    {B00000000, B11101111, B10100001, B10100001, B11100001, B00100001, B00100001, B00100001},
    // 98
    {B00000000, B11101111, B10101001, B10101001, B11101111, B00101001, B00101001, B00101111},
    // 99
    {B00000000, B11101111, B10101001, B10101001, B11101111, B00100001, B00100001, B00100001},
    // 0 - 100
    {B00000000, B00111100, B01100110, B01100110, B01100110, B01100110, B01100110, B00111100},
    // A - 101
    {B00000000, B00111100, B01100110, B01100110, B01111110, B01100110, B01100110, B01100110},
    // B - 102
    {B00000000, B01111100, B01100110, B01100110, B01111100, B01100110, B01100110, B01111100},
    // C - 103
    {B00000000, B00111100, B01100110, B01100000, B01100000, B01100000, B01100110, B00111100},
    // D - 104
    {B00000000, B01111100, B01100110, B01100110, B01100110, B01100110, B01100110, B01111100},
    // E - 105
    {B00000000, B01111110, B01100000, B01100000, B01111100, B01100000, B01100000, B01111110},
    // F - 106
    {B00000000, B01111110, B01100000, B01100000, B01111100, B01100000, B01100000, B01100000},
    // G - 107
    {B00000000, B00111100, B01100110, B01100000, B01100000, B01101110, B01100110, B00111100},
    // H - 108
    {B00000000, B01100110, B01100110, B01100110, B01111110, B01100110, B01100110, B01100110},
    // I - 109
    {B00000000, B00111100, B00011000, B00011000, B00011000, B00011000, B00011000, B00111100},
    // J - 110
    {B00000000, B00011110, B00001100, B00001100, B00001100, B01101100, B01101100, B00111000},
    // K - 111
    {B00000000, B01100110, B01101100, B01111000, B01110000, B01111000, B01101100, B01100110},
    // L - 112
    {B00000000, B01100000, B01100000, B01100000, B01100000, B01100000, B01100000, B01111110},
    // M - 113
    {B00000000, B01100011, B01110111, B01111111, B01101011, B01100011, B01100011, B01100011},
    // N - 114
    {B00000000, B01100011, B01110011, B01111011, B01101111, B01100111, B01100011, B01100011},
    // O - 115
    {B00000000, B00111100, B01100110, B01100110, B01100110, B01100110, B01100110, B00111100},
    // P - 116
    {B00000000, B01111100, B01100110, B01100110, B01100110, B01111100, B01100000, B01100000},
    // Q - 117
    {B00000000, B00111100, B01100110, B01100110, B01100110, B01101110, B00111100, B00000110},
    // R - 118
    {B00000000, B01111100, B01100110, B01100110, B01111100, B01111000, B01101100, B01100110},
    // S - 119
    {B00000000, B00111100, B01100110, B01100000, B00111100, B00000110, B01100110, B00111100},
    // T - 120
    {B00000000, B01111110, B00011000, B00011000, B00011000, B00011000, B00011000, B00011000},
    // U - 121
    {B00000000, B01100110, B01100110, B01100110, B01100110, B01100110, B01100110, B00111110},
    // V - 122
    {B00000000, B01100110, B01100110, B01100110, B01100110, B01100110, B00111100, B00011000},
    // W - 123
    {B00000000, B01100011, B01100011, B01100011, B01101011, B01111111, B01110111, B01100011},
    // X - 124
    {B00000000, B01100011, B01100011, B00110110, B00011100, B00110110, B01100011, B01100011},
    // Y - 125
    {B00000000, B01100110, B01100110, B01100110, B00111100, B00011000, B00011000, B00011000},
    // x - 126
    {B00000000, B01111110, B00000110, B00001100, B00011000, B00110000, B01100000, B01111110},
    // All One Color - 127
    {B11111111, B11111111, B11111111, B11111111, B11111111, B11111111, B11111111, B11111111},
    // CheckMark - 128
    {B00000011, B00000011, B00000111, B00000110, B11001100, B11111100, B01111000, B00110000},
    // Xout - 129
    {B10000001, B01000010, B00100100, B00011000, B00011000, B00100100, B01000010, B10000001},
    // Border - 130
    {B11111111, B10000001, B10000001, B10000001, B10000001, B10000001, B10000001, B11111111},
    // a -131
    {B00000000, B00000000, B00000000, B00111100, B00000110, B00111110, B01100110, B00111110},
    // b -132
    {B00000000, B01100000, B01100000, B01100000, B01111100, B01100110, B01100110, B01111100},
    // c -133
    {B00000000, B00000000, B00000000, B00111100, B01100110, B01100000, B01100110, B00111100},
    // d -134
    {B00000000, B00000110, B00000110, B00000110, B00111110, B01100110, B01100110, B00111110},
    // e -135
    {B00000000, B00000000, B00000000, B00111100, B01100110, B01111110, B01100000, B00111100},
    // f -136
    {B00000000, B00011100, B00110110, B00110000, B00110000, B01111100, B00110000, B00110000},
    // g -137
    {B00000000, B00000000, B00111110, B01100110, B01100110, B00111110, B00000110, B00111100},
    // h -138
    {B00000000, B01100000, B01100000, B01100000, B01111100, B01100110, B01100110, B01100110},
    // i -139
    {B00000000, B00000000, B00011000, B00000000, B00011000, B00011000, B00011000, B00111100},
    // j -140
    {B00000000, B00001100, B00000000, B00001100, B00001100, B01101100, B01101100, B00111000},
    // k -141
    {B00000000, B01100000, B01100000, B01100110, B01101100, B01111000, B01101100, B01100110},
    // l -142
    {B00000000, B00011000, B00011000, B00011000, B00011000, B00011000, B00011000, B00011000},
    // m -143
    {B00000000, B00000000, B00000000, B01100011, B01110111, B01111111, B01101011, B01101011},
    // n -144
    {B00000000, B00000000, B00000000, B01111100, B01111110, B01100110, B01100110, B01100110},
    // o -145
    {B00000000, B00000000, B00000000, B00111100, B01100110, B01100110, B01100110, B00111100},
    // p -146
    {B00000000, B00000000, B01111100, B01100110, B01100110, B01111100, B01100000, B01100000},
    // q -147
    {B00000000, B00000000, B00111100, B01101100, B01101100, B00111100, B00001101, B00001111},
    // r -148
    {B00000000, B00000000, B00000000, B01111100, B01100110, B01100110, B01100000, B01100000},
    // s -149
    {B00000000, B00000000, B00000000, B00111110, B01000000, B00111100, B00000010, B01111100},
    // t -150
    {B00000000, B00000000, B00011000, B00011000, B01111110, B00011000, B00011000, B00011000},
    // u -151
    {B00000000, B00000000, B00000000, B01100110, B01100110, B01100110, B01100110, B00111110},
    // v -152
    {B00000000, B00000000, B00000000, B00000000, B01100110, B01100110, B00111100, B00011000},
    // w -153
    {B00000000, B00000000, B00000000, B01100011, B01101011, B01101011, B01101011, B00111110},
    // x -155
    {B00000000, B00000000, B00000000, B01100110, B00111100, B00011000, B00111100, B01100110},
    // y -155
    {B00000000, B00000000, B00000000, B01100110, B01100110, B00111110, B00000110, B00111100},
    // z -156
    {B00000000, B00000000, B00000000, B00111100, B00001100, B00011000, B00110000, B00111100},
    // WifiHigh - 157
    {B00000000, B00000011, B00000011, B00011011, B00011011, B11011011, B11011011, B11011011},
    // WifiMedium- 158
    {B00000000, B00000000, B00000000, B00011000, B00011000, B11011000, B11011000, B11011000},
    // WifiLow- 159
    {B00000000, B00000000, B00000000, B00000000, B00000000, B11000000, B11000000, B11000000},
    // Down Arrow - 160
    {B00111000, B00111000, B00111000, B00111000, B11111110, B01111100, B00111000, B00010000},
    // Right Arrow - 161
    {B00001000, B00001100, B11111110, B11111111, B11111110, B00001100, B00001000, B00000000},
    // Up Arrow 162
    {B00010000, B00111000, B01111100, B11111110, B00111000, B00111000, B00111000, B00111000},
    // Left Arrow 163
    {B00010000, B00110000, B01111111, B11111111, B01111111, B00110000, B00010000, B00000000},
    // SmileyFace - 164
    {B00111100, B01111110, B10111101, B10111101, B11111111, B10111101, B01000010, B00111100},
    // Plus Sign - 165
    {B00000000, B00110000, B00110000, B11111100, B00110000, B00110000, B00000000, B00000000},
    // Minus Sign - 166
    {B00000000, B00000000, B00000000, B11111100, B00000000, B00000000, B00000000, B00000000},
    // Asterix - 167
    {B00000000, B01100110, B00111100, B11111111, B00111100, B01100110, B00000000, B00000000},
    // Forward Slash - 168
    {B00000110, B00001100, B00011000, B00110000, B01100000, B11000000, B10000000, B00000000},
    // Divide - 169
    {B00000000, B11000110, B11001100, B00011000, B00110000, B01100110, B11000110, B00000000},
    // Equals - 170
    {B00000000, B00000000, B11111100, B00000000, B00000000, B11111100, B00000000, B00000000},
    // Carrot - 171
    {B00010000, B00111000, B01101100, B11000110, B00000000, B00000000, B00000000, B00000000},
    // Left Angle Bracket - 172
    {B00011000, B00110000, B01100000, B11000000, B01100000, B00110000, B00011000, B00000000},
    // Right Angle Bracket - 173
    {B01100000, B00110000, B00011000, B00001100, B00011000, B00110000, B01100000, B00000000},
    // Left parentheses- 174
    {B00011000, B00110000, B01100000, B01100000, B01100000, B00110000, B00011000, B00000000},
    // Right parentheses - 175
    {B01100000, B00110000, B00011000, B00011000, B00011000, B00110000, B01100000, B00000000},
    // Left Square Bracket -176
    {B01111000, B01100000, B01100000, B01100000, B01100000, B01100000, B01111000, B00000000},
    // Right Square Bracket -177
    {B01111000, B00011000, B00011000, B00011000, B00011000, B00011000, B01111000, B00000000},
    // Left Curly Brace -178
    {B00011100, B00110000, B00110000, B11100000, B00110000, B00110000, B00011100, B00000000},
    // Right Curly Brace -179
    {B11100000, B00110000, B00110000, B00011100, B00110000, B00110000, B11100000, B00000000},
    // Period -180
    {B00000000, B00000000, B00000000, B00000000, B00000000, B00110000, B00110000, B00000000},
    // Colon -181
    {B00000000, B00110000, B00110000, B00000000, B00000000, B00110000, B00110000, B00000000},
    // SemiColon -182
    {B00000000, B00110000, B00110000, B00000000, B00000000, B00110000, B00110000, B01100000},
    // Comma -183
    {B00000000, B00000000, B00000000, B00000000, B00000000, B00110000, B00110000, B01100000},
    // Exclamation Point -184
    {B00011000, B00111100, B00111100, B00011000, B00011000, B00000000, B00011000, B00000000},
    // QuestionMark -185
    {B01111000, B11001100, B00001100, B00011000, B00110000, B00000000, B00110000, B00000000},
    // At Sign -186
    {B01111100, B11000110, B11011110, B11011110, B11011110, B11000000, B01111000, B00000000},
    // Ampersand -187
    {B00111000, B01101100, B00111000, B01110110, B11011100, B11001100, B01110110, B00000000},
    // Dollar Sign -188
    {B00110000, B01111100, B11000000, B01111000, B00001100, B11111000, B00110000, B00000000},
    // Pound Sign - 189
    {B01101100, B01101100, B11111110, B01101100, B11111110, B01101100, B01101100, B00000000},
    // BackSlash -190
    {B11000000, B01100000, B00110000, B00011000, B00001100, B00000110, B00000010, B00000000},
    // Left Single Quote mark - 191
    {B00110000, B00110000, B00011000, B00000000, B00000000, B00000000, B00000000, B00000000},
    // Right Single Quote mark - 192
    {B01100000, B01100000, B11000000, B00000000, B00000000, B00000000, B00000000, B00000000},
    // a with aque mark - 193
    {B00001000, B00010000, B00000000, B01111100, B00001100, B01111100, B11001100, B01111110},
    // e with aque mark - 194
    {B00001000, B00010000, B00000000, B01111000, B11001100, B11111100, B11000000, B01111000},
    // i with aque mark - 195
    {B00000010, B00000100, B00110000, B00000000, B00110000, B00110000, B00110000, B00110000},
    // o with aque mark- 196
    {B00000000, B00001000, B00010000, B00000000, B01111000, B11001100, B11001100, B01111000},
    // n with tilde mark- 197
    {B00000000, B01010000, B10101000, B00000000, B11111000, B11001100, B11001100, B11001100},
    // u with aque mark- 198
    {B00000100, B00001000, B00010000, B00000000, B11001100, B11001100, B01001100, B01111110},
    // u with double dots- 199
    {B00000000, B00000000, B11001100, B00000000, B11001100, B11001100, B11111100, B01111110}};

struct ButtonPress
{
  bool startOfPress;
  long lengthOfPressInMillis;
  MSGPACK_DEFINE(startOfPress, lengthOfPressInMillis);
};

struct ButtonStatus
{
  int batteryLevel;
  long wifiStrength;
  String iPAddress;
  String versionId;
  MSGPACK_DEFINE(batteryLevel, wifiStrength, iPAddress, versionId);
};

struct ButtonSettings
{
  bool sendReleaseMessage;
  String alternateWifiSSID;
  String alternateWifiPassword;
  short minBatteryLevel;
  short pressSensitivity;
  MSGPACK_DEFINE(sendReleaseMessage, alternateWifiSSID, alternateWifiPassword, minBatteryLevel, pressSensitivity);
};

struct ButtonImage
{
  MsgPack::arr_t<uint32> imageData;
  MSGPACK_DEFINE(imageData);
};

const char *ssid = "BAPButton";
const char *password = "B@pB@p1234";
const char *basePrivateTopic = "buttons";
String privateTopicCmd = "temp/";
String privateTopicRestart = "temp/";
String privateTopicTesting = "temp/";
String privateTopicAn = "temp/";
String privateTopicStatus = "temp/";
String privateTopicGetStatus = "temp/";
String privateTopicPowerOff = "temp/";
String privateTopicButtonSettings = "temp/";
String publicCmd = "general/command";
String publicRestart = "general/restart";
String publicStatus = "general/status";
String publicPowerOff = "general/poweroff";
String publicButtonSettings = "general/buttonsettings";

WiFiUDP ntpUDP;
ButtonSettings buttonSettings;

String clientName = "";
String clientId = "TB-";
unsigned long currentMillis = 0;
// unsigned long timeOutInterval = 0;
unsigned long powerOffInterval = 600000;
unsigned long commandLastRecievedMillis = 0;
unsigned long buttonFellAtMillis = 0;

unsigned long turnOffDisplayTime = 0;

ButtonStatus buttonStatus;
ButtonPress buttonPress;
ButtonImage buttonImage;

#define BUTTONPIN 5
#define NEOPIXELPIN 4
#define PowerOnPin 14
#define LedOnOffPin 13
WiFiClient espClient;
PubSubClient client(espClient);

#define MSG_BUFFER_SIZE (200)
char msg[MSG_BUFFER_SIZE];

// #define AN_MSG_BUFFER_SIZE (100)
// char anMsg[100];

Bounce debouncer = Bounce();

NeoPixelBrightnessBus<NeoGrbFeature, NeoEsp8266Dma800KbpsMethod> pixelMatrix(NUMPIXELS, 3);

void SetPattern(short patternId, uint8_t red, uint8_t green, uint8_t blue, uint8_t brightness)
{
  pixelMatrix.SetBrightness(brightness);
  term.print(F("Setting the Pattern with id: "));
  term.print(patternId);
  term.print(F(" at millis "));
  term.println(millis());
  term.print(F("Green is "));
  term.print(green);
  term.print(F(" and Red is  "));
  term.print(red);
  term.print(F(" and Blue is  "));
  term.print(blue);
  term.print(F(" and Brightness is  "));
  term.println(brightness);
  int lcdIx = 0; // index in lcd pixels

  for (int row = 0; row < NUMROWS; row++)
  {
    byte currentPatternByte = builtinPatterns[patternId][row];
    for (int col = 0; col < NUMCOLS; col++)
    {
      lcdIx = (row * NUMCOLS) + ((NUMCOLS - 1) - col); // current pixel in LCD. Subtract col by (NUMCOLS-1) since LCD is mirrored horizontally
      byte mask = pow(2, col);                         // 2 to the power of (NUMCOLS-1)-col gives us byte for the bit we're testing (e.g. 2^2 = 00000100)
      if (mask & currentPatternByte)
      {
        RgbColor color = RgbColor(red, green, blue);
        // Turn on with correct color
        pixelMatrix.SetPixelColor(lcdIx, color);
      }
      else
      {
          pixelMatrix.SetPixelColor(lcdIx, RgbColor(0, 0, 0));
      }
    }
  };
  //
  pixelMatrix.Show();
  term.println(F("Showing the pattern"));
}

void PowerOff()
{
  term.print("This is a shutdown Command. Getting Ready to shut down");
  SetPattern(3, 0, 255, 0, 32);
  client.publish(privateTopicStatus.c_str(), "", true);
  client.unsubscribe(privateTopicCmd.c_str());
  // client.unsubscribe(privateTopicCustomImage.c_str());
  client.unsubscribe(privateTopicRestart.c_str());
  client.unsubscribe(privateTopicPowerOff.c_str());
  client.unsubscribe(privateTopicGetStatus.c_str());
  client.unsubscribe(privateTopicTesting.c_str());
  client.unsubscribe(publicCmd.c_str());
  client.unsubscribe(publicStatus.c_str());
  client.unsubscribe(publicRestart.c_str());
  client.unsubscribe(publicPowerOff.c_str());
  delay(250);
  SetPattern(2, 0, 255, 0, 32);
  delay(500);
  SetPattern(1, 0, 255, 0, 32);
  delay(500);
  SetPattern(100, 0, 255, 0, 32);
  term.print("Shutdown Images Down. GoodBye");
  delay(1000);
  shutdownInProgress = 1;
  state.saveToRTC();
  digitalWrite(PowerOnPin, LOW);
  // There seems to be a little power left. Probably from the capacitor that drives the LED. So I am trying both.
  ESP.deepSleep(0);
}

void setup_wifi()
{
  String correctSSID = buttonSettings.alternateWifiSSID.length() > 0 ? buttonSettings.alternateWifiSSID : ssid;
  String correctPassword = buttonSettings.alternateWifiPassword.length() > 0 ? buttonSettings.alternateWifiPassword : password;

  delay(10);
  // We start by connecting to a WiFi network
  term.println();
  term.print("Connecting to ");
  term.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(correctSSID, correctPassword);
  int wifiConnectionAttempts = 0;

  while (WiFi.status() != WL_CONNECTED)
  {
    if (wifiConnectionAttempts > 100)
    {
      PowerOff();
    }
    delay(500);
    term.print(".");
    if (wifiConnectionAttempts == 20 && buttonSettings.alternateWifiSSID.length() > 0)
    {
      // This part is untested.
      correctSSID = ssid;
      correctPassword = password;
      WiFi.disconnect();
      WiFi.begin(correctSSID, correctPassword);
    }
    wifiConnectionAttempts = wifiConnectionAttempts + 1;
  }
  int dBm = WiFi.RSSI();
  if (dBm >= -60)
  {
    SetPattern(157, 0, 255, 0, 32);
  }
  if (dBm <= -90)
  {
    SetPattern(159, 255, 0, 0, 32);
  }
  else
  {
    SetPattern(158, 255, 255, 0, 32);
  }

  // timeClient.begin();
  term.println("");
  term.println("WiFi connected");
  term.println("IP address: ");
  term.println(WiFi.localIP());
  term.println(WiFi.macAddress());
  clientName = WiFi.macAddress().substring(9);
  clientName.remove(2, 1);
  clientName.remove(4, 1);
  clientId += clientName;
  privateTopicCmd = String("buttons/") + clientName + "/command";
  privateTopicAn = String("buttons/") + clientName + "/announce";
  privateTopicStatus = String("buttons/") + clientName + "/status";
  privateTopicGetStatus = String("buttons/") + clientName + "/getstatus";
  privateTopicPowerOff = String("buttons/") + clientName + "/poweroff";
  privateTopicButtonSettings = String("buttons/") + clientName + "/buttonsettings";
  // privateTopicCustomImage = String("buttons/") + clientName + "/customImage";
  privateTopicRestart = String("buttons/") + clientName + "/restart";
  privateTopicTesting = String("buttons/") + clientName + "/testing";
}

void ApplyImageToLEDPanel(uint32 image[NUMPIXELS], uint8_t brightness)
{
  term.print(F("Displaying the Image with brightness"));
  term.println(brightness);
  pixelMatrix.SetBrightness(brightness);
  int lcdIx = 0; // index in lcd pixels

  for (int row = 0; row < NUMROWS; row++)
  {
    for (int col = 0; col < NUMCOLS; col++)
    {
      lcdIx = (row * NUMCOLS) + col;
      uint32 colorValue = image[row * NUMCOLS + col];
      uint8_t red = (uint8_t)(colorValue >> 16);
      uint8_t green = (uint8_t)(colorValue >> 8);
      uint8_t blue = (uint8_t)colorValue;

      RgbColor color = RgbColor(red, green, blue);
      pixelMatrix.SetPixelColor(lcdIx, color);
      // if (showCustomImageDebug)
      // {
      //   Serial.print("Setting pixel ");
      //   Serial.print(lcdIx);
      //   Serial.print(" to Value ");
      //   Serial.println(image[row * NUMCOLS + col]);
      // }
    }
  };

  pixelMatrix.Show();
}

void setTurnOffDisplay(long interval, bool addToCurrentInterval)
{
  term.print("Timeout Interval is ");
  term.println(interval);
  if (addToCurrentInterval)
  {
    turnOffDisplayTime = turnOffDisplayTime + interval;
  }
  else
  {
    if (interval == 0)
    {
      turnOffDisplayTime = 0;
    }
    else
    {
      turnOffDisplayTime = currentMillis + interval;
    }
  }
}

void SendStatus(bool showDisplay)
{
  uint8_t batteryLevel = battery.level();
  term.println("Sending Status");
  term.print("Voltage is ");
  term.println(battery.voltage());
  term.print("Battery Level is ");
  term.println(batteryLevel);
  buttonStatus.batteryLevel = batteryLevel;

  buttonStatus.wifiStrength = WiFi.RSSI();
  buttonStatus.iPAddress = WiFi.localIP().toString();
  buttonStatus.versionId = versionId;

  term.print("A0 is reading ");
  term.println(analogRead(A0));

  if (showDisplay)
  {
    term.println("Showing status info");
    // SetPattern(164, 0, 255, 0, 32, true);
    uint8_t pattern = batteryLevel == 0 ? 100 : batteryLevel;
    SetPattern(pattern, 0, 255, 0, 12);
    pixelMatrix.Show();
    setTurnOffDisplay(3000, false);
  }

  MsgPack::Packer packer;
  packer.serialize(buttonStatus);
  client.publish(privateTopicStatus.c_str(), packer.data(), packer.size(), true);
}

void callback(char *topic, byte *payload, unsigned int length)
{
  term.print("Message arrived [");
  term.print(topic);
  term.println("] ");
  commandLastRecievedMillis = millis();
  turnOffDisplayTime = 0;

  if (String(topic) == privateTopicCmd || String(topic) == publicCmd)
  {
    term.print("Its a button image");
    testingMode = false;
    MsgPack::Unpacker unpacker;
    unpacker.feed(payload, length);
    unpacker.deserialize(buttonImage);
    // SaveCustomImage(customImage);
    uint32_t image[NUMPIXELS];
    for (size_t i = 0; i < buttonImage.imageData.size(); i++)
    {
      image[i] = buttonImage.imageData[i];
    }
    ApplyImageToLEDPanel(image, 16);
  }
  else if (String(topic) == publicStatus || String(topic) == privateTopicGetStatus)
  {
    SendStatus(true);
    testingMode = false;
  }
  else if (String(topic) == privateTopicRestart || String(topic) == publicRestart)
  {
    client.publish(privateTopicStatus.c_str(), "", true);
    ESP.restart();
  }
  else if (String(topic) == privateTopicPowerOff || String(topic) == publicPowerOff)
  {
    PowerOff();
  }
  else if (String(topic) == privateTopicTesting)
  {
    testingMode = true;
    currentTestPattern = 1;
    SetPattern(currentTestPattern, 0, 255, 0, 32);
  }
  else if (String(topic) == privateTopicButtonSettings || String(topic) == publicButtonSettings)
  {
    MsgPack::Unpacker unpacker;
    unpacker.feed(payload, length);
    unpacker.deserialize(buttonSettings);
    EEPROM.put(buttonSettingsAddress, buttonSettings);
  }
}

void subsAndAnnounce()
{
  client.subscribe(privateTopicCmd.c_str());
  client.subscribe(privateTopicRestart.c_str());
  client.subscribe(privateTopicPowerOff.c_str());
  client.subscribe(privateTopicGetStatus.c_str());
  client.subscribe(privateTopicTesting.c_str());
  client.subscribe(publicCmd.c_str());
  client.subscribe(publicStatus.c_str());
  client.subscribe(publicRestart.c_str());
  client.subscribe(publicPowerOff.c_str());
  SendStatus(false);
}

void reconnect()
{
  int mqttConnectionAttempts = 0;
  // Loop until we're reconnected
  while (!client.connected())
  {
    term.println("Attempting MQTT connection...");
    // Attempt to connect
    term.print("Connecting as ");
    term.println(clientId);
    if (client.connect(clientId.c_str(), "", "", privateTopicStatus.c_str(), 0, true, "", true))
    {
      term.println("connected");
      SetPattern(128, 0, 255, 0, 32);
      setTurnOffDisplay(2000, false);
      subsAndAnnounce();
    }
    else
    {
      if (mqttConnectionAttempts > 20)
      {
        PowerOff();
      }
      term.print("failed, rc=");
      term.print(client.state());
      term.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
      mqttConnectionAttempts = mqttConnectionAttempts + 1;
    }
  }
}

ESP8266WebServer server(80);

void setup()
{
  state.registerVar(&shutdownInProgress);
  if (state.loadFromRTC())
  { // we load the values from rtc memory back into the registered variables
    shutdownInProgress++;
    Serial.println("This is shutdown attempt number " + (String)shutdownInProgress);
    state.saveToRTC(); // since we changed a state relevant variable, we store the new values
    digitalWrite(PowerOnPin, LOW);
    ESP.deepSleep(0);
  }
  // Need this to keep the device on after it is started from a button press.
  pinMode(PowerOnPin, OUTPUT);
  digitalWrite(PowerOnPin, HIGH);
  battery.begin(3300, 2.0, &sigmoidal);

  Serial.begin(74880);
  term.link(Serial);
  pixelMatrix.Begin();
  pixelMatrix.Show();
  term.println("Showing the Bap Logo");
  // Show the smiley face
  ApplyImageToLEDPanel(const_cast<uint32 *>(bapLogo), 32);
  pixelMatrix.Show();

  EEPROM.get(buttonSettingsAddress, buttonSettings);

  setup_wifi();
  server.begin();
  term.setAsDefaultWhenUrlNotFound(); // optional : redirect any unknown url to /term.html
  term.activateArduinoFavicon();      // optional : send Arduino icon when a browser asks for favicon
 
  term.begin(server);
  term.println("WiFiTerm started");
  term.print("I'm waiting for you at http://");
  term.print(WiFi.localIP());
  term.println("/term.html");
  term.print("Current Version of Firmware is ");
  term.println(versionId);
  String firmwareUpdateUrl = "http://" + WiFi.gatewayIP().toString() + "/api/firmware";
  term.print("Update url is ");
  term.println(firmwareUpdateUrl);
  t_httpUpdate_return ret;
  ret = ESPhttpUpdate.update(espClient, firmwareUpdateUrl, versionId);
  switch (ret)
  {
  case HTTP_UPDATE_FAILED:
    term.println("[update] Update failed.");
    break;
  case HTTP_UPDATE_NO_UPDATES:
    term.println("[update] No Update.");
    break;
  case HTTP_UPDATE_OK:
    term.println("[update] Update ok."); // may not be called since we reboot the ESP
    break;
  }

  boolean setBuffer = client.setBufferSize(352);
  term.print("Buffer updated ");
  term.println(setBuffer);
  term.print("Server is set to ");
  term.println(WiFi.gatewayIP());
  client.setServer(WiFi.gatewayIP(), 1883);

  client.setCallback(callback);
  pinMode(BUTTONPIN, INPUT_PULLUP);
  debouncer.attach(BUTTONPIN);
  debouncer.interval(70); // interval in ms

  LittleFS.begin();
}

void loop()
{
  server.handleClient();
  term.handleClient();
  // timeClient.update();
  client.loop();
  currentMillis = millis();
  if (turnOffDisplayTime > 0 && currentMillis > turnOffDisplayTime)
  {
    term.print("Light next change Millis has kicked in.");
    term.print("CurrentMillis is ");
    term.println(currentMillis);
    term.print("lightNextChangeMillis is ");
    term.println(turnOffDisplayTime);
    term.println("setting lightNextChangeMillis it to 0");
    turnOffDisplayTime = 0;
    SetPattern(0, 0, 0, 0, 32);
  }
  if ((unsigned long)(currentMillis - commandLastRecievedMillis) >= powerOffInterval)
  {
    term.println("TimeOut. No command recieved. Powering Off");
    PowerOff();
  }
  if (!client.connected())
  {
    reconnect();
  }
  debouncer.update();

  if (debouncer.fell())
  {
    if (testingMode)
    {
      SetPattern(currentTestPattern,0,255,0,32);
      currentTestPattern = currentTestPattern + 1;
      if(currentTestPattern > 100)
      {
        currentTestPattern = 0;
      }
    }
    buttonFellAtMillis = millis();
    buttonPress.startOfPress = true;
    buttonPress.lengthOfPressInMillis = 0;
    term.println(F("Button Pressed"));
    term.println("Sending the message to button pressed topic");
    // memset(anMsg, 0, sizeof(anMsg));
    MsgPack::Packer packer;
    packer.serialize(buttonPress);
    client.publish(privateTopicAn.c_str(), packer.data(), packer.size());
  }
  if (debouncer.rose() && buttonSettings.sendReleaseMessage)
  {
    buttonPress.startOfPress = false;
    buttonPress.lengthOfPressInMillis = millis() - buttonFellAtMillis;
    term.println(F("Button Relased"));
    term.println("Sending the message to button pressed topic");
    // memset(anMsg, 0, sizeof(anMsg));
    MsgPack::Packer packer;
    packer.serialize(buttonPress);
    client.publish(privateTopicAn.c_str(), packer.data(), packer.size());
  }
}
