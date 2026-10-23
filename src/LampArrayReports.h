// Copyright (c) Microsoft Corporation.  All rights reserved.


#pragma once

// Report IDs and C structures for the HID LampArray feature reports declared
// in LampArrayHidDescriptor.h. The structs are packed so their memory layout
// matches each report's payload byte for byte (the report ID itself is handled
// by TinyUSB and is not part of the struct). As defined by the HID Lighting and
// Illumination usage page, lengths are in micrometers and times are in
// microseconds.

// Report IDs (must match the REPORT_ID items in LampArrayHidDescriptor.h).
#define LAMP_ARRAY_ATTRIBUTES_REPORT_ID        1   // GET: LampArrayAttributesReport
#define LAMP_ATTRIBUTES_REQUEST_REPORT_ID      2   // SET: LampAttributesRequestReport
#define LAMP_ATTRIBUTES_RESPONSE_REPORT_ID     3   // GET: LampAttributesResponseReport
#define LAMP_MULTI_UPDATE_REPORT_ID            4   // SET: LampMultiUpdateReport
#define LAMP_RANGE_UPDATE_REPORT_ID            5   // SET: LampRangeUpdateReport
#define LAMP_ARRAY_CONTROL_REPORT_ID           6   // SET: LampArrayControlReport

// LampUpdateFlags bit the host sets on the last report of a batch of color
// updates; the new colors should only be displayed once it arrives.
#define LAMP_UPDATE_FLAG_UPDATE_COMPLETE       1

// Number of lamp slots in a LampMultiUpdateReport.
#define LAMP_MULTI_UPDATE_LAMP_COUNT           8

// Color of one lamp: 8-bit red, green and blue levels plus an 8-bit gain
// (intensity) that scales the overall brightness.
typedef struct __attribute__ ((__packed__)) LampArrayColor_
{
    uint8_t RedChannel;
    uint8_t GreenChannel;
    uint8_t BlueChannel;
    uint8_t GainChannel;
} LampArrayColor;

// Report 1 (GET): attributes of the whole lamp array.
typedef struct __attribute__ ((__packed__)) LampArrayAttributesReport_
{
    uint16_t LampCount;                         // number of lamps in the array
    uint32_t BoundingBoxWidth;                  // micrometers
    uint32_t BoundingBoxHeight;                 // micrometers
    uint32_t BoundingBoxDepth;                  // micrometers
    uint32_t LampArrayKind;                     // type of device, e.g. 1 = keyboard
    uint32_t MinUpdateIntervalInMicroseconds;   // shortest allowed time between updates
} LampArrayAttributesReport;

// Report 2 (SET): selects the lamp described by the next
// LampAttributesResponseReport.
typedef struct __attribute__ ((__packed__)) LampAttributesRequestReport_
{
    uint16_t LampId;
} LampAttributesRequestReport;

// Report 3 (GET): attributes of a single lamp.
typedef struct __attribute__ ((__packed__)) LampAttributesResponseReport_
{
    uint16_t LampId;
    uint32_t PositionX;          // micrometers from the array's origin
    uint32_t PositionY;          // micrometers from the array's origin
    uint32_t PositionZ;          // micrometers from the array's origin
    uint32_t UpdateLatency;      // microseconds
    uint32_t LampPurpose;        // LampPurposes bit flags, e.g. 2 = accent
    uint8_t  RedChannelsCount;   // number of red levels
    uint8_t  GreenChannelsCount; // number of green levels
    uint8_t  BlueChannelsCount;  // number of blue levels
    uint8_t  GainChannelsCount;  // number of gain (intensity) levels
    uint8_t  IsProgrammable;     // 1 if the host can set this lamp's color
    uint8_t  LampKey;            // keyboard HID usage the lamp is bound to (InputBinding), 0 = none
} LampAttributesResponseReport;

// Report 4 (SET): sets the colors of up to LAMP_MULTI_UPDATE_LAMP_COUNT lamps;
// lamp LampIds[i] gets UpdateColors[i] for the first LampCount entries.
typedef struct __attribute__ ((__packed__)) LampMultiUpdateReport_
{
    uint8_t  LampCount;          // number of valid entries in the arrays below
    uint8_t  LampUpdateFlags;    // LAMP_UPDATE_FLAG_* bits
    uint16_t LampIds[LAMP_MULTI_UPDATE_LAMP_COUNT];
    LampArrayColor UpdateColors[LAMP_MULTI_UPDATE_LAMP_COUNT];
} LampMultiUpdateReport;

// Report 5 (SET): sets lamps LampIdStart..LampIdEnd (inclusive) to one color.
typedef struct __attribute__ ((__packed__)) LampRangeUpdateReport_
{
    uint8_t  LampUpdateFlags;    // LAMP_UPDATE_FLAG_* bits
    uint16_t LampIdStart;
    uint16_t LampIdEnd;
    LampArrayColor UpdateColor;
} LampRangeUpdateReport;

// Report 6 (SET): AutonomousMode = 1 lets the device control its own lamps;
// 0 gives control to the host.
typedef struct __attribute__ ((__packed__)) LampArrayControlReport_
{
    uint8_t AutonomousMode;
} LampArrayControlReport;