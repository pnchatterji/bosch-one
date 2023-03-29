/**
 * @file bhix60_dbg.c
 * @brief Debugging related functions of the bhix60 driver
 * @copyright Copyright (c) 2023 Bosch Sensortec GmbH
 * 
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bhix60_int.h"
#include <zephyr/logging/log.h>
/*redefine the following constants here to avoid including
advanced sensor headers, which may not be available in some projects*/
#define BHY2_SENSOR_ID_SWIM	     UINT8_C(114)
#define BHY2_SENSOR_ID_KLIO      UINT8_C(112)
#define BHY2_SENSOR_ID_KLIO_LOG  UINT8_C(127)
#define BHY2_SENSOR_ID_PDR       UINT8_C(113)
#define BHY2_SENSOR_ID_PDR_LOG   UINT8_C(119)
#define BHY2_SENSOR_ID_AIR_QUALITY  UINT8_C(115)

LOG_MODULE_DECLARE(bhix60, CONFIG_SENSOR_LOG_LEVEL);

const char* bhix60_get_api_error(int8_t error_code)
{
    switch (error_code)
    {
        case BHY2_OK:
			return "NO Error";
        case BHY2_E_NULL_PTR:
            return "Null pointer";
        case BHY2_E_INVALID_PARAM:
            return "Invalid parameter";
        case BHY2_E_IO:
            return "IO error";
        case BHY2_E_MAGIC:
            return "Invalid firmware";
        case BHY2_E_TIMEOUT:
            return "Timed out";
        case BHY2_E_BUFFER:
            return "Invalid buffer";
        case BHY2_E_INVALID_FIFO_TYPE:
            return "Invalid FIFO type";
        case BHY2_E_INVALID_EVENT_SIZE:
            return "Invalid Event size";
        case BHY2_E_PARAM_NOT_SET:
            return "Parameter not set";
        default:
            return "Unknown API error code";
    }
}

const char* bhix60_get_sensor_error_text(uint8_t sensor_error)
{
    const char *ret=NULL;

    switch (sensor_error)
    {
        case 0x00:
            break;
        case 0x10:
            ret = "Bootloader reports: Firmware Expected Version Mismatch";
            break;
        case 0x11:
            ret = "Bootloader reports: Firmware Upload Failed: Bad Header CRC";
            break;
        case 0x12:
            ret = "Bootloader reports: Firmware Upload Failed: SHA Hash Mismatch";
            break;
        case 0x13:
            ret = "Bootloader reports: Firmware Upload Failed: Bad Image CRC";
            break;
        case 0x14:
            ret = "Bootloader reports: Firmware Upload Failed: ECDSA Signature Verification Failed";
            break;
        case 0x15:
            ret = "Bootloader reports: Firmware Upload Failed: Bad Public Key CRC";
            break;
        case 0x16:
            ret = "Bootloader reports: Firmware Upload Failed: Signed Firmware Required";
            break;
        case 0x17:
            ret = "Bootloader reports: Firmware Upload Failed: FW Header Missing";
            break;
        case 0x19:
            ret = "Bootloader reports: Unexpected Watchdog Reset";
            break;
        case 0x1A:
            ret = "ROM Version Mismatch";
            break;
        case 0x1B:
            ret = "Bootloader reports: Fatal Firmware Error";
            break;
        case 0x1C:
            ret = "Chained Firmware Error: Next Payload Not Found";
            break;
        case 0x1D:
            ret = "Chained Firmware Error: Payload Not Valid";
            break;
        case 0x1E:
            ret = "Chained Firmware Error: Payload Entries Invalid";
            break;
        case 0x1F:
            ret = "Bootloader reports: Bootloader Error: OTP CRC Invalid";
            break;
        case 0x20:
            ret = "Firmware Init Failed";
            break;
        case 0x21:
            ret = "Sensor Init Failed: Unexpected Device ID";
            break;
        case 0x22:
            ret = "Sensor Init Failed: No Response from Device";
            break;
        case 0x23:
            ret = "Sensor Init Failed: Unknown";
            break;
        case 0x24:
            ret = "Sensor Error: No Valid Data";
            break;
        case 0x25:
            ret = "Slow Sample Rate";
            break;
        case 0x26:
            ret = "Data Overflow (saturated sensor data)";
            break;
        case 0x27:
            ret = "Stack Overflow";
            break;
        case 0x28:
            ret = "Insufficient Free RAM";
            break;
        case 0x29:
            ret = "Sensor Init Failed: Driver Parsing Error";
            break;
        case 0x2A:
            ret = "Too Many RAM Banks Required";
            break;
        case 0x2B:
            ret = "Invalid Event Specified";
            break;
        case 0x2C:
            ret = "More than 32 On Change";
            break;
        case 0x2D:
            ret = "Firmware Too Large";
            break;
        case 0x2F:
            ret = "Invalid RAM Banks";
            break;
        case 0x30:
            ret = "Math Error";
            break;
        case 0x40:
            ret = "Memory Error";
            break;
        case 0x41:
            ret = "SWI3 Error";
            break;
        case 0x42:
            ret = "SWI4 Error";
            break;
        case 0x43:
            ret = "Illegal Instruction Error";
            break;
        case 0x44:
            ret = "Bootloader reports: Unhandled Interrupt Error / Exception / Postmortem Available";
            break;
        case 0x45:
            ret = "Invalid Memory Access";
            break;
        case 0x50:
            ret = "Algorithm Error: BSX Init";
            break;
        case 0x51:
            ret = "Algorithm Error: BSX Do Step";
            break;
        case 0x52:
            ret = "Algorithm Error: Update Sub";
            break;
        case 0x53:
            ret = "Algorithm Error: Get Sub";
            break;
        case 0x54:
            ret = "Algorithm Error: Get Phys";
            break;
        case 0x55:
            ret = "Algorithm Error: Unsupported Phys Rate";
            break;
        case 0x56:
            ret = "Algorithm Error: Cannot find BSX Driver";
            break;
        case 0x60:
            ret = "Sensor Self-Test Failure";
            break;
        case 0x61:
            ret = "Sensor Self-Test X Axis Failure";
            break;
        case 0x62:
            ret = "Sensor Self-Test Y Axis Failure";
            break;
        case 0x64:
            ret = "Sensor Self-Test Z Axis Failure";
            break;
        case 0x65:
            ret = "FOC Failure";
            break;
        case 0x66:
            ret = "Sensor Busy";
            break;
        case 0x6F:
            ret = "Self-Test or FOC Test Unsupported";
            break;
        case 0x72:
            ret = "No Host Interrupt Set";
            break;
        case 0x73:
            ret = "Event ID Passed to Host Interface Has No Known Size";
            break;
        case 0x75:
            ret = "Host Download Channel Underflow (Host Read Too Fast)";
            break;
        case 0x76:
            ret = "Host Upload Channel Overflow (Host Wrote Too Fast)";
            break;
        case 0x77:
            ret = "Host Download Channel Empty";
            break;
        case 0x78:
            ret = "DMA Error";
            break;
        case 0x79:
            ret = "Corrupted Input Block Chain";
            break;
        case 0x7A:
            ret = "Corrupted Output Block Chain";
            break;
        case 0x7B:
            ret = "Buffer Block Manager Error";
            break;
        case 0x7C:
            ret = "Input Channel Not Word Aligned";
            break;
        case 0x7D:
            ret = "Too Many Flush Events";
            break;
        case 0x7E:
            ret = "Unknown Host Channel Error";
            break;
        case 0x81:
            ret = "Decimation Too Large";
            break;
        case 0x90:
            ret = "Master SPI/I2C Queue Overflow";
            break;
        case 0x91:
            ret = "SPI/I2C Callback Error";
            break;
        case 0xA0:
            ret = "Timer Scheduling Error";
            break;
        case 0xB0:
            ret = "Invalid GPIO for Host IRQ";
            break;
        case 0xB1:
            ret = "Error Sending Initialized Meta Events";
            break;
        case 0xC0:
            ret = "Bootloader reports: Command Error";
            break;
        case 0xC1:
            ret = "Bootloader reports: Command Too Long";
            break;
        case 0xC2:
            ret = "Bootloader reports: Command Buffer Overflow";
            break;
        case 0xD0:
            ret = "User Mode Error: Sys Call Invalid";
            break;
        case 0xD1:
            ret = "User Mode Error: Trap Invalid";
            break;
        case 0xE1:
            ret = "Firmware Upload Failed: Firmware header corrupt";
            break;
        case 0xE2:
            ret = "Sensor Data Injection: Invalid input stream";
            break;
        default:
            ret = "Unknown error code";
    }

    return ret;
}

const char* bhix60_get_sensor_name(uint8_t sensor_id)
{
    const char *ret=NULL;

    switch (sensor_id)
    {
        case BHY2_SENSOR_ID_ACC_PASS:
            ret = "Accelerometer passthrough";
            break;
        case BHY2_SENSOR_ID_ACC_RAW:
            ret = "Accelerometer uncalibrated";
            break;
        case BHY2_SENSOR_ID_ACC:
            ret = "Accelerometer corrected";
            break;
        case BHY2_SENSOR_ID_ACC_BIAS:
            ret = "Accelerometer offset";
            break;
        case BHY2_SENSOR_ID_ACC_WU:
            ret = "Accelerometer corrected wake up";
            break;
        case BHY2_SENSOR_ID_ACC_RAW_WU:
            ret = "Accelerometer uncalibrated wake up";
            break;
        case BHY2_SENSOR_ID_GYRO_PASS:
            ret = "Gyroscope passthrough";
            break;
        case BHY2_SENSOR_ID_GYRO_RAW:
            ret = "Gyroscope uncalibrated";
            break;
        case BHY2_SENSOR_ID_GYRO:
            ret = "Gyroscope corrected";
            break;
        case BHY2_SENSOR_ID_GYRO_BIAS:
            ret = "Gyroscope offset";
            break;
        case BHY2_SENSOR_ID_GYRO_WU:
            ret = "Gyroscope wake up";
            break;
        case BHY2_SENSOR_ID_GYRO_RAW_WU:
            ret = "Gyroscope uncalibrated wake up";
            break;
        case BHY2_SENSOR_ID_MAG_PASS:
            ret = "Magnetometer passthrough";
            break;
        case BHY2_SENSOR_ID_MAG_RAW:
            ret = "Magnetometer uncalibrated";
            break;
        case BHY2_SENSOR_ID_MAG:
            ret = "Magnetometer corrected";
            break;
        case BHY2_SENSOR_ID_MAG_BIAS:
            ret = "Magnetometer offset";
            break;
        case BHY2_SENSOR_ID_MAG_WU:
            ret = "Magnetometer wake up";
            break;
        case BHY2_SENSOR_ID_MAG_RAW_WU:
            ret = "Magnetometer uncalibrated wake up";
            break;
        case BHY2_SENSOR_ID_GRA:
            ret = "Gravity vector";
            break;
        case BHY2_SENSOR_ID_GRA_WU:
            ret = "Gravity vector wake up";
            break;
        case BHY2_SENSOR_ID_LACC:
            ret = "Linear acceleration";
            break;
        case BHY2_SENSOR_ID_LACC_WU:
            ret = "Linear acceleration wake up";
            break;
        case BHY2_SENSOR_ID_RV:
            ret = "Rotation vector";
            break;
        case BHY2_SENSOR_ID_RV_WU:
            ret = "Rotation vector wake up";
            break;
        case BHY2_SENSOR_ID_GAMERV:
            ret = "Game rotation vector";
            break;
        case BHY2_SENSOR_ID_GAMERV_WU:
            ret = "Game rotation vector wake up";
            break;
        case BHY2_SENSOR_ID_GEORV:
            ret = "Geo-magnetic rotation vector";
            break;
        case BHY2_SENSOR_ID_GEORV_WU:
            ret = "Geo-magnetic rotation vector wake up";
            break;
        case BHY2_SENSOR_ID_ORI:
            ret = "Orientation";
            break;
        case BHY2_SENSOR_ID_ORI_WU:
            ret = "Orientation wake up";
            break;
        case BHY2_SENSOR_ID_TILT_DETECTOR:
            ret = "Tilt detector";
            break;
        case BHY2_SENSOR_ID_STD:
            ret = "Step detector";
            break;
        case BHY2_SENSOR_ID_STC:
            ret = "Step counter";
            break;
        case BHY2_SENSOR_ID_STC_WU:
            ret = "Step counter wake up";
            break;
        case BHY2_SENSOR_ID_SIG:
            ret = "Significant motion";
            break;
        case BHY2_SENSOR_ID_WAKE_GESTURE:
            ret = "Wake gesture";
            break;
        case BHY2_SENSOR_ID_GLANCE_GESTURE:
            ret = "Glance gesture";
            break;
        case BHY2_SENSOR_ID_PICKUP_GESTURE:
            ret = "Pickup gesture";
            break;
        case BHY2_SENSOR_ID_AR:
            ret = "Activity recognition";
            break;
        case BHY2_SENSOR_ID_WRIST_TILT_GESTURE:
            ret = "Wrist tilt gesture";
            break;
        case BHY2_SENSOR_ID_DEVICE_ORI:
            ret = "Device orientation";
            break;
        case BHY2_SENSOR_ID_DEVICE_ORI_WU:
            ret = "Device orientation wake up";
            break;
        case BHY2_SENSOR_ID_STATIONARY_DET:
            ret = "Stationary detect";
            break;
        case BHY2_SENSOR_ID_MOTION_DET:
            ret = "Motion detect";
            break;
        case BHY2_SENSOR_ID_ACC_BIAS_WU:
            ret = "Accelerometer offset wake up";
            break;
        case BHY2_SENSOR_ID_GYRO_BIAS_WU:
            ret = "Gyroscope offset wake up";
            break;
        case BHY2_SENSOR_ID_MAG_BIAS_WU:
            ret = "Magnetometer offset wake up";
            break;
        case BHY2_SENSOR_ID_STD_WU:
            ret = "Step detector wake up";
            break;
        case BHY2_SENSOR_ID_TEMP:
            ret = "Temperature";
            break;
        case BHY2_SENSOR_ID_BARO:
            ret = "Barometer";
            break;
        case BHY2_SENSOR_ID_HUM:
            ret = "Humidity";
            break;
        case BHY2_SENSOR_ID_GAS:
            ret = "Gas";
            break;
        case BHY2_SENSOR_ID_TEMP_WU:
            ret = "Temperature wake up";
            break;
        case BHY2_SENSOR_ID_BARO_WU:
            ret = "Barometer wake up";
            break;
        case BHY2_SENSOR_ID_HUM_WU:
            ret = "Humidity wake up";
            break;
        case BHY2_SENSOR_ID_GAS_WU:
            ret = "Gas wake up";
            break;
        case BHY2_SENSOR_ID_STC_HW:
            ret = "Hardware Step counter";
            break;
        case BHY2_SENSOR_ID_STD_HW:
            ret = "Hardware Step detector";
            break;
        case BHY2_SENSOR_ID_SIG_HW:
            ret = "Hardware Significant motion";
            break;
        case BHY2_SENSOR_ID_STC_HW_WU:
            ret = "Hardware Step counter wake up";
            break;
        case BHY2_SENSOR_ID_STD_HW_WU:
            ret = "Hardware Step detector wake up";
            break;
        case BHY2_SENSOR_ID_SIG_HW_WU:
            ret = "Hardware Significant motion wake up";
            break;
        case BHY2_SENSOR_ID_ANY_MOTION:
            ret = "Any motion";
            break;
        case BHY2_SENSOR_ID_ANY_MOTION_WU:
            ret = "Any motion wake up";
            break;
        case BHY2_SENSOR_ID_EXCAMERA:
            ret = "External camera trigger";
            break;
        case BHY2_SENSOR_ID_GPS:
            ret = "GPS";
            break;
        case BHY2_SENSOR_ID_LIGHT:
            ret = "Light";
            break;
        case BHY2_SENSOR_ID_PROX:
            ret = "Proximity";
            break;
        case BHY2_SENSOR_ID_LIGHT_WU:
            ret = "Light wake up";
            break;
        case BHY2_SENSOR_ID_PROX_WU:
            ret = "Proximity wake up";
            break;
		case BHY2_SENSOR_ID_SWIM:
            ret = "Swim";
			break;
		case BHY2_SENSOR_ID_KLIO:
            ret = "Klio";
			break; 
		case BHY2_SENSOR_ID_KLIO_LOG:
            ret = "Klio Log";
			break;  
		case BHY2_SENSOR_ID_PDR:
            ret = "PDR";
			break;       
		case BHY2_SENSOR_ID_PDR_LOG:
            ret = "PDR Log";
			break;   
        case BHY2_SENSOR_ID_AIR_QUALITY:
            ret = "BSEC";
            break;
        default:
            if ((sensor_id >= BHY2_SENSOR_ID_CUSTOM_START) && (sensor_id <= BHY2_SENSOR_ID_CUSTOM_END))
            {
                ret = "Custom sensor ID ";
            }
            else
            {
                ret = "Undefined sensor ID ";
            }
    }

    return ret;
}


void bhix60_parse_meta_event(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    (void)callback_ref;
    uint8_t meta_event_type = callback_info->data_ptr[0];
    uint8_t byte1 = callback_info->data_ptr[1];
    uint8_t byte2 = callback_info->data_ptr[2];
    uint8_t *accuracy = (uint8_t*)callback_ref;
    char *event_text;

    if (callback_info->sensor_id == BHY2_SYS_ID_META_EVENT)
    {
        event_text = "[META EVENT]";
    }
    else if (callback_info->sensor_id == BHY2_SYS_ID_META_EVENT_WU)
    {
        event_text = "[META EVENT WAKE UP]";
    }
    else
    {
        return;
    }

    switch (meta_event_type)
    {
        case BHY2_META_EVENT_FLUSH_COMPLETE:
            LOG_INF("%s Flush complete for sensor id %u", event_text, byte1);
            break;
        case BHY2_META_EVENT_SAMPLE_RATE_CHANGED:
            LOG_INF("%s Sample rate changed for sensor id %u", event_text, byte1);
            break;
        case BHY2_META_EVENT_POWER_MODE_CHANGED:
            LOG_INF("%s Power mode changed for sensor id %u", event_text, byte1);
            break;
        case BHY2_META_EVENT_ALGORITHM_EVENTS:
            LOG_INF("%s Algorithm event", event_text);
            break;
        case BHY2_META_EVENT_SENSOR_STATUS:
            LOG_INF("%s Accuracy for sensor id %u changed to %u", event_text, byte1, byte2);
            if (accuracy)
            {
                *accuracy = byte2;
            }
            break;
        case BHY2_META_EVENT_BSX_DO_STEPS_MAIN:
            LOG_INF("%s BSX event (do steps main)", event_text);
            break;
        case BHY2_META_EVENT_BSX_DO_STEPS_CALIB:
            LOG_INF("%s BSX event (do steps calib)", event_text);
            break;
        case BHY2_META_EVENT_BSX_GET_OUTPUT_SIGNAL:
            LOG_INF("%s BSX event (get output signal)", event_text);
            break;
        case BHY2_META_EVENT_SENSOR_ERROR:
            LOG_INF("%s Sensor id %u reported error 0x%02X", event_text, byte1, byte2);
            break;
        case BHY2_META_EVENT_FIFO_OVERFLOW:
            LOG_INF("%s FIFO overflow", event_text);
            break;
        case BHY2_META_EVENT_DYNAMIC_RANGE_CHANGED:
            LOG_INF("%s Dynamic range changed for sensor id %u", event_text, byte1);
            break;
        case BHY2_META_EVENT_FIFO_WATERMARK:
            LOG_INF("%s FIFO watermark reached", event_text);
            break;
        case BHY2_META_EVENT_INITIALIZED:
            LOG_INF("%s Firmware initialized. Firmware version %u", event_text, ((uint16_t)byte2 << 8) | byte1);
            break;
        case BHY2_META_TRANSFER_CAUSE:
            LOG_INF("%s Transfer cause for sensor id %u", event_text, byte1);
            break;
        case BHY2_META_EVENT_SENSOR_FRAMEWORK:
            LOG_INF("%s Sensor framework event for sensor id %u", event_text, byte1);
            break;
        case BHY2_META_EVENT_RESET:
            LOG_INF("%s Reset event", event_text);
            break;
        case BHY2_META_EVENT_SPACER:
            break;
        default:
            LOG_INF("%s Unknown meta event with id: %u", event_text, meta_event_type);
            break;
    }
}

void bhix60_convert_time(uint64_t time_ticks, uint32_t *s, uint32_t *ns)
{
    uint64_t timestamp = time_ticks; /* Store the last timestamp */

    timestamp = timestamp * 15625; /* timestamp is now in nanoseconds */
    *s = (uint32_t)(timestamp / UINT64_C(1000000000));
    *ns = (uint32_t)(timestamp - ((*s) * UINT64_C(1000000000)));
}

void bhix60_parse_debug_message(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
  uint32_t s, ns;
  bhix60_convert_time(*callback_info->time_stamp, &s, &ns);
  LOG_DBG("[BHIx60 DEBUG MSG]; T: %u.%09u; flag: 0x%x; data: %s",
        s,
        ns,
        callback_info->data_ptr[0],
        &callback_info->data_ptr[1]);
}

void bhix60_print_sensors(struct bhy2_dev *p_bhy2) 
{
    uint32_t sensor_id =0;

    LOG_INF("BHIx60 Present sensors: ");
    for (uint16_t i = 0; i < sizeof(p_bhy2->present_buff); i++)
    {
        for (uint8_t j = 0; j < 8; j++)
        {
            sensor_id = i*8+j;
            if((p_bhy2->present_buff[i] >> j) & 0x01)
            {
                 LOG_INF("%d - %s",sensor_id,bhix60_get_sensor_name(sensor_id));
            }
        }
    }
}