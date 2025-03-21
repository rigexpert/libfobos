//==============================================================================
//       _____     __           _______
//      /  __  \  /_/          /  ____/                                __
//     /  /_ / / _   ____     / /_   __  __   ____    ____    ____   _/ /_
//    /    __ / / / /  _  \  / __/   \ \/ /  / __ \  / __ \  / ___\ /  _/
//   /  /\ \   / / /  /_/ / / /____  /   /  / /_/ / /  ___/ / /     / /_
//  /_ /  \_\ /_/ _\__   / /______/ /_/\_\ / ____/  \____/ /_/      \___/
//               /______/                 /_/             
//  Fobos SDR API library
//  2024.03.21
//  2024.04.08
//  2024.05.29 - sync mode (fobos_rx_start_sync, fobos_rx_read_sync, fobos_rx_stop_sync)
//  2024.06.21 - update fow hw rev.3.0.0
//  2024.07.08 - new band plan
//  2024.07.20 - IQ calibration on the fly
//  2025.01.16 - v.2.3.2 distinguishing the alternative firmware, fobos_rx_write_firmware()
//  2025.01.19 - v.2.3.2 fobos_rx_reset()
//==============================================================================
#ifndef LIB_FOBOS_H
#include <stdint.h>
#ifdef __cplusplus
extern "C"
{
#endif
#ifdef _WIN32
#define CALL_CONV __cdecl
#define API_EXPORT __declspec(dllexport)
#else
#define CALL_CONV
#define API_EXPORT
#endif // _WIN32
//==============================================================================
#define FOBOS_ERR_OK 0
#define FOBOS_ERR_NO_DEV -1
#define FOBOS_ERR_NOT_OPEN -2
#define FOBOS_ERR_NO_MEM -3
#define FOBOS_ERR_CONTROL -4
#define FOBOS_ERR_ASYNC_IN_SYNC -5
#define FOBOS_ERR_SYNC_IN_ASYNC -6
#define FOBOS_ERR_SYNC_NOT_STARTED -7
#define FOBOS_ERR_UNSUPPORTED -8
#define FOBOS_ERR_LIBUSB -9
//==============================================================================
struct fobos_dev_t;
typedef void(*fobos_rx_cb_t)(float *buf, uint32_t buf_length, void *ctx);
//==============================================================================
// obtain the software info
API_EXPORT int CALL_CONV fobos_rx_get_api_info(char * lib_version, char * drv_version);
// obtain connected devices count
API_EXPORT int CALL_CONV fobos_rx_get_device_count(void);
// obtain the list of connected devices if space delimited format
API_EXPORT int CALL_CONV fobos_rx_list_devices(char * serials);
// open the specified device
API_EXPORT int CALL_CONV fobos_rx_open(struct fobos_dev_t ** out_dev, uint32_t index);
// close device
API_EXPORT int CALL_CONV fobos_rx_close(struct fobos_dev_t * dev);
// close and reset device
API_EXPORT int CALL_CONV fobos_rx_reset(struct fobos_dev_t * dev);
// get the board info
API_EXPORT int CALL_CONV fobos_rx_get_board_info(struct fobos_dev_t * dev, char * hw_revision, char * fw_version, char * manufacturer, char * product, char * serial);
// set rx frequency, Hz
API_EXPORT int CALL_CONV fobos_rx_set_frequency(struct fobos_dev_t * dev, double value, double * actual);
// set rx direct sampling mode:  0 - disabled (default),  1 - enabled
API_EXPORT int CALL_CONV fobos_rx_set_direct_sampling(struct fobos_dev_t * dev, unsigned int enabled);
// low noise amplifier 0..2
API_EXPORT int CALL_CONV fobos_rx_set_lna_gain(struct fobos_dev_t * dev, unsigned int value);
// variable gain amplifier 0..15
API_EXPORT int CALL_CONV fobos_rx_set_vga_gain(struct fobos_dev_t * dev, unsigned int value);
// get available sample rate list
API_EXPORT int CALL_CONV fobos_rx_get_samplerates(struct fobos_dev_t * dev, double * values, unsigned int * count);
// set sample rate nearest to specified
API_EXPORT int CALL_CONV fobos_rx_set_samplerate(struct fobos_dev_t * dev, double value, double * actual);
// statr the iq rx streaming
API_EXPORT int CALL_CONV fobos_rx_read_async(struct fobos_dev_t * dev, fobos_rx_cb_t cb, void *ctx, uint32_t buf_count, uint32_t buf_length);
// stop the iq rx streaming
API_EXPORT int CALL_CONV fobos_rx_cancel_async(struct fobos_dev_t * dev);
// set user general purpose output bits (0x00 .. 0xFF)
API_EXPORT int CALL_CONV fobos_rx_set_user_gpo(struct fobos_dev_t * dev, uint8_t value);
// clock source: 0 - internal (default), 1- extrnal
API_EXPORT int CALL_CONV fobos_rx_set_clk_source(struct fobos_dev_t * dev, int value);
// explicitly set the max2830 frequency, Hz (23500000000 .. 2550000000)
API_EXPORT int CALL_CONV fobos_max2830_set_frequency(struct fobos_dev_t * dev, double value, double * actual);
// explicitly set rffc507x frequency, Hz (25000000 .. 5400000000)
API_EXPORT int CALL_CONV fobos_rffc507x_set_lo_frequency_hz(struct fobos_dev_t * dev, uint64_t lo_freq, uint64_t * tune_freq_hz);
// start synchronous rx mode
API_EXPORT int CALL_CONV fobos_rx_start_sync(struct fobos_dev_t * dev, uint32_t buf_length);
// read samples in synchronous rx mode
API_EXPORT int CALL_CONV fobos_rx_read_sync(struct fobos_dev_t * dev, float * buf, uint32_t * actual_buf_length);
// stop synchronous rx mode
API_EXPORT int CALL_CONV fobos_rx_stop_sync(struct fobos_dev_t * dev);
// read firmware from the device
API_EXPORT int CALL_CONV fobos_rx_read_firmware(struct fobos_dev_t* dev, const char * file_name, int verbose);
// write firmware file to the device
API_EXPORT int CALL_CONV fobos_rx_write_firmware(struct fobos_dev_t* dev, const char * file_name, int verbose);
// obtain error text by code
API_EXPORT const char * CALL_CONV fobos_rx_error_name(int error);
//==============================================================================
#ifdef __cplusplus
}
#endif
#endif // !LIB_FOBOS_H
//==============================================================================