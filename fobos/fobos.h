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
//  2025.08.23 - v.2.4.0 DC filter improved, VGA gain fixed
//  2025.10.23 - v.2.4.1 new software DC filter
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
#define FOBOS_ERR_OK                 0
#define FOBOS_ERR_NO_DEV            -1
#define FOBOS_ERR_NOT_OPEN          -2
#define FOBOS_ERR_NO_MEM            -3
#define FOBOS_ERR_CONTROL           -4
#define FOBOS_ERR_ASYNC_IN_SYNC     -5
#define FOBOS_ERR_SYNC_IN_ASYNC     -6
#define FOBOS_ERR_SYNC_NOT_STARTED  -7
#define FOBOS_ERR_UNSUPPORTED       -8
#define FOBOS_ERR_LIBUSB            -9
#define FOBOS_ERR_BAD_PARAM         -10
#define FOBOS_MIN_FREQS_CNT 2
#define FOBOS_MAX_FREQS_CNT 256
//==============================================================================
#define fobos_sdr_dev_t fobos_dev_t
//struct fobos_dev_t;
struct fobos_sdr_dev_t;
typedef void(*fobos_rx_cb_t)(float *buf, uint32_t buf_length, void *ctx);
typedef void(*fobos_sdr_cb_t)(float *buf, uint32_t buf_length, struct fobos_sdr_dev_t* sender, void *user);
typedef void(*fobos_rx_cb_raw_t)(uint16_t *buf, uint32_t buf_length, int need_to_swap_iq, void *ctx);
//==============================================================================
// obtain the software info
API_EXPORT int CALL_CONV fobos_rx_get_api_info(char * lib_version, char * drv_version);
// obtain connected devices count
API_EXPORT int CALL_CONV fobos_rx_get_device_count(void);
// obtain the list of connected devices if space delimited format
API_EXPORT int CALL_CONV fobos_rx_list_devices(char * serials);
// open the specified device by index
API_EXPORT int CALL_CONV fobos_rx_open(struct fobos_dev_t ** out_dev, uint32_t index);
// open the specified device by serial
API_EXPORT int CALL_CONV fobos_rx_open_by_serial(struct fobos_dev_t ** out_dev, char* serial);
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
// start the iq rx streaming
API_EXPORT int CALL_CONV fobos_rx_read_async(struct fobos_dev_t * dev, fobos_rx_cb_t cb, void *ctx, uint32_t buf_count, uint32_t buf_length);
// start the raw uint16_t iq rx streaming
int fobos_rx_read_async_raw(struct fobos_dev_t * dev, fobos_rx_cb_raw_t cb, void *ctx, uint32_t buf_count, uint32_t buf_length);
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
// start the frequencies scanning process or restarts it with new parameters
// frequencies - a list of frequencies to scan to
// count - the list length, should be within a range 2..256;
API_EXPORT int CALL_CONV fobos_rx_start_scan(struct fobos_sdr_dev_t * dev, double *frequencies, unsigned int count);
// stops the scanning
API_EXPORT int CALL_CONV fobos_rx_stop_scan(struct fobos_sdr_dev_t * dev);
// returns the current index of the frequency scanning list
// -1 if scannint mode is not activated or tuning is not complete
API_EXPORT int CALL_CONV fobos_rx_get_scan_index(struct fobos_sdr_dev_t * dev);
// returns 1 if the scanning mode is active
//         0 if there is no scanning mode activated or it was internaly disabled
//         FOBOS_ERR_ if some error happened
API_EXPORT int CALL_CONV fobos_rx_is_scanning(struct fobos_sdr_dev_t * dev);
// reads user data from the device EEPROM (up to 256 bytes)
// (!) can be used for device specific applications
API_EXPORT int CALL_CONV fobos_rx_read_user(struct fobos_dev_t* dev, void * data, int size);
// writes user data to the device EEPROM (up to 256 bytes)
// (!) can be used for device specific applications
API_EXPORT int CALL_CONV fobos_rx_write_user(struct fobos_dev_t* dev, void * data, int size);
// sets rx filter bandwidth relative to sample rate, disables the exact bandwidth mode
// use 0.8 .. 0.9 for the most cases or play around
// set 0.0 or call fobos_sdr_set_bandwidth() to disable the auto bandwidth mode
// (!) further call fobos_sdr_set_samplerate() will auto change the actual bandwidth
API_EXPORT int CALL_CONV fobos_rx_set_auto_bandwidth(struct fobos_dev_t * dev, double value);


//==============================================================================
//==============================================================================
// obtain the software info
API_EXPORT int CALL_CONV fobos_sdr_get_api_info(char * lib_version, char * drv_version);

// obtain connected devices count
API_EXPORT int CALL_CONV fobos_sdr_get_device_count(void);

// obtain the list of connected devices if space delimited format
API_EXPORT int CALL_CONV fobos_sdr_list_devices(char * serials);

// open the specified device
API_EXPORT int CALL_CONV fobos_sdr_open(struct fobos_sdr_dev_t ** out_dev, uint32_t index);

// open the specified device by serial
API_EXPORT int CALL_CONV fobos_sdr_open_by_serial(struct fobos_dev_t ** out_dev, char* serial);

// close device
API_EXPORT int CALL_CONV fobos_sdr_close(struct fobos_sdr_dev_t * dev);

// close and reset device
API_EXPORT int CALL_CONV fobos_sdr_reset(struct fobos_sdr_dev_t * dev);

// get the board info
API_EXPORT int CALL_CONV fobos_sdr_get_board_info(
    struct fobos_sdr_dev_t * dev,
    char * hw_revision,
    char * fw_version,
    char * manufacturer,
    char * product,
    char * serial);

// set rx frequency, Hz
// (!) will be ignored when the frequency scanning process is active
API_EXPORT int CALL_CONV fobos_sdr_set_frequency(struct fobos_sdr_dev_t * dev, double value);

// start the frequencies scanning process or restarts it with new parameters
// frequencies - a list of frequencies to scan to
// count - the list length, should be within a range 2..256;
API_EXPORT int CALL_CONV fobos_sdr_start_scan(struct fobos_sdr_dev_t * dev, double *frequencies, unsigned int count);

// stops the scanning
API_EXPORT int CALL_CONV fobos_sdr_stop_scan(struct fobos_sdr_dev_t * dev);

// returns the current index of the frequency scanning list
// -1 if scannint mode is not activated or tuning is not complete
API_EXPORT int CALL_CONV fobos_sdr_get_scan_index(struct fobos_sdr_dev_t * dev);

// returns 1 if the scanning mode is active
//         0 if there is no scanning mode activated or it was internaly disabled
//         FOBOS_ERR_ if some error happened
API_EXPORT int CALL_CONV fobos_sdr_is_scanning(struct fobos_sdr_dev_t * dev);

// set rx direct sampling mode:  0 - disabled (default),  1 - enabled
API_EXPORT int CALL_CONV fobos_sdr_set_direct_sampling(struct fobos_sdr_dev_t * dev, unsigned int enabled);

// low noise amplifier 0..2
API_EXPORT int CALL_CONV fobos_sdr_set_lna_gain(struct fobos_sdr_dev_t * dev, unsigned int value);

// variable gain amplifier 0..15
API_EXPORT int CALL_CONV fobos_sdr_set_vga_gain(struct fobos_sdr_dev_t * dev, unsigned int value);

// get available sample rate list or sample rate range
// (!) for hw rev.3.x.x and earlier returns the exact available sample rates to be set
// (!) for hw rev.4.x.x and later returns recommended sample rates, one may set arbitrary value in the range
API_EXPORT int CALL_CONV fobos_sdr_get_samplerates(struct fobos_sdr_dev_t * dev, double * values, unsigned int * count);

// set the sample rate
// value - the sample rate (Hz) to be set
// (!) for hw rev.3.x.x and earlier sets the nearest sample rate to specified
// (!) for hw rev.4.x.x and later one may set arbitrary sample rate with double precision
API_EXPORT int CALL_CONV fobos_sdr_set_samplerate(struct fobos_sdr_dev_t * dev, double value);

// sets the baseband low pass filter exact bandwidth, Hz, disables the auto bandwidth mode (see later)
// value - filter bandwidth (Hz)
// (!) the actual bandwidth is set nearest to available in hardware
API_EXPORT int CALL_CONV fobos_sdr_set_bandwidth(struct fobos_sdr_dev_t * dev, double value);

// sets rx filter bandwidth relative to sample rate, disables the exact bandwidth mode
// use 0.8 .. 0.9 for the most cases or play around
// set 0.0 or call fobos_sdr_set_bandwidth() to disable the auto bandwidth mode
// (!) further call fobos_sdr_set_samplerate() will auto change the actual bandwidth
API_EXPORT int CALL_CONV fobos_sdr_set_auto_bandwidth(struct fobos_sdr_dev_t * dev, double value);

// starts the iq rx streaming
// cb - callback function
// user - callback user data (pointer)
// buff_count - libusb buffers count
// buff_len - complex samples per buffer, should be an even multiple of 8192, otherwise will be truncated to nearest one
//            should be at least 65536 for the frequence scanning to be posible
API_EXPORT int CALL_CONV fobos_sdr_read_async(
    struct fobos_sdr_dev_t * dev,
    fobos_sdr_cb_t cb,
    void *user,
    uint32_t buf_count,
    uint32_t buf_length);

// stops the iq rx streaming
// (!) does not terminate the streamming immediately
// (!) actual streaming completion depends on libusb behaviour
API_EXPORT int CALL_CONV fobos_sdr_cancel_async(struct fobos_sdr_dev_t * dev);

// set user general purpose output bits (0x00 .. 0xFF)
API_EXPORT int CALL_CONV fobos_sdr_set_user_gpo(struct fobos_sdr_dev_t * dev, uint8_t value);

// clock source: 0 - internal (default), 1- extrnal
API_EXPORT int CALL_CONV fobos_sdr_set_clk_source(struct fobos_sdr_dev_t * dev, int value);

// start synchronous rx mode
API_EXPORT int CALL_CONV fobos_sdr_start_sync(struct fobos_sdr_dev_t * dev, uint32_t buf_length);

// read samples in synchronous rx mode
API_EXPORT int CALL_CONV fobos_sdr_read_sync(struct fobos_sdr_dev_t * dev, float * buf, uint32_t * actual_buf_length);

// stop synchronous rx mode
API_EXPORT int CALL_CONV fobos_sdr_stop_sync(struct fobos_sdr_dev_t * dev);

// read firmware from the device
API_EXPORT int CALL_CONV fobos_sdr_read_firmware(struct fobos_sdr_dev_t* dev, const char * file_name, int verbose);

// write firmware file to the device
API_EXPORT int CALL_CONV fobos_sdr_write_firmware(struct fobos_sdr_dev_t* dev, const char * file_name, int verbose);

// reads user data from the device EEPROM (up to 256 bytes)
// (!) can be used for device specific applications
API_EXPORT int CALL_CONV fobos_sdr_read_user(struct fobos_sdr_dev_t* dev, void * data, int size);

// writes user data to the device EEPROM (up to 256 bytes)
// (!) can be used for device specific applications
API_EXPORT int CALL_CONV fobos_sdr_write_user(struct fobos_sdr_dev_t* dev, void * data, int size);

// obtain error text by code
API_EXPORT const char * CALL_CONV fobos_sdr_error_name(int error);
//==============================================================================

#ifdef __cplusplus
}
#endif
#endif // !LIB_FOBOS_H
//==============================================================================

