//==============================================================================
//       _____     __           _______
//      /  __  \  /_/          /  ____/                                __
//     /  /_ / / _   ____     / /__  __  __   ____    ____    ____   _/ /_
//    /    __ / / / /  _  \  / ___/  \ \/ /  / __ \  / __ \  / ___\ /  _/
//   /  /\ \   / / /  /_/ / / /___   /   /  / /_/ / /  ___/ / /     / /_
//  /_ /  \_\ /_/  \__   / /______/ /_/\_\ / ____/  \____/ /_/      \___/
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
//==============================================================================
#define _CRT_SECURE_NO_WARNINGS
#include "fobos.h"
#include <float.h>
#include <limits.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef _WIN32
#include <Windows.h>
#include <conio.h>
#include <libusb-1.0/libusb.h>
#pragma comment(lib, "libusb-1.0.lib")
#define printf_internal _cprintf
#else
#include <libusb-1.0/libusb.h>
#include <unistd.h>
#endif
#ifndef printf_internal
#define printf_internal printf
#endif // !printf_internal
//==============================================================================
#define FOBOS_PRINT_DEBUG
//==============================================================================
#define LIB_VERSION "2.4.0"
#define DRV_VERSION "libusb"
//==============================================================================
#define FOBOS_VENDOR_ID 0x16d0
#define FOBOS_PRODUCT_ID 0x132e
#define FOBOS_DEV_ID 0x0000
//==============================================================================
#define FOBOS_DEV_PRESEL_V1 0
#define FOBOS_DEV_PRESEL_V2 1
#define FOBOS_DEV_LNA_LP_SHD 2
#define FOBOS_DEV_LNA_HP_SHD 3
#define FOBOS_DEV_IF_V1 4
#define FOBOS_DEV_IF_V2 5
#define FOBOS_DEV_OQMUX 6
#define FOBOS_DEV_LPF_A0 6
#define FOBOS_DEV_LPF_A1 7
#define FOBOS_DEV_NENBL_HF 8
#define FOBOS_DEV_CLKSEL 9
#define FOBOS_DEV_ADC_NCS 10
#define FOBOS_DEV_ADC_SCK 11
#define FOBOS_DEV_ADC_SDI 12
#define FOBOS_MAX2830_ANTSEL 13
//==============================================================================
#define bitset(x, nbit) ((x) |= (1 << (nbit)))
#define bitclear(x, nbit) ((x) &= ~(1 << (nbit)))
#define FOBOS_DEF_BUF_COUNT 16
#define FOBOS_MAX_BUF_COUNT 64
#define FOBOS_DEF_BUF_LENGTH (16 * 32 * 512)
#define LIBUSB_BULK_TIMEOUT 0
#define LIBUSB_BULK_IN_ENDPOINT 0x81
#define LIBUSB_DDESCRIPTOR_LEN 64
#ifndef LIBUSB_CALL
#define LIBUSB_CALL
#endif
//==============================================================================
static const float SAMPLE_NORM = (1.0 / (float)(SHRT_MAX >> 2)); // 14 bit ADC
static const int16_t SAMPLE_OFFSET = (int16_t)(1 << 13);
//==============================================================================
static const float PI = 3.14159265F;
static const bool CALIBRATION_DEBUG_GAINS = true;
static const bool CALIBRATION_DEBUG_SIGNAL = false;
static const size_t CALIBRATION_NUM_STEPS = 8U;
static const uint32_t CALIBRATION_DC_OFFSET_MAX2830_LNA_GAIN = 1U;
static const uint32_t CALIBRATION_DC_OFFSET_MAX2830_VGA_GAIN = 0U;
static const uint32_t CALIBRATION_IQ_CALIBRATION_MAX2830_LNA_GAIN = 1U;
static const uint32_t CALIBRATION_IQ_CALIBRATION_MAX2830_VGA_GAIN = 0U;
static const uint64_t CALIBRATION_MAX2830_LO_FREQUENCY = 2350000000U;
static const uint64_t CALIBRATION_RFFC5072_LO_FREQUENCY_OFFSET = 5000000U;
static const float CALIBRATION_SUCCESS_PHASE_DIFFERENCE_RAD = 0.2F * PI / 180.0;
static const float CALIBRATION_SUCCESS_GAIN_RATIO = 0.001F;
enum fobos_calibration_state
{
    CALIBRATION_START = 0,
    CALIBRATION_DC_OFFSET,
    CALIBRATION_IQ_BALANCE,
    CALIBRATION_SANITY_CHECK,
    CALIBRATION_DONE
};
//==============================================================================
static const uint32_t FREQUENCY_BAND_UNSET = UINT32_MAX;
static const double FREQUENCY_UNSET = 0.0;
static const double DEFAULT_FREQUENCY = 400e6;
static const double DEFAULT_SAMPLERATE = 25e6;
//==============================================================================
enum fobos_async_status
{
    FOBOS_IDLE = 0,
    FOBOS_STARTING,
    FOBOS_RUNNING,
    FOBOS_CANCELING
};
//==============================================================================
struct fobos_dev_t
{
    //=== libusb ===============================================================
    libusb_context *libusb_ctx;
    struct libusb_device_handle *libusb_devh;
    uint32_t transfer_buf_count;
    uint32_t transfer_buf_size;
    struct libusb_transfer **transfer;
    unsigned char **transfer_buf;
    int transfer_errors;
    int dev_lost;
    int use_zerocopy;
    //=== common ===============================================================
    uint16_t user_gpo;
    uint16_t dev_gpo;
    char hw_revision[32];
    char fw_version[32];
    char fw_build[32];
    char manufacturer[LIBUSB_DDESCRIPTOR_LEN];
    char product[LIBUSB_DDESCRIPTOR_LEN];
    char serial[LIBUSB_DDESCRIPTOR_LEN];
    //=== rx stuff =============================================================
    double rx_frequency;
    uint32_t rx_frequency_band;
    double rx_samplerate;
    double rx_bandwidth;
    double max2830_lo_frequency;
    uint32_t rx_lpf_idx;
    uint32_t _rx_lna_gain;
    uint32_t _rx_vga_gain;
    uint32_t rx_lna_gain;
    uint32_t rx_vga_gain;
    uint32_t rx_bw_idx;
    uint32_t rx_bw_adj;
    uint32_t rx_direct_sampling;
    fobos_rx_cb_t rx_cb;
    void *rx_cb_ctx;
    enum fobos_async_status rx_async_status;
    int rx_async_cancel;
    uint32_t rx_failures;
    uint32_t rx_buff_counter;
    int rx_swap_iq;
    enum fobos_calibration_state rx_calibration_state;
    size_t rx_calibration_pos;
    int64_t summ_re;
    int64_t summ_im;
    double re_re;
    double re_im;
    double im_im;
    size_t num_calibration_samples;
    float rx_dc_re;
    float rx_dc_im;
    float rx_avg_re;
    float rx_avg_im;
    float rx_calibration_a11;
    float rx_calibration_a21;
    float rx_calibration_a22;
    float *rx_buff;
    double max2830_clock;
    uint64_t rffc507x_clock;
    uint16_t rffc507x_registers_local[31];
    uint16_t rffc500x_registers_remote[31];
    int rx_sync_started;
    unsigned char *rx_sync_buf;
    int do_reset;
};
//==============================================================================
static inline int16_t to_signed(int16_t offset_binary_value)
{
    int16_t result = offset_binary_value & (int16_t)0x3FFF;
    return result - SAMPLE_OFFSET;
}
//==============================================================================
char *to_bin(uint16_t s16, char *str)
{
    for (uint16_t i = 0; i < 16; i++)
    {
        *str = ((s16 & 0x8000) >> 15) + '0';
        str++;
        s16 <<= 1;
    }
    *str = 0;
    return str;
}
//==============================================================================
void print_buff(void *buff, int size)
{
    uint16_t *b16 = (uint16_t *)buff;
    int count = size / 4;
    char bin_re[17];
    char bin_im[17];
    for (int i = 0; i < count; i++)
    {
        uint16_t re16 = b16[i * 2 + 0] & 0xFFFF;
        uint16_t im16 = b16[i * 2 + 1] & 0xFFFF;
        to_bin(re16, bin_re);
        to_bin(im16, bin_im);
        printf_internal("%s % 6d  %s % 6d \r\n", bin_re, re16, bin_im, im16);
    }
}
//==============================================================================
void fobos_rffc507x_register_modify(uint16_t *p_data, uint8_t bit_to, uint8_t bit_from, uint16_t value)
{
    uint16_t mask = (~(~0u << (bit_to - bit_from + 1))) << (bit_from);
    *p_data = (*p_data & (~mask)) | ((value << bit_from) & mask);
}
//==============================================================================
int fobos_rx_get_api_info(char *lib_version, char *drv_version)
{
    if (lib_version)
    {
        strcpy(lib_version, LIB_VERSION " " __DATE__ " " __TIME__);
    }
    if (drv_version)
    {
        strcpy(drv_version, DRV_VERSION);
    }
    return 0;
}
//==============================================================================
int fobos_rx_get_device_count(void)
{
    int i;
    int result;
    libusb_context *ctx;
    libusb_device **list;
    uint32_t device_count = 0;
    struct libusb_device_descriptor dd;
    ssize_t cnt;
#ifdef FOBOS_PRINT_DEBUG
    printf_internal("%s();\n", __FUNCTION__);
#endif // FOBOS_PRINT_DEBUG
    result = libusb_init(&ctx);
    if (result < 0)
    {
        return 0;
    }
    cnt = libusb_get_device_list(ctx, &list);
    for (i = 0; i < cnt; i++)
    {
        libusb_get_device_descriptor(list[i], &dd);
#ifdef FOBOS_PRINT_DEBUG
        printf_internal("%04x:%04x\n", dd.idVendor, dd.idProduct);
#endif // FOBOS_PRINT_DEBUG
        if ((dd.idVendor == FOBOS_VENDOR_ID) && (dd.idProduct == FOBOS_PRODUCT_ID) && (dd.bcdDevice == FOBOS_DEV_ID))
        {
            device_count++;
        }
    }
    libusb_free_device_list(list, 1);
    libusb_exit(ctx);
    return device_count;
}
//==============================================================================
int fobos_rx_list_devices(char *serials)
{
    int i;
    int result;
    libusb_context *ctx;
    libusb_device **list;
    uint32_t device_count = 0;
    struct libusb_device_descriptor dd;
    ssize_t cnt;
    libusb_device_handle *handle;
    char string[256];
#ifdef FOBOS_PRINT_DEBUG
    printf_internal("%s();\n", __FUNCTION__);
#endif // FOBOS_PRINT_DEBUG
    memset(string, 0, sizeof(string));
    result = libusb_init(&ctx);
    if (result < 0)
    {
        return 0;
    }
    cnt = libusb_get_device_list(ctx, &list);
    for (i = 0; i < cnt; i++)
    {
        libusb_get_device_descriptor(list[i], &dd);
        if ((dd.idVendor == FOBOS_VENDOR_ID) && (dd.idProduct == FOBOS_PRODUCT_ID) && (dd.bcdDevice == FOBOS_DEV_ID))
        {
            if (serials)
            {
                handle = 0;
                result = libusb_open(list[i], &handle);
                if ((result == 0) && (handle))
                {
                    result = libusb_get_string_descriptor_ascii(handle, dd.iSerialNumber, (unsigned char *)string,
                                                                sizeof(string));
                    if (result > 0)
                    {
                        serials = strcat(serials, string);
                    }
                    libusb_close(handle);
                }
                else
                {
                    serials = strcat(serials, "XXXXXXXXXXXX");
                }
                serials = strcat(serials, " ");
            }
            device_count++;
        }
    }
    libusb_free_device_list(list, 1);
    libusb_exit(ctx);
    return device_count;
}
//==============================================================================
int fobos_check(struct fobos_dev_t *dev)
{
    if (dev != NULL)
    {
        if ((dev->libusb_ctx != NULL) && (dev->libusb_devh != NULL))
        {
            return FOBOS_ERR_OK;
        }
        return FOBOS_ERR_NOT_OPEN;
    }
    return FOBOS_ERR_NO_DEV;
}
//==============================================================================
#define CTRLI (LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_IN)
#define CTRLO (LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT)
#define CTRL_TIMEOUT 300
//==============================================================================
void fobos_i2c_write(struct fobos_dev_t *dev, uint8_t address, uint8_t *data, uint16_t size)
{
    uint8_t req_code = 0xE7;
    int result = fobos_check(dev);
    uint16_t xsize;
    if (result == 0)
    {
        if ((data != 0) && (size > 0))
        {
            xsize = libusb_control_transfer(dev->libusb_devh, CTRLO, req_code, address, 0, data, size, CTRL_TIMEOUT);
            if (xsize != size)
            {
                result = FOBOS_ERR_CONTROL;
            }
        }
    }
    if (result != 0)
    {
        printf_internal("fobos_i2c_write() err %d\n", result);
    }
}
//==============================================================================
void fobos_i2c_read(struct fobos_dev_t *dev, uint8_t address, uint8_t *data, uint16_t size)
{
    uint8_t req_code = 0xE7;
    int result = fobos_check(dev);
    uint16_t xsize;
    if (result == 0)
    {
        if ((data != 0) && (size > 0))
        {
            xsize = libusb_control_transfer(dev->libusb_devh, CTRLI, req_code, address, 0, data, size, CTRL_TIMEOUT);
            if (xsize != size)
            {
                result = FOBOS_ERR_CONTROL;
            }
        }
    }
    if (result != 0)
    {
        printf_internal("fobos_i2c_read() err %d\n", result);
    }
}
//==============================================================================
void fobos_max2830_write_reg(struct fobos_dev_t *dev, uint8_t addr, uint16_t data)
{
    uint8_t req_code = 0xE5;
    int result = fobos_check(dev);
    uint8_t tx[3];
    uint16_t xsize;
    tx[0] = addr;
    tx[1] = data & 0xFF;
    tx[2] = (data >> 8) & 0xFF;
    if (result == 0)
    {
        xsize = libusb_control_transfer(dev->libusb_devh, CTRLO, req_code, 1, 0, tx, 3, CTRL_TIMEOUT);
        if (xsize != 3)
        {
            result = FOBOS_ERR_CONTROL;
        }
    }
    if (result != 0)
    {
        printf_internal("fobos_max2830_write_reg() err %d\n", result);
    }
}
//==============================================================================
int fobos_max2830_init(struct fobos_dev_t *dev)
{
    fobos_max2830_write_reg(dev, 0, 0x0740);
    fobos_max2830_write_reg(dev, 1, 0x119A);
    fobos_max2830_write_reg(dev, 2, 0x1003);
    fobos_max2830_write_reg(dev, 3, 0x0079);
    fobos_max2830_write_reg(dev, 4, 0x3666);
    fobos_max2830_write_reg(dev, 5, 0x00A0); // D2 = 0 -> Reference Frequency Divider = 1
    fobos_max2830_write_reg(dev, 6, 0x0060);
    fobos_max2830_write_reg(dev, 7, 0x1022);
    fobos_max2830_write_reg(dev, 8, 0x3020);
    fobos_max2830_write_reg(dev, 9, 0x03B5);
    fobos_max2830_write_reg(dev, 10, 0x1DA4);
    fobos_max2830_write_reg(dev, 11, 0x0000);
    fobos_max2830_write_reg(dev, 12, 0x0140);
    fobos_max2830_write_reg(dev, 13, 0x0E92);
    fobos_max2830_write_reg(dev, 14, 0x033B);
    fobos_max2830_write_reg(dev, 15, 0x0145);
    return 0;
}
//==============================================================================
int fobos_max2830_set_frequency(struct fobos_dev_t *dev, double value, double *actual)
{
#ifdef FOBOS_PRINT_DEBUG
    printf_internal("%s(%f);\n", __FUNCTION__, value);
#endif // FOBOS_PRINT_DEBUG
    double fcomp = dev->max2830_clock;
    if (fcomp > 26000000.0)
    {
        fcomp /= 2.0;
        fobos_max2830_write_reg(dev, 5, 0x00A4); // Reference Frequency Divider = 2
    }
    else
    {
        fobos_max2830_write_reg(dev, 5, 0x00A0); // Reference Frequency Divider = 1
    }
    double div = value / fcomp;
    uint32_t div_int = (uint32_t)(div) & 0x000000FF;
    uint32_t div_frac = (uint32_t)((div - div_int) * 1048575.0 + 0.5);
    if (actual)
    {
        div = (double)(div_int) + (double)(div_frac) / 1048575.0;
        *actual = div * fcomp;
    }
    fobos_max2830_write_reg(dev, 3, ((div_frac << 8) | div_int) & 0x3FFF);
    fobos_max2830_write_reg(dev, 4, (div_frac >> 6) & 0x3FFF);
    return 0;
}
//==============================================================================
int fobos_rffc507x_write_reg(struct fobos_dev_t *dev, uint8_t addr, uint16_t data)
{
    uint8_t req_code = 0xE6;
    int result = fobos_check(dev);
    uint8_t tx[3];
    uint16_t xsize;
    tx[0] = addr;
    tx[1] = data & 0xFF;
    tx[2] = (data >> 8) & 0xFF;
    if (result == 0)
    {
        xsize = libusb_control_transfer(dev->libusb_devh, CTRLO, req_code, 1, 0, tx, 3, CTRL_TIMEOUT);
        if (xsize != 3)
        {
            result = FOBOS_ERR_CONTROL;
        }
    }
    return result;
}
//==============================================================================
int fobos_rffc507x_read_reg(struct fobos_dev_t *dev, uint8_t addr, uint16_t *data)
{
    uint8_t req_code = 0xE6;
    int result = fobos_check(dev);
    uint8_t rx[2];
    uint16_t xsize;
    if ((result == 0) && data)
    {
        xsize = libusb_control_transfer(dev->libusb_devh, CTRLI, req_code, addr, 0, rx, 2, CTRL_TIMEOUT);
        *data = rx[0] | (rx[1] << 8);
        if (xsize != 2)
        {
            result = FOBOS_ERR_CONTROL;
        }
    }
    return result;
}
//==============================================================================
#define RFFC507X_REGS_COUNT 31
static const uint16_t rffc507x_regs_default[RFFC507X_REGS_COUNT] = {
    0xbefa, /* 0x00   1011 1110 1111 1010*/
    0x4064, /* 0x01 */
    0x9055, /* 0x02 */
    0x2d02, /* 0x03 */
    0xb0bf, /* 0x04 */
    0xb0bf, /* 0x05 */
    0x0028, /* 0x06 */
    0x0028, /* 0x07 */
    0xfc06, /* 0x08   1111 1111 0000 0000 */
    0x8220, /* 0x09   1000 0010 0010 0000 */
    0x0202, /* 0x0A */
    0x4800, /* 0x0B   0100 1000 0000 0000*/
    0x2324, /* 0x0C */
    0x6276, /* 0x0D */
    0x2700, /* 0x0E */
    0x2f16, /* 0x0F */
    0x3b13, /* 0x10 */
    0xb100, /* 0x11 */
    0x2a80, /* 0x12 */
    0x0000, /* 0x13 */
    0x0000, /* 0x14 */
    0x0000, /* 0x15 */
    0x0000, /* 0x16 */
    0x4900, /* 0x17 */
    0x0283, /* 0x18 */
    0xf00f, /* 0x19 */
    0x0000, /* 0x1A */
    0x000F, /* 0x1B */
    0xc840, /* 0x1C */
    0x1000, /* 0x1D */
    0x0001, /* 0x1E */
};
//==============================================================================
int fobos_rffc507x_commit(struct fobos_dev_t *dev, int force)
{
    if (fobos_check(dev) == 0)
    {
        for (int i = 0; i < RFFC507X_REGS_COUNT; i++)
        {
            uint16_t local = dev->rffc507x_registers_local[i];
            if ((dev->rffc500x_registers_remote[i] != local) || force)
            {
                fobos_rffc507x_write_reg(dev, i, local);
            }
            dev->rffc500x_registers_remote[i] = local;
        }
        return FOBOS_ERR_OK;
    }
    return FOBOS_ERR_NO_DEV;
}
//==============================================================================
int fobos_rffc507x_init(struct fobos_dev_t *dev)
{
    int i;
    if (fobos_check(dev) == 0)
    {
        for (i = 0; i < RFFC507X_REGS_COUNT; i++)
        {
            fobos_rffc507x_write_reg(dev, i, rffc507x_regs_default[i]);
            dev->rffc507x_registers_local[i] = rffc507x_regs_default[i];
            dev->rffc500x_registers_remote[i] = rffc507x_regs_default[i];
        }
#ifdef FOBOS_PRINT_DEBUG
        uint16_t data = 0;
        for (i = 0; i < RFFC507X_REGS_COUNT; i++)
        {
            fobos_rffc507x_read_reg(dev, i, &data);
            printf_internal("0x%04x\n", data);
        }
#endif // FOBOS_PRINT_DEBUG
       // ENBL and MODE pins are ignored and become available as GPO5 and GPO6
        fobos_rffc507x_register_modify(&dev->rffc507x_registers_local[0x15], 15, 15, 1);
        // 0 = half duplex
        fobos_rffc507x_register_modify(&dev->rffc507x_registers_local[0x0B], 15, 15, 0);
        int MIX1_IDD = 1;
        int MIX2_IDD = 1;
        int mix = (MIX1_IDD << 3) | MIX2_IDD;
        fobos_rffc507x_register_modify(&dev->rffc507x_registers_local[0x0B], 14, 9, mix);

        // MODE pin = 1, Active PLL Register Bank = 2, Active Mixer = 2;
        fobos_rffc507x_register_modify(&dev->rffc507x_registers_local[0x15], 13, 13, 1);

        // Best performance settings according to Section 2.1 of "Integrated Synthesizer/Mixer Programming Guide"
        fobos_rffc507x_register_modify(&dev->rffc507x_registers_local[0x0C], 1, 0, 0);
        fobos_rffc507x_register_modify(&dev->rffc507x_registers_local[0x0F], 1, 0, 0);

        fobos_rffc507x_register_modify(&dev->rffc507x_registers_local[0x08], 7, 1, 0);
        fobos_rffc507x_register_modify(&dev->rffc507x_registers_local[0x08], 14, 8, 127);

        fobos_rffc507x_register_modify(&dev->rffc507x_registers_local[0x04], 12, 8, 12);
        fobos_rffc507x_register_modify(&dev->rffc507x_registers_local[0x05], 12, 8, 12);

        fobos_rffc507x_register_modify(&dev->rffc507x_registers_local[0x1E], 2, 2, 1);

        // Charge pump up enable
        fobos_rffc507x_register_modify(&dev->rffc507x_registers_local[0x03], 2, 1, 3);

        // Wide lock detect range
        fobos_rffc507x_register_modify(&dev->rffc507x_registers_local[0x09], 4, 4, 1);

        // Tune CT calibration parameters
        // Maximum possible duration
        fobos_rffc507x_register_modify(&dev->rffc507x_registers_local[0x02], 14, 10, 31);
        // Average 128 samples
        fobos_rffc507x_register_modify(&dev->rffc507x_registers_local[0x03], 14, 13, 3);

        // Enable KV calibration
        // Average 128 samples
        fobos_rffc507x_register_modify(&dev->rffc507x_registers_local[0x03], 10, 9, 3);
        // Enable KV calibration for Path 1
        fobos_rffc507x_register_modify(&dev->rffc507x_registers_local[0x06], 15, 15, 1);
        // Enable KV calibration for Path 2
        fobos_rffc507x_register_modify(&dev->rffc507x_registers_local[0x07], 15, 15, 1);

        // Zero frequency control for Path 1
        fobos_rffc507x_register_modify(&dev->rffc507x_registers_local[0x0C], 6, 4, 0);  // p1lodiv
        fobos_rffc507x_register_modify(&dev->rffc507x_registers_local[0x0C], 15, 7, 0); // p1n
        fobos_rffc507x_register_modify(&dev->rffc507x_registers_local[0x0C], 3, 2, 0);  // p1presc
        fobos_rffc507x_register_modify(&dev->rffc507x_registers_local[0x0D], 15, 0, 0); // p1nmsb
        fobos_rffc507x_register_modify(&dev->rffc507x_registers_local[0x0E], 15, 8, 0); // p1nlsb

        // Zero frequency control for Path 2
        fobos_rffc507x_register_modify(&dev->rffc507x_registers_local[0x0F], 6, 4, 0);  // p2lodiv
        fobos_rffc507x_register_modify(&dev->rffc507x_registers_local[0x0F], 15, 7, 0); // p1n
        fobos_rffc507x_register_modify(&dev->rffc507x_registers_local[0x0F], 3, 2, 0);  // p1presc
        fobos_rffc507x_register_modify(&dev->rffc507x_registers_local[0x10], 15, 0, 0); // p1nmsb
        fobos_rffc507x_register_modify(&dev->rffc507x_registers_local[0x11], 15, 8, 0); // p1nlsb

        fobos_rffc507x_commit(dev, 0);
        return FOBOS_ERR_OK;
    }
    return FOBOS_ERR_NO_DEV;
}
//==============================================================================
int fobos_rffc507x_set_lo_frequency_hz(struct fobos_dev_t *dev, uint64_t lo_freq_hz, uint64_t *tune_freq_hz)
{
    uint64_t lodiv;
    uint64_t fvco;
    uint32_t fbkdiv;
    uint16_t pllcpl;
    uint16_t n;
    uint16_t p1nmsb;
    uint16_t p1nlsb;
    uint64_t lo_max = 5400000000ULL;
    uint64_t fref = 25000000ULL;
    fref = dev->rffc507x_clock;
    uint16_t n_lo = 0; // aka p1lodiv
    uint32_t x = lo_max / lo_freq_hz;
    while ((x > 1) && (n_lo < 5))
    {
        n_lo++;
        x >>= 1;
    }

    lodiv = 1 << n_lo;
    fvco = lodiv * lo_freq_hz;

    fbkdiv = 2;
    pllcpl = 2;
    if (fvco > 3200000000ULL)
    {
        fbkdiv = 4;
        pllcpl = 3;
    }

    fobos_rffc507x_register_modify(&dev->rffc507x_registers_local[0x15], 14, 14, 0); // enbl = 0
    fobos_rffc507x_commit(dev, 0);

    fobos_rffc507x_register_modify(&dev->rffc507x_registers_local[0x00], 2, 0, pllcpl);

    uint64_t tmp_n = (fvco << 29ULL) / (fbkdiv * fref);
    n = tmp_n >> 29ULL;

    p1nmsb = (tmp_n >> 13ULL) & 0xffff;
    p1nlsb = (tmp_n >> 5ULL) & 0xff;
    uint64_t freq_hz = (fref * (tmp_n >> 5ULL) * fbkdiv) / (lodiv * (1 << 24ULL));
    if (tune_freq_hz)
    {
        *tune_freq_hz = freq_hz;
    }
    // Path 1
    fobos_rffc507x_register_modify(&dev->rffc507x_registers_local[0x0C], 6, 4, n_lo);        // p1lodiv
    fobos_rffc507x_register_modify(&dev->rffc507x_registers_local[0x0C], 15, 7, n);          // p1n
    fobos_rffc507x_register_modify(&dev->rffc507x_registers_local[0x0C], 3, 2, fbkdiv >> 1); // p1presc
    fobos_rffc507x_register_modify(&dev->rffc507x_registers_local[0x0D], 15, 0, p1nmsb);     // p1nmsb
    fobos_rffc507x_register_modify(&dev->rffc507x_registers_local[0x0E], 15, 8, p1nlsb);     // p1nlsb
                                                                                             // Path 2
    fobos_rffc507x_register_modify(&dev->rffc507x_registers_local[0x0F], 6, 4, n_lo);        // p2lodiv
    fobos_rffc507x_register_modify(&dev->rffc507x_registers_local[0x0F], 15, 7, n);          // p1n
    fobos_rffc507x_register_modify(&dev->rffc507x_registers_local[0x0F], 3, 2, fbkdiv >> 1); // p1presc
    fobos_rffc507x_register_modify(&dev->rffc507x_registers_local[0x10], 15, 0, p1nmsb);     // p1nmsb
    fobos_rffc507x_register_modify(&dev->rffc507x_registers_local[0x11], 15, 8, p1nlsb);     // p1nlsb

    fobos_rffc507x_commit(dev, 0);

    fobos_rffc507x_register_modify(&dev->rffc507x_registers_local[0x15], 14, 14, 1); // enbl = 1
    fobos_rffc507x_commit(dev, 0);
#ifdef FOBOS_PRINT_DEBUG
    double ff = (double)freq_hz;
    printf_internal("rffc507x lo_freq_mhz = %lld %f\n", lo_freq_hz, ff);
#endif // FOBOS_PRINT_DEBUG
    return 0;
}
//==============================================================================
#define SI5351C_ADDRESS 0x60
void fobos_si5351c_write_reg(struct fobos_dev_t *dev, uint8_t reg, uint8_t val)
{
    uint8_t data[] = {reg, val};
    fobos_i2c_write(dev, SI5351C_ADDRESS, data, sizeof(data));
}
//==============================================================================
uint8_t fobos_si5351c_read_reg(struct fobos_dev_t *dev, uint8_t reg)
{
    uint8_t data = 0x00;
    fobos_i2c_write(dev, SI5351C_ADDRESS, &reg, 1);
    fobos_i2c_read(dev, SI5351C_ADDRESS, &data, 1);
    return data;
}
//==============================================================================
void fobos_si5351c_write(struct fobos_dev_t *dev, uint8_t *data, uint16_t size)
{
    fobos_i2c_write(dev, SI5351C_ADDRESS, data, size);
}
//==============================================================================
void fobos_si5351c_read(struct fobos_dev_t *dev, uint8_t *data, uint16_t size)
{
    fobos_i2c_read(dev, SI5351C_ADDRESS, data, size);
}
//==============================================================================
void fobos_si5351c_config_pll(struct fobos_dev_t *dev, uint8_t ms_number, uint32_t p1, uint32_t p2, uint32_t p3)
{
    ms_number &= 0x03;
    uint8_t addr = 26 + (ms_number * 8);
    uint8_t data[] = {addr,
                      (p3 >> 8) & 0xFF,
                      (p3 >> 0) & 0xFF,
                      (p1 >> 16) & 0x3,
                      (p1 >> 8) & 0xFF,
                      (p1 >> 0) & 0xFF,
                      (((p3 >> 16) & 0xF) << 4) | (((p2 >> 16) & 0xF) << 0),
                      (p2 >> 8) & 0xFF,
                      (p2 >> 0) & 0xFF};
    fobos_si5351c_write(dev, data, sizeof(data));
}
//==============================================================================
void fobos_si5351c_config_msynth(struct fobos_dev_t *dev, uint8_t ms_number, uint32_t p1, uint32_t p2, uint32_t p3,
                                 uint8_t r_div)
{
    uint8_t addr = 42 + (ms_number * 8);
    uint8_t data[] = {addr,
                      (p3 >> 8) & 0xFF,
                      (p3 >> 0) & 0xFF,
                      (r_div << 4) | (0 << 2) | ((p1 >> 16) & 0x3),
                      (p1 >> 8) & 0xFF,
                      (p1 >> 0) & 0xFF,
                      (((p3 >> 16) & 0xF) << 4) | (((p2 >> 16) & 0xF) << 0),
                      (p2 >> 8) & 0xFF,
                      (p2 >> 0) & 0xFF};
    fobos_si5351c_write(dev, data, sizeof(data));
}
//==============================================================================
uint8_t si5351c_compose_clk_ctrl(uint8_t pwr_down, uint8_t int_mode, uint8_t ms_src_pll, uint8_t invert,
                                 uint8_t clk_source, uint8_t drv_strength)
{
    uint8_t result = 0;
    result |= ((pwr_down & 1) << 7);
    result |= ((int_mode & 1) << 6);
    result |= ((ms_src_pll & 1) << 5);
    result |= ((invert & 1) << 4);
    result |= ((clk_source & 3) << 2);
    result |= (drv_strength & 3);
    return result;
}
//==============================================================================
int fobos_rffc507x_clock(struct fobos_dev_t *dev, int enabled)
{
    uint8_t pwr_down = (enabled == 0);
    uint8_t data = si5351c_compose_clk_ctrl(pwr_down, 1, 0, 0, 3, 1);
    fobos_si5351c_write_reg(dev, 16, data);
    return 0;
}
//==============================================================================
int fobos_max2830_clock(struct fobos_dev_t *dev, int enabled)
{
    uint8_t pwr_down = (enabled == 0);
    uint8_t data = si5351c_compose_clk_ctrl(pwr_down, 1, 0, 0, 3, 1);
    fobos_si5351c_write_reg(dev, 20, data);
    return 0;
}
//==============================================================================
int fobos_si5351c_init(struct fobos_dev_t *dev)
{
    fobos_si5351c_write_reg(dev, 3, 0xFF);   // disable all outputs
    fobos_si5351c_write_reg(dev, 9, 0xFF);   // disable oeb pin control
    fobos_si5351c_write_reg(dev, 3, 0x00);   // enable all outputs
    fobos_si5351c_write_reg(dev, 15, 0x0C);  // clock source = CLKIN
    fobos_si5351c_write_reg(dev, 187, 0xC0); // Fanout Enable

    fobos_si5351c_write_reg(dev, 177, 0xA0); // reset plls

    uint8_t clk_ctrl_data[9];
    clk_ctrl_data[0] = 16;
    // #0 rffc507x_clk  prw up, int mode, plla, not inv, msx->clkx, 4ma
    clk_ctrl_data[1] = si5351c_compose_clk_ctrl(0, 1, 0, 0, 3, 1);
    clk_ctrl_data[2] = si5351c_compose_clk_ctrl(1, 1, 0, 0, 2, 0); // #1 pwr down
    clk_ctrl_data[3] = si5351c_compose_clk_ctrl(0, 1, 0, 0, 3, 0); // #2 ADC+
    clk_ctrl_data[4] = si5351c_compose_clk_ctrl(1, 1, 0, 0, 3, 0); // #3 ADC-
    clk_ctrl_data[5] = si5351c_compose_clk_ctrl(0, 1, 0, 0, 3, 1); // #4 MAX2830_CLK
    clk_ctrl_data[6] = si5351c_compose_clk_ctrl(1, 1, 0, 0, 2, 0); // #5
    clk_ctrl_data[7] = si5351c_compose_clk_ctrl(1, 1, 0, 0, 2, 0); // #6
    clk_ctrl_data[8] = si5351c_compose_clk_ctrl(1, 1, 0, 0, 2, 0); // #7

    fobos_si5351c_write(dev, clk_ctrl_data, sizeof(clk_ctrl_data));

    fobos_si5351c_config_pll(dev, 0, 80 * 128 - 512, 0, 1);

    // Configure rffc507x_clk
    fobos_si5351c_config_msynth(dev, 0, 20 * 128 - 512, 0, 1, 0); // 40 MHz
    dev->rffc507x_clock = 40000000ULL;

    // Configure max2830_clk
    fobos_si5351c_config_msynth(dev, 4, 20 * 128 - 512, 0, 1, 0); // 40 MHz
    dev->max2830_clock = 40000000.0;

#ifdef FOBOS_PRINT_DEBUG
    printf_internal("si5351c registers:\n");
    uint8_t addr = 0;
    uint8_t data[32];
    fobos_si5351c_write(dev, &addr, 1);
    fobos_si5351c_read(dev, data, 32);
    for (int i = 0; i < 32; ++i)
    {
        addr = i;
        printf_internal("[%d]=0x%02x\n", addr, data[i]);
    }
#endif // FOBOS_PRINT_DEBUG
    return FOBOS_ERR_OK;
}
//==============================================================================
int fobos_fx3_command(struct fobos_dev_t *dev, uint8_t code, uint16_t value, uint16_t index)
{
    int result = fobos_check(dev);
    if (result != FOBOS_ERR_OK)
    {
        return result;
    }
    result = libusb_control_transfer(dev->libusb_devh, CTRLO, code, value, index, 0, 0, CTRL_TIMEOUT);
    return result;
}
//==============================================================================
int fobos_rx_set_user_gpo(struct fobos_dev_t *dev, uint8_t value)
{
#ifdef FOBOS_PRINT_DEBUG
    printf_internal("%s(0x%04x);\n", __FUNCTION__, value);
#endif // FOBOS_PRINT_DEBUG
    return fobos_fx3_command(dev, 0xE3, value, 0);
}
//==============================================================================
int fobos_rx_set_dev_gpo(struct fobos_dev_t *dev, uint16_t value)
{
#ifdef FOBOS_PRINT_DEBUG
    printf_internal("%s(0x%04x);\n", __FUNCTION__, value);
#endif // FOBOS_PRINT_DEBUG
    return fobos_fx3_command(dev, 0xE4, value, 0);
}
//==============================================================================
int fobos_rx_open(struct fobos_dev_t **out_dev, uint32_t index)
{
    int result = 0;
    int i = 0;
    struct fobos_dev_t *dev = NULL;
    libusb_device **dev_list;
    libusb_device *device = NULL;
    ssize_t cnt;
    uint32_t device_count = 0;
    struct libusb_device_descriptor dd;
    dev = (struct fobos_dev_t *)malloc(sizeof(struct fobos_dev_t));
    if (NULL == dev)
    {
        return FOBOS_ERR_NO_MEM;
    }
    memset(dev, 0, sizeof(struct fobos_dev_t));
    result = libusb_init(&dev->libusb_ctx);
    if (result < 0)
    {
        free(dev);
        return result;
    }
    cnt = libusb_get_device_list(dev->libusb_ctx, &dev_list);
    for (i = 0; i < cnt; i++)
    {
        libusb_get_device_descriptor(dev_list[i], &dd);
        if ((dd.idVendor == FOBOS_VENDOR_ID) && (dd.idProduct == FOBOS_PRODUCT_ID) && (dd.bcdDevice == FOBOS_DEV_ID))
        {
            if (index == device_count)
            {
                device = dev_list[i];
                break;
            }
            device_count++;
        }
    }
    if (device)
    {
        result = libusb_open(device, &dev->libusb_devh);
        if (result == 0)
        {
            libusb_get_string_descriptor_ascii(dev->libusb_devh, dd.iSerialNumber, (unsigned char *)dev->serial,
                                               sizeof(dev->serial));
            libusb_get_string_descriptor_ascii(dev->libusb_devh, dd.iManufacturer, (unsigned char *)dev->manufacturer,
                                               sizeof(dev->manufacturer));
            libusb_get_string_descriptor_ascii(dev->libusb_devh, dd.iProduct, (unsigned char *)dev->product,
                                               sizeof(dev->product));
            result = libusb_claim_interface(dev->libusb_devh, 0);
            if (result == 0)
            {
                *out_dev = dev;
                //======================================================================
                result = libusb_control_transfer(dev->libusb_devh, CTRLI, 0xE8, 0, 0, (unsigned char *)dev->hw_revision,
                                                 sizeof(dev->hw_revision), CTRL_TIMEOUT);
                if (result <= 0)
                {
                    strcpy(dev->hw_revision, "2.0.0");
                }
                result = libusb_control_transfer(dev->libusb_devh, CTRLI, 0xE8, 1, 0, (unsigned char *)dev->fw_version,
                                                 sizeof(dev->fw_version), CTRL_TIMEOUT);
                if (result <= 0)
                {
                    strcpy(dev->hw_revision, "2.0.0");
                }
                result = libusb_control_transfer(dev->libusb_devh, CTRLI, 0xE8, 2, 0, (unsigned char *)dev->fw_build,
                                                 sizeof(dev->fw_build), CTRL_TIMEOUT);
                if (result <= 0)
                {
                    strcpy(dev->fw_build, "unknown");
                }
                //======================================================================
                dev->rx_frequency_band = FREQUENCY_BAND_UNSET;
                dev->dev_gpo = 0;
                dev->rx_dc_re = 0.f;
                dev->rx_dc_im = 0.f;
                dev->rx_avg_re = 0.0f;
                dev->rx_avg_im = 0.0f;
                dev->rx_calibration_state = CALIBRATION_DONE;
                if (fobos_check(dev) == 0)
                {
                    bitset(dev->dev_gpo, FOBOS_DEV_CLKSEL);
                    bitset(dev->dev_gpo, FOBOS_DEV_LNA_LP_SHD);
                    bitset(dev->dev_gpo, FOBOS_DEV_LNA_HP_SHD);
                    bitset(dev->dev_gpo, FOBOS_DEV_ADC_NCS);
                    bitset(dev->dev_gpo, FOBOS_DEV_ADC_SCK);
                    bitset(dev->dev_gpo, FOBOS_DEV_ADC_SDI);
                    bitset(dev->dev_gpo, FOBOS_DEV_NENBL_HF);
                    fobos_rx_set_dev_gpo(dev, dev->dev_gpo);
                    fobos_si5351c_init(dev);
                    fobos_max2830_init(dev);
                    fobos_rffc507x_init(dev);
                    fobos_rx_set_frequency(dev, DEFAULT_FREQUENCY, 0);
                    fobos_rx_set_samplerate(dev, DEFAULT_SAMPLERATE, 0);
                    return FOBOS_ERR_OK;
                }
            }
            else
            {
                printf_internal("usb_claim_interface error %d\n", result);
            }
        }
        else
        {
            printf_internal("usb_open error %d\n", result);
#ifndef _WIN32
            if (result == LIBUSB_ERROR_ACCESS)
            {
                printf_internal("Please fix the device permissions by installing fobos-sdr.rules\n");
            }
#endif
        }
    }
    libusb_free_device_list(dev_list, 1);
    if (dev->libusb_devh)
    {
        libusb_close(dev->libusb_devh);
    }
    if (dev->libusb_ctx)
    {
        libusb_exit(dev->libusb_ctx);
    }
    free(dev);
    return FOBOS_ERR_NO_DEV;
}
//==============================================================================
int fobos_rx_close(struct fobos_dev_t *dev)
{
    int result = fobos_check(dev);
#ifdef FOBOS_PRINT_DEBUG
    printf_internal("%s();\n", __FUNCTION__);
#endif // FOBOS_PRINT_DEBUG
    if (result != FOBOS_ERR_OK)
    {
        return result;
    }
    fobos_rx_cancel_async(dev);
    fobos_rx_stop_sync(dev);
    while (FOBOS_IDLE != dev->rx_async_status)
    {
#ifdef FOBOS_PRINT_DEBUG
        printf_internal("s");
#endif
#ifdef _WIN32
        Sleep(1);
#else
        sleep(1);
#endif
    }
    fobos_fx3_command(dev, 0xE1, 0, 0); // stop fx
    bitset(dev->dev_gpo, FOBOS_DEV_ADC_SDI);
    bitclear(dev->dev_gpo, FOBOS_DEV_LPF_A0);
    bitclear(dev->dev_gpo, FOBOS_DEV_LPF_A1);
    bitset(dev->dev_gpo, FOBOS_DEV_NENBL_HF);
    fobos_rx_set_dev_gpo(dev, dev->dev_gpo);
    // disable rffc507x
    fobos_rffc507x_register_modify(&dev->rffc507x_registers_local[0x15], 14, 14, 0); // enbl = 0
    fobos_rffc507x_commit(dev, 0);
    // disable clocks
    fobos_rffc507x_clock(dev, 0);
    fobos_max2830_clock(dev, 0);
    if (dev->do_reset)
    {
        libusb_control_transfer(dev->libusb_devh, CTRLO, 0xE0, 0, 0, 0, 0, CTRL_TIMEOUT);
    }
    libusb_close(dev->libusb_devh);
    libusb_exit(dev->libusb_ctx);
    free(dev);
    return result;
}
//==============================================================================
int fobos_rx_reset(struct fobos_dev_t *dev)
{
    int result = fobos_check(dev);
#ifdef FOBOS_PRINT_DEBUG
    printf_internal("%s();\n", __FUNCTION__);
#endif // FOBOS_PRINT_DEBUG
    if (result != FOBOS_ERR_OK)
    {
        return result;
    }
    dev->do_reset = 1;
    return fobos_rx_close(dev);
}
//==============================================================================
int fobos_rx_get_board_info(struct fobos_dev_t *dev, char *hw_revision, char *fw_version, char *manufacturer,
                            char *product, char *serial)
{
#ifdef FOBOS_PRINT_DEBUG
    printf_internal("%s();\n", __FUNCTION__);
#endif // FOBOS_PRINT_DEBUG
    int result = fobos_check(dev);
    if (result != FOBOS_ERR_OK)
    {
        return result;
    }
    if (hw_revision)
    {
        strcpy(hw_revision, dev->hw_revision);
    }
    if (fw_version)
    {
        strcpy(fw_version, dev->fw_version);
        strcat(fw_version, " ");
        strcat(fw_version, dev->fw_build);
    }
    if (manufacturer)
    {
        strcpy(manufacturer, dev->manufacturer);
    }
    if (product)
    {
        strcpy(product, dev->product);
    }
    if (serial)
    {
        strcpy(serial, dev->serial);
    }
    return result;
}
//==============================================================================
#define FOBOS_PRESELECT_BYPASS 0
#define FOBOS_PRESELECT_LOWPASS 1
#define FOBOS_PRESELECT_HIGHPASS 2
#define FOBOS_IF_FILTER_NONE 0
#define FOBOS_IF_FILTER_LOW 1
#define FOBOS_IF_FILTER_HIGH 2
#define FOBOS_IF_FREQ_AUTO 0
#define FOBOS_IF_FREQ_2350 2350
#define FOBOS_IF_FREQ_2400 2400
#define FOBOS_IF_FREQ_2450 2450
#define FOBOS_INJECT_NONE 0
#define FOBOS_INJECT_LOW 1
#define FOBOS_INJECT_HIGH 2
//==============================================================================
typedef struct
{
    uint32_t idx;
    uint32_t freq_mhz_min;
    uint32_t freq_mhz_max;
    uint32_t preselect;
    uint32_t if_filter;
    uint32_t if_freq_mhz;
    uint32_t rffc507x_enabled;
    uint32_t rffc507x_inject;
    uint32_t swap_iq;
} fobos_rx_band_param_t;
//==============================================================================
const fobos_rx_band_param_t fobos_rx_bands[] = {
    {
        .idx = 0,
        .freq_mhz_min = 50,
        .freq_mhz_max = 2200,
        .preselect = FOBOS_PRESELECT_LOWPASS,
        .if_filter = FOBOS_IF_FILTER_LOW,
        .if_freq_mhz = FOBOS_IF_FREQ_2350,
        .rffc507x_enabled = 1,
        .rffc507x_inject = FOBOS_INJECT_LOW,
        .swap_iq = 1,
    },
    {
        .idx = 1,
        .freq_mhz_min = 2200,
        .freq_mhz_max = 2300,
        .preselect = FOBOS_PRESELECT_LOWPASS,
        .if_filter = FOBOS_IF_FILTER_HIGH,
        .if_freq_mhz = FOBOS_IF_FREQ_2450,
        .rffc507x_enabled = 1,
        .rffc507x_inject = FOBOS_INJECT_LOW,
        .swap_iq = 1,
    },
    {
        .idx = 2,
        .freq_mhz_min = 2300,
        .freq_mhz_max = 2550,
        .preselect = FOBOS_PRESELECT_BYPASS,
        .if_filter = FOBOS_IF_FILTER_NONE,
        .if_freq_mhz = FOBOS_IF_FREQ_AUTO,
        .rffc507x_enabled = 0,
        .rffc507x_inject = FOBOS_INJECT_NONE,
        .swap_iq = 0,
    },
    {
        .idx = 3,
        .freq_mhz_min = 2550,
        .freq_mhz_max = 3000,
        .preselect = FOBOS_PRESELECT_HIGHPASS,
        .if_filter = FOBOS_IF_FILTER_LOW,
        .if_freq_mhz = FOBOS_IF_FREQ_2350,
        .rffc507x_enabled = 1,
        //.rffc507x_inject = FOBOS_INJECT_LOW,
        //.swap_iq = 1,
        .rffc507x_inject = FOBOS_INJECT_HIGH,
        .swap_iq = 0,
    },
    {
        .idx = 4,
        .freq_mhz_min = 3000,
        .freq_mhz_max = 3100,
        .preselect = FOBOS_PRESELECT_HIGHPASS,
        .if_filter = FOBOS_IF_FILTER_LOW,
        .if_freq_mhz = FOBOS_IF_FREQ_2350,
        .rffc507x_enabled = 1,
        .rffc507x_inject = FOBOS_INJECT_HIGH,
        .swap_iq = 0,
    },
    {
        .idx = 5,
        .freq_mhz_min = 3100,
        .freq_mhz_max = 3200,
        .preselect = FOBOS_PRESELECT_HIGHPASS,
        .if_filter = FOBOS_IF_FILTER_HIGH,
        .if_freq_mhz = FOBOS_IF_FREQ_2450,
        .rffc507x_enabled = 1,
        .rffc507x_inject = FOBOS_INJECT_HIGH,
        .swap_iq = 0,
    },
    {
        .idx = 6,
        .freq_mhz_min = 3200,
        .freq_mhz_max = 3400,
        .preselect = FOBOS_PRESELECT_HIGHPASS,
        .if_filter = FOBOS_IF_FILTER_LOW,
        .if_freq_mhz = FOBOS_IF_FREQ_2350,
        .rffc507x_enabled = 1,
        .rffc507x_inject = FOBOS_INJECT_HIGH,
        .swap_iq = 0,
    },
    {
        .idx = 7,
        .freq_mhz_min = 3400,
        .freq_mhz_max = 3600,
        .preselect = FOBOS_PRESELECT_HIGHPASS,
        .if_filter = FOBOS_IF_FILTER_HIGH,
        .if_freq_mhz = FOBOS_IF_FREQ_2450,
        .rffc507x_enabled = 1,
        .rffc507x_inject = FOBOS_INJECT_HIGH,
        .swap_iq = 0,
    },
    {
        .idx = 8,
        .freq_mhz_min = 3600,
        .freq_mhz_max = 4000,
        .preselect = FOBOS_PRESELECT_HIGHPASS,
        .if_filter = FOBOS_IF_FILTER_LOW,
        .if_freq_mhz = FOBOS_IF_FREQ_2350,
        .rffc507x_enabled = 1,
        .rffc507x_inject = FOBOS_INJECT_HIGH,
        .swap_iq = 0,
    },
    {
        .idx = 9,
        .freq_mhz_min = 4000,
        .freq_mhz_max = 4800,
        .preselect = FOBOS_PRESELECT_HIGHPASS,
        .if_filter = FOBOS_IF_FILTER_HIGH,
        .if_freq_mhz = FOBOS_IF_FREQ_2450,
        .rffc507x_enabled = 1,
        .rffc507x_inject = FOBOS_INJECT_HIGH,
        .swap_iq = 0,
    },
    {
        .idx = 10,
        .freq_mhz_min = 4800,
        .freq_mhz_max = 6900,
        .preselect = FOBOS_PRESELECT_HIGHPASS,
        .if_filter = FOBOS_IF_FILTER_LOW,
        .if_freq_mhz = FOBOS_IF_FREQ_2350,
        .rffc507x_enabled = 1,
        .rffc507x_inject = FOBOS_INJECT_HIGH,
        .swap_iq = 0,
    },
};
//==============================================================================
int fobos_rx_set_frequency(struct fobos_dev_t *dev, double value, double *actual)
{
    int result = fobos_check(dev);
#ifdef FOBOS_PRINT_DEBUG
    printf_internal("%s(%f);\n", __FUNCTION__, value);
#endif // FOBOS_PRINT_DEBUG
    if (result != FOBOS_ERR_OK)
    {
        return result;
    }
    if (dev->rx_frequency != value)
    {
        size_t count = sizeof(fobos_rx_bands) / sizeof(fobos_rx_bands[0]);
        uint32_t freq_mhz = (uint32_t)(value / 1E6 + 0.5);
        size_t idx = count;
        for (size_t i = 0; i < count; i++)
        {
            if ((freq_mhz >= fobos_rx_bands[i].freq_mhz_min) && (freq_mhz <= fobos_rx_bands[i].freq_mhz_max))
            {
                idx = i;
                break;
            }
        }
        if (idx == count)
        {
            return FOBOS_ERR_UNSUPPORTED;
        }
        if (dev->rx_frequency_band != idx)
        {
            switch (fobos_rx_bands[idx].preselect)
            {
            case FOBOS_PRESELECT_BYPASS: {
                bitclear(dev->dev_gpo, FOBOS_DEV_PRESEL_V1);
                bitclear(dev->dev_gpo, FOBOS_DEV_PRESEL_V2);
                bitclear(dev->dev_gpo, FOBOS_DEV_LNA_LP_SHD); // shut down both lnas
                bitclear(dev->dev_gpo, FOBOS_DEV_LNA_HP_SHD); // shut down both lnas
                break;
            }
            case FOBOS_PRESELECT_LOWPASS: {
                bitset(dev->dev_gpo, FOBOS_DEV_PRESEL_V1);
                bitclear(dev->dev_gpo, FOBOS_DEV_PRESEL_V2);
                bitclear(dev->dev_gpo, FOBOS_DEV_LNA_LP_SHD); // enable lowpass lna
                bitset(dev->dev_gpo, FOBOS_DEV_LNA_HP_SHD);   // shut down highpass lna
                break;
            }
            case FOBOS_PRESELECT_HIGHPASS: {
                bitclear(dev->dev_gpo, FOBOS_DEV_PRESEL_V1);
                bitset(dev->dev_gpo, FOBOS_DEV_PRESEL_V2);
                bitset(dev->dev_gpo, FOBOS_DEV_LNA_LP_SHD);   // shut down lowpass lna
                bitclear(dev->dev_gpo, FOBOS_DEV_LNA_HP_SHD); // enable highpass lna
                break;
            }
            }
            switch (fobos_rx_bands[idx].if_filter)
            {
            case FOBOS_IF_FILTER_NONE: {
                bitclear(dev->dev_gpo, FOBOS_DEV_IF_V1);
                bitclear(dev->dev_gpo, FOBOS_DEV_IF_V2);
                bitset(dev->dev_gpo, FOBOS_MAX2830_ANTSEL);
                break;
            }
            case FOBOS_IF_FILTER_LOW: {
                bitset(dev->dev_gpo, FOBOS_DEV_IF_V1);
                bitclear(dev->dev_gpo, FOBOS_DEV_IF_V2);
                bitclear(dev->dev_gpo, FOBOS_MAX2830_ANTSEL);
                break;
            }
            case FOBOS_IF_FILTER_HIGH: {
                bitclear(dev->dev_gpo, FOBOS_DEV_IF_V1);
                bitset(dev->dev_gpo, FOBOS_DEV_IF_V2);
                bitclear(dev->dev_gpo, FOBOS_MAX2830_ANTSEL);
                break;
            }
            }
            fobos_rx_set_dev_gpo(dev, dev->dev_gpo); // commit dev_gpo value
            fobos_rffc507x_clock(dev, fobos_rx_bands[idx].rffc507x_enabled);
            fobos_rffc507x_register_modify(&dev->rffc507x_registers_local[0x15], 14, 14,
                                           fobos_rx_bands[idx].rffc507x_enabled);
            fobos_rffc507x_commit(dev, 0);
            dev->rx_frequency_band = idx;
        }
        dev->rx_swap_iq = fobos_rx_bands[idx].swap_iq;

        double max2830_freq = FREQUENCY_UNSET;
        double max2830_freq_actual = FREQUENCY_UNSET;
        uint64_t RFFC5071_freq;
        uint64_t RFFC5071_freq_hz_actual;
        double rx_frequency = FREQUENCY_UNSET;
        switch (fobos_rx_bands[idx].rffc507x_inject)
        {
        case FOBOS_INJECT_NONE: {
            max2830_freq = value;
            fobos_max2830_set_frequency(dev, max2830_freq, &max2830_freq_actual);
            rx_frequency = max2830_freq_actual;
            result = FOBOS_ERR_OK;
            break;
        }
        case FOBOS_INJECT_LOW: {
            max2830_freq = fobos_rx_bands[idx].if_freq_mhz * 1E6;
            fobos_max2830_set_frequency(dev, max2830_freq, &max2830_freq_actual);
            RFFC5071_freq = (uint64_t)max2830_freq_actual + (uint64_t)value;
            fobos_rffc507x_set_lo_frequency_hz(dev, RFFC5071_freq, &RFFC5071_freq_hz_actual);
            rx_frequency = RFFC5071_freq_hz_actual - max2830_freq_actual;
            result = FOBOS_ERR_OK;
            break;
        }
        case FOBOS_INJECT_HIGH: {
            max2830_freq = fobos_rx_bands[idx].if_freq_mhz * 1E6;
            fobos_max2830_set_frequency(dev, max2830_freq, &max2830_freq_actual);
            RFFC5071_freq = (uint64_t)value - (uint64_t)max2830_freq_actual;
            fobos_rffc507x_set_lo_frequency_hz(dev, RFFC5071_freq, &RFFC5071_freq_hz_actual);
            rx_frequency = RFFC5071_freq_hz_actual + max2830_freq_actual;
            result = FOBOS_ERR_OK;
            break;
        }
        }
        if (result == FOBOS_ERR_OK)
        {
            dev->max2830_lo_frequency = max2830_freq_actual;
            dev->rx_frequency = rx_frequency;
            if (actual)
            {
                *actual = rx_frequency;
            }
        }
    }
    return result;
}
//==============================================================================
int fobos_rx_set_direct_sampling(struct fobos_dev_t *dev, unsigned int enabled)
{
#ifdef FOBOS_PRINT_DEBUG
    printf_internal("%s(%d);\n", __FUNCTION__, enabled);
#endif // FOBOS_PRINT_DEBUG
    int result = fobos_check(dev);
    if (result != FOBOS_ERR_OK)
    {
        return result;
    }
    if (dev->rx_direct_sampling != enabled)
    {
        if (enabled)
        {
            if (dev->hw_revision[0] == '2')
            {
                bitset(dev->dev_gpo, FOBOS_DEV_LPF_A0);
                bitset(dev->dev_gpo, FOBOS_DEV_LPF_A1);
            }
            else
            {
                bitclear(dev->dev_gpo, FOBOS_DEV_LPF_A0);
                bitclear(dev->dev_gpo, FOBOS_DEV_LPF_A1);
            }
            bitclear(dev->dev_gpo, FOBOS_DEV_NENBL_HF);
            fobos_rx_set_dev_gpo(dev, dev->dev_gpo);
            // disable clocks
            fobos_rffc507x_clock(dev, 0);
            fobos_max2830_clock(dev, 0);
            // disable rffc507x
            fobos_rffc507x_register_modify(&dev->rffc507x_registers_local[0x15], 14, 14, 0); // enbl = 0
            fobos_rffc507x_commit(dev, 0);
        }
        else
        {
            bitclear(dev->dev_gpo, FOBOS_DEV_LPF_A0);
            bitclear(dev->dev_gpo, FOBOS_DEV_LPF_A1);
            if (dev->rx_lpf_idx > 2)
            {
                dev->rx_lpf_idx = 2;
            }
            if (dev->hw_revision[0] == '2')
            {
                if (dev->rx_lpf_idx & 1)
                {
                    bitset(dev->dev_gpo, FOBOS_DEV_LPF_A1);
                }
                if (dev->rx_lpf_idx & 2)
                {
                    bitset(dev->dev_gpo, FOBOS_DEV_LPF_A0);
                }
            }
            else
            {
                bitset(dev->dev_gpo, FOBOS_DEV_LPF_A0);
                bitset(dev->dev_gpo, FOBOS_DEV_LPF_A1);
            }
            bitset(dev->dev_gpo, FOBOS_DEV_NENBL_HF);
            fobos_rx_set_dev_gpo(dev, dev->dev_gpo);
            // enable clocks
            fobos_rffc507x_clock(dev, 1);
            fobos_max2830_clock(dev, 1);
            // enable rffc507x
            fobos_rffc507x_register_modify(&dev->rffc507x_registers_local[0x15], 14, 14, 1); // enbl = 1
            fobos_rffc507x_commit(dev, 0);
        }
        dev->rx_direct_sampling = enabled;
    }
    return result;
}
//==============================================================================
int fobos_rx_set_lna_gain(struct fobos_dev_t *dev, unsigned int value)
{
#ifdef FOBOS_PRINT_DEBUG
    printf_internal("%s(%d)\n", __FUNCTION__, value);
#endif // FOBOS_PRINT_DEBUG
    int result = fobos_check(dev);
    if (result != FOBOS_ERR_OK)
    {
        return result;
    }
    if (value > 3)
        value = 3;
    if (value != dev->rx_lna_gain)
    {
        dev->rx_lna_gain = value;
        int lna_gain = dev->rx_lna_gain & 0x0003;
        int vga_gain = dev->rx_vga_gain & 0x001F;
        fobos_max2830_write_reg(dev, 11, (lna_gain << 5) | vga_gain);
    }
    return result;
}
//==============================================================================
int fobos_rx_set_vga_gain(struct fobos_dev_t *dev, unsigned int value)
{
#ifdef FOBOS_PRINT_DEBUG
    printf_internal("%s(%d)\n", __FUNCTION__, value);
#endif // FOBOS_PRINT_DEBUG
    int result = fobos_check(dev);
    if (result != FOBOS_ERR_OK)
    {
        return result;
    }
    result = 0;
    if (value > 31)
        value = 31;
    if (value != dev->rx_vga_gain)
    {
        dev->rx_vga_gain = value;
        int lna_gain = dev->rx_lna_gain & 0x0003;
        int vga_gain = dev->rx_vga_gain & 0x001F;
        fobos_max2830_write_reg(dev, 11, (lna_gain << 5) | vga_gain);
    }
    return result;
}
//==============================================================================
int fobos_rx_set_lpf(struct fobos_dev_t *dev, double bandwidth)
{
    int result = fobos_check(dev);
#ifdef FOBOS_PRINT_DEBUG
    printf_internal("%s(%f)\n", __FUNCTION__, bandwidth);
#endif // FOBOS_PRINT_DEBUG
    if (result != FOBOS_ERR_OK)
    {
        return result;
    }
    result = 0;
    int rx_lpf_idx = dev->rx_lpf_idx;
    if (!dev->rx_direct_sampling)
    {
        bitclear(dev->dev_gpo, FOBOS_DEV_LPF_A0);
        bitclear(dev->dev_gpo, FOBOS_DEV_LPF_A1);
        if (dev->hw_revision[0] == '2')
        {
            if (bandwidth < 13000000.0)
            {
                rx_lpf_idx = 0;
            }
            else if (bandwidth < 26000000.0)
            {
                rx_lpf_idx = 1;
            }
            else
            {
                rx_lpf_idx = 2;
            }
            if (rx_lpf_idx & 1)
            {
                bitset(dev->dev_gpo, FOBOS_DEV_LPF_A1);
            }
            if (rx_lpf_idx & 2)
            {
                bitset(dev->dev_gpo, FOBOS_DEV_LPF_A0);
            }
        }
        else
        {
            bitset(dev->dev_gpo, FOBOS_DEV_LPF_A0);
            bitset(dev->dev_gpo, FOBOS_DEV_LPF_A1);
        }
        fobos_rx_set_dev_gpo(dev, dev->dev_gpo);
    }
    dev->rx_lpf_idx = rx_lpf_idx;
    return result;
}
//==============================================================================
const double fobos_max2830_bws[] = {2.0 * 7.5E6, 2.0 * 8.5E6, 2.0 * 15.0E6, 2.0 * 18.0E6};
//==============================================================================
const double fobos_max2830_adj[] = {0.90, 0.95, 1.00, 1.05, 1.10};
//==============================================================================
int fobos_rx_set_bandwidth(struct fobos_dev_t *dev, double value, double *actual)
{
    int result = fobos_check(dev);
#ifdef FOBOS_PRINT_DEBUG
    printf_internal("%s(%f)\n", __FUNCTION__, value);
#endif // FOBOS_PRINT_DEBUG
    if (result != FOBOS_ERR_OK)
    {
        return result;
    }
    result = 0;
    uint32_t p1 = 0;
    size_t count = sizeof(fobos_max2830_bws) / sizeof(fobos_max2830_bws[0]);
    double dmin = value;
    size_t idx = 0;
    for (size_t i = 0; i < count; i++)
    {
        double df = fabs(value - fobos_max2830_bws[i]);
        if (df < dmin)
        {
            dmin = df;
            idx = i;
        }
    }
    count = sizeof(fobos_max2830_adj) / sizeof(fobos_max2830_adj[0]);
    dmin = value;
    size_t adj = 0;
    for (size_t i = 0; i < count; i++)
    {
        double bw = fobos_max2830_bws[idx] * fobos_max2830_adj[i];
        double df = fabs(value - bw);
        if (df < dmin)
        {
            dmin = df;
            adj = i;
        }
    }
    if (actual)
    {
        *actual = fobos_max2830_bws[idx] * fobos_max2830_adj[adj];
    }
    if (dev->rx_bw_idx != idx)
    {
        dev->rx_bw_idx = idx;
        fobos_max2830_write_reg(dev, 8, idx | 0x3020);
    }
    if (dev->rx_bw_adj != adj)
    {
        dev->rx_bw_adj = adj;
        fobos_max2830_write_reg(dev, 7, adj | 0x1020);
    }
    // printf("idx = %d, adj = %d\n", idx, adj);
    return result;
}
//==============================================================================
const double fobos_sample_rates[] = {80000000.0, 50000000.0, 40000000.0, 32000000.0, 25000000.0,
                                     20000000.0, 16000000.0, 12500000.0, 10000000.0, 8000000.0};
//==============================================================================
int fobos_rx_get_samplerates(struct fobos_dev_t *dev, double *values, unsigned int *count)
{
    int result = fobos_check(dev);
    if (result != FOBOS_ERR_OK)
    {
        return result;
    }
    result = 0;
    if (count)
    {
        *count = sizeof(fobos_sample_rates) / sizeof(fobos_sample_rates[0]);
        if (values)
        {
            memcpy(values, fobos_sample_rates, sizeof(fobos_sample_rates));
        }
    }
    return result;
}
//==============================================================================
const uint32_t fobos_p1s[] = {10, 16, 20, 25, 32, 40, 50, 64, 80, 100};
//==============================================================================
int fobos_rx_set_samplerate(struct fobos_dev_t *dev, double value, double *actual)
{
    int result = fobos_check(dev);
#ifdef FOBOS_PRINT_DEBUG
    printf_internal("%s(%f)\n", __FUNCTION__, value);
#endif // FOBOS_PRINT_DEBUG
    if (result != FOBOS_ERR_OK)
    {
        return result;
    }
    result = 0;
    uint32_t p1 = 0;
    size_t count = sizeof(fobos_sample_rates) / sizeof(fobos_sample_rates[0]);
    double df_min = fobos_sample_rates[0];
    size_t i_min = 0;
    for (size_t i = 0; i < count; i++)
    {
        double df = fabs(value - fobos_sample_rates[i]);
        if (df < df_min)
        {
            df_min = df;
            i_min = i;
        }
    }
    p1 = fobos_p1s[i_min] * 128 - 512;
    fobos_si5351c_config_msynth(dev, 2, p1, 0, 1, 0);
    fobos_si5351c_config_msynth(dev, 3, p1, 0, 1, 0);
    value = fobos_sample_rates[i_min];
    if (result == 0)
    {
        double bandwidth = value * 0.8;
        fobos_rx_set_lpf(dev, bandwidth);
        fobos_rx_set_bandwidth(dev, bandwidth, 0);
        dev->rx_samplerate = value;
        if (actual)
        {
            *actual = dev->rx_samplerate;
        }
    }
    return result;
}
//==============================================================================
int fobos_rx_set_clk_source(struct fobos_dev_t *dev, int value)
{
    int result = fobos_check(dev);
#ifdef FOBOS_PRINT_DEBUG
    printf_internal("%s(%d)\n", __FUNCTION__, value);
#endif // FOBOS_PRINT_DEBUG
    if (result != FOBOS_ERR_OK)
    {
        return result;
    }
    if (value)
    {
        bitclear(dev->dev_gpo, FOBOS_DEV_CLKSEL);
    }
    else
    {
        bitset(dev->dev_gpo, FOBOS_DEV_CLKSEL);
    }
    result = fobos_rx_set_dev_gpo(dev, dev->dev_gpo);
    return result;
}
//==============================================================================
// This calibration routine is based on
// S. W. Ellingson "Correcting I-Q Imbalance in Direct Conversion Receivers",
// http://argus.naapo.org/~rchilders/swe_argus_pubs/iqbal.pdf
void fobos_rx_dc_offset_calibration(struct fobos_dev_t *dev, void *restrict data, uint32_t size)
{
#ifdef FOBOS_PRINT_DEBUG
    printf_internal("%s(%d)\n", __FUNCTION__, size);
#endif // FOBOS_PRINT_DEBUG
    size_t complex_samples_count = size / 4U;
    int16_t *restrict psample = (int16_t *)data;

    if (dev->rx_calibration_pos >= CALIBRATION_NUM_STEPS)
    {
        return;
    }

    if (dev->rx_calibration_pos == 0U)
    {
        dev->summ_re = 0;
        dev->summ_im = 0;
        dev->num_calibration_samples = 0U;
    }
    for (size_t i = 0U; i < complex_samples_count; i++)
    {
        int16_t const re = to_signed(psample[0]);
        int16_t const im = to_signed(psample[1]);
        dev->summ_re += re;
        dev->summ_im += im;
        psample += 2;
    }

    dev->num_calibration_samples += complex_samples_count;

    if (dev->rx_calibration_pos == (CALIBRATION_NUM_STEPS - 1))
    {
        dev->rx_dc_re = (float)dev->summ_re / (float)dev->num_calibration_samples;
        dev->rx_dc_im = (float)dev->summ_im / (float)dev->num_calibration_samples;
#ifdef FOBOS_PRINT_DEBUG
        printf_internal("DC OFFSET: re = %f, img = %f\n", dev->rx_dc_re, dev->rx_dc_im);
#endif // FOBOS_PRINT_DEBUG
    }

    ++dev->rx_calibration_pos;
}

void fobos_rx_iq_balance_calibration(struct fobos_dev_t *dev, void *restrict data, uint32_t size)
{
#ifdef FOBOS_PRINT_DEBUG
    printf_internal("%s(%d)\n", __FUNCTION__, size);
#endif // FOBOS_PRINT_DEBUG
    size_t complex_samples_count = size / 4U;
    int16_t *restrict psample = (int16_t *)data;

    if (dev->rx_calibration_pos >= CALIBRATION_NUM_STEPS)
    {
        return;
    }

    if (dev->rx_calibration_pos == 0U)
    {
        dev->re_re = 0.0;
        dev->re_im = 0.0;
        dev->im_im = 0.0;
        dev->num_calibration_samples = 0U;
    }

    float max_re = -FLT_MAX;
    float max_im = -FLT_MAX;

    for (size_t i = 0U; i < complex_samples_count; i++)
    {
        float re = (float)to_signed(psample[0]);
        float im = (float)to_signed(psample[1]);

        max_re = fmaxf(re, max_re);
        max_im = fmaxf(im, max_im);

        re = (re - dev->rx_dc_re) * SAMPLE_NORM;
        im = (im - dev->rx_dc_im) * SAMPLE_NORM;

        dev->re_re += (re * re);
        dev->re_im += (re * im);
        dev->im_im += (im * im);

        psample += 2;
    }

    dev->num_calibration_samples += complex_samples_count;

    if (dev->rx_calibration_pos == (CALIBRATION_NUM_STEPS - 1))
    {
        double const avg_re_re = dev->re_re / (double)dev->num_calibration_samples;
        double const avg_re_im = dev->re_im / (double)dev->num_calibration_samples;
        double const avg_im_im = dev->im_im / (double)dev->num_calibration_samples;

        double const re_amp = sqrt(2.0) * sqrt(avg_re_re);
        double const im_amp = sqrt(2.0) * sqrt(avg_im_im);

        double const sin_phi = (2.0 / (re_amp * im_amp)) * avg_re_im;
        double const cos_phi = sqrt(1.0 - sin_phi * sin_phi);

        double const inv_alpha = sqrt(avg_im_im / avg_re_re);
        double const sec_phi = 1.0 / cos_phi;
        double const tan_phi = sin_phi / cos_phi;

        dev->rx_calibration_a11 = inv_alpha;
        dev->rx_calibration_a21 = -inv_alpha * tan_phi;
        dev->rx_calibration_a22 = sec_phi;

#ifdef FOBOS_PRINT_DEBUG
        printf_internal("IQ BALANCE: max re = %f, max im = %f\n", max_re, max_im);
        printf_internal("IQ BALANCE: sin_phi = %f\n", sin_phi);
        printf_internal("IQ BALANCE: a11 = %f, a21 = %f, a22 = %f, re_amp = %f, im_amp = %f\n", dev->rx_calibration_a11,
                        dev->rx_calibration_a21, dev->rx_calibration_a22, re_amp, im_amp);
#endif // FOBOS_PRINT_DEBUG
    }

    ++dev->rx_calibration_pos;
}
//==============================================================================
void fobos_rx_calibration_sanity_check(struct fobos_dev_t *dev, void *restrict data, uint32_t size)
{
#ifdef FOBOS_PRINT_DEBUG
    printf_internal("%s(%d)\n", __FUNCTION__, size);
    size_t complex_samples_count = size / 4U;
    int16_t *restrict psample = (int16_t *)data;

    if (dev->rx_calibration_pos >= CALIBRATION_NUM_STEPS)
    {
        return;
    }

    if (dev->rx_calibration_pos == 0U)
    {
        dev->re_re = 0.0;
        dev->re_im = 0.0;
        dev->im_im = 0.0;
        dev->summ_re = 0;
        dev->summ_im = 0;
        dev->num_calibration_samples = 0U;
    }

    float max_re = -FLT_MAX;
    float max_im = -FLT_MAX;

    float const dc_re = dev->rx_dc_re;
    float const dc_im = dev->rx_dc_im;

    float re = 0.0F;
    float im = 0.0F;

    float const calibration_a11 = dev->rx_calibration_a11;
    float const calibration_a21 = dev->rx_calibration_a21;
    float const calibration_a22 = dev->rx_calibration_a22;

    for (size_t i = 0U; i < complex_samples_count; i++)
    {
        float re = (float)to_signed(psample[0]) - dc_re;
        float im = (float)to_signed(psample[1]) - dc_im;

        dev->summ_re += (int64_t)re;
        dev->summ_im += (int64_t)im;

        re *= SAMPLE_NORM;
        im *= SAMPLE_NORM;

        re = calibration_a11 * re;
        im = calibration_a21 * re + calibration_a22 * im;

        max_re = fmaxf(re, max_re);
        max_im = fmaxf(im, max_im);

        dev->re_re += (re * re);
        dev->re_im += (re * im);
        dev->im_im += (im * im);

        psample += 2;
    }

    dev->num_calibration_samples += complex_samples_count;

    if (dev->rx_calibration_pos == (CALIBRATION_NUM_STEPS - 1))
    {

        float const dc_re_ = (float)dev->summ_re / (float)dev->num_calibration_samples;
        float const dc_im_ = (float)dev->summ_im / (float)dev->num_calibration_samples;

        dev->rx_dc_re += dc_re_;
        dev->rx_dc_im += dc_im_;

        double const avg_re_re = dev->re_re / (double)dev->num_calibration_samples;
        double const avg_re_im = dev->re_im / (double)dev->num_calibration_samples;
        double const avg_im_im = dev->im_im / (double)dev->num_calibration_samples;

        double const re_amp = sqrt(2.0) * sqrt(avg_re_re);
        double const im_amp = sqrt(2.0) * sqrt(avg_im_im);

        double const sin_phi = (2.0 / (re_amp * im_amp)) * avg_re_im;
        double const cos_phi = sqrt(1.0 - sin_phi * sin_phi);

        double const inv_alpha = sqrt(avg_im_im / avg_re_re);
        double const sec_phi = 1.0 / cos_phi;
        double const tan_phi = sin_phi / cos_phi;

        float const a11 = inv_alpha;
        float const a21 = -inv_alpha * tan_phi;
        float const a22 = sec_phi;

        if (sin_phi < CALIBRATION_SUCCESS_PHASE_DIFFERENCE_RAD)
        {
            printf_internal("IQ phase difference calibration SUCCEEDED.\n");
        }
        else
        {
            printf_internal("IQ phase difference calibration FAILED.\n");
        }
        if (fabsf(a11 - 1.0f) < CALIBRATION_SUCCESS_GAIN_RATIO)
        {
            printf_internal("IQ gain ratio calibration SUCCEEDED.\n");
        }
        else
        {
            printf_internal("IQ gain ratio calibration FAILED.\n");
        }
        printf_internal("DC OFFSET: re = %f, img = %f\n", dc_re_, dc_im_);
        printf_internal("IQ BALANCE: max re = %f, max im = %f\n", max_re, max_im);
        printf_internal("IQ BALANCE: sin_phi = %f\n", sin_phi);
        printf_internal("IQ BALANCE: a11 = %f, a21 = %f, a22 = %f, re_amp = %f, im_amp = %f\n", a11, a21, a22, re_amp,
                        im_amp);
    }
#endif // FOBOS_PRINT_DEBUG
    ++dev->rx_calibration_pos;
}
//==============================================================================
int fobos_rx_set_calibration(struct fobos_dev_t *dev, enum fobos_calibration_state state)
{
    int result = fobos_check(dev);
    double f = FREQUENCY_UNSET;
    if (result != FOBOS_ERR_OK)
    {
        return result;
    }
    if (dev->rx_calibration_state != state)
    {
        switch (state)
        {
        case CALIBRATION_START:

            dev->_rx_lna_gain = dev->rx_lna_gain;
            dev->_rx_vga_gain = dev->rx_vga_gain;

            if (dev->rx_direct_sampling)
            {
                // turn on lpf
                bitclear(dev->dev_gpo, FOBOS_DEV_LPF_A0);
                bitclear(dev->dev_gpo, FOBOS_DEV_LPF_A1);
                if (dev->rx_lpf_idx & 1)
                {
                    bitset(dev->dev_gpo, FOBOS_DEV_LPF_A1);
                }
                if (dev->rx_lpf_idx & 2)
                {
                    bitset(dev->dev_gpo, FOBOS_DEV_LPF_A0);
                }
                fobos_rx_set_dev_gpo(dev, dev->dev_gpo);
                // enable clocks
                fobos_rffc507x_clock(dev, 1);
                fobos_max2830_clock(dev, 1);
                // enable rffc507x
                fobos_rffc507x_register_modify(&dev->rffc507x_registers_local[0x15], 14, 14, 1); // enbl = 1
                fobos_rffc507x_commit(dev, 0);
            }
            // bypass preselector: shut down both LNAs and filters
            bitclear(dev->dev_gpo, FOBOS_DEV_LNA_LP_SHD);
            bitclear(dev->dev_gpo, FOBOS_DEV_LNA_HP_SHD);
            bitclear(dev->dev_gpo, FOBOS_DEV_PRESEL_V1);
            bitclear(dev->dev_gpo, FOBOS_DEV_PRESEL_V2);
            // set_if_filter(low);
            bitset(dev->dev_gpo, FOBOS_DEV_IF_V1);
            bitclear(dev->dev_gpo, FOBOS_DEV_IF_V2);
            // turn on max2830 ant1 (main) input, turn off ant2 (aux) input
            bitclear(dev->dev_gpo, FOBOS_MAX2830_ANTSEL);
            // commit dev_gpo value
            fobos_rx_set_dev_gpo(dev, dev->dev_gpo);
            // set frequency direct to max2830
            if (dev->max2830_lo_frequency == FREQUENCY_UNSET)
            {
                fobos_max2830_set_frequency(dev, (double)CALIBRATION_MAX2830_LO_FREQUENCY, &dev->max2830_lo_frequency);
            }
            uint64_t const rffc507x_lo_frequency =
                (uint64_t)dev->max2830_lo_frequency + CALIBRATION_RFFC5072_LO_FREQUENCY_OFFSET;
            // set frequency direct rffc507x
            fobos_rffc507x_set_lo_frequency_hz(dev, rffc507x_lo_frequency, 0);

            dev->rx_calibration_pos = 0;
            dev->rx_calibration_state = CALIBRATION_START;
            break;
        case CALIBRATION_DC_OFFSET: // "remove dc" calibration state
            if (!CALIBRATION_DEBUG_GAINS)
            {
                // set max2830 lna gain
                fobos_rx_set_lna_gain(dev, CALIBRATION_DC_OFFSET_MAX2830_LNA_GAIN);
                // set max2830 vga gain
                fobos_rx_set_vga_gain(dev, CALIBRATION_DC_OFFSET_MAX2830_VGA_GAIN);
            }
            dev->rx_calibration_pos = 0;
            dev->rx_calibration_state = CALIBRATION_DC_OFFSET;
            break;
        case CALIBRATION_IQ_BALANCE:
            if (!CALIBRATION_DEBUG_GAINS)
            {
                // set max2830 lna gain
                fobos_rx_set_lna_gain(dev, CALIBRATION_IQ_CALIBRATION_MAX2830_LNA_GAIN);
                // set max2830 vga gain
                fobos_rx_set_vga_gain(dev, CALIBRATION_IQ_CALIBRATION_MAX2830_VGA_GAIN);
            }
            dev->rx_calibration_pos = 0;
            dev->rx_calibration_state = CALIBRATION_IQ_BALANCE;
            break;
        case CALIBRATION_SANITY_CHECK:
            dev->rx_calibration_pos = 0;
            dev->rx_calibration_state = CALIBRATION_SANITY_CHECK;
            break;
        case CALIBRATION_DONE:
            if (!CALIBRATION_DEBUG_SIGNAL)
            {
                f = dev->rx_frequency;
                dev->rx_frequency = FREQUENCY_UNSET;
                dev->rx_frequency_band = FREQUENCY_BAND_UNSET;
                fobos_rx_set_frequency(dev, f, 0);
                if (dev->rx_direct_sampling)
                {
                    dev->rx_direct_sampling = 0;
                    fobos_rx_set_direct_sampling(dev, 1);
                }
            }
            if (!CALIBRATION_DEBUG_GAINS)
            {
                fobos_rx_set_lna_gain(dev, dev->_rx_lna_gain);
                fobos_rx_set_vga_gain(dev, dev->_rx_vga_gain);
            }

            dev->rx_calibration_pos = 0;
            dev->rx_calibration_state = CALIBRATION_DONE;
            break;
        }
    }
    return result;
}
//==============================================================================
void fobos_rx_calibration_controller(struct fobos_dev_t *dev)
{
    if (dev->rx_calibration_state == CALIBRATION_START)
    {
        fobos_rx_set_calibration(dev, CALIBRATION_DC_OFFSET);
    }
    else if (dev->rx_calibration_state == CALIBRATION_DC_OFFSET && dev->rx_calibration_pos >= CALIBRATION_NUM_STEPS)
    {
        fobos_rx_set_calibration(dev, CALIBRATION_IQ_BALANCE);
    }
    else if (dev->rx_calibration_state == CALIBRATION_IQ_BALANCE && dev->rx_calibration_pos >= CALIBRATION_NUM_STEPS)
    {
        fobos_rx_set_calibration(dev, CALIBRATION_SANITY_CHECK);
    }
    else if (dev->rx_calibration_state == CALIBRATION_SANITY_CHECK && dev->rx_calibration_pos >= CALIBRATION_NUM_STEPS)
    {
        fobos_rx_set_calibration(dev, CALIBRATION_DONE);
    }
}
//==============================================================================
void fobos_rx_proceed_calibration(struct fobos_dev_t *dev, void *restrict data, uint32_t size)
{
    if (dev->rx_calibration_pos < CALIBRATION_NUM_STEPS)
    {
        if (dev->rx_calibration_state == CALIBRATION_DC_OFFSET)
        {
            fobos_rx_dc_offset_calibration(dev, data, size);
        }
        else if (dev->rx_calibration_state == CALIBRATION_IQ_BALANCE)
        {
            fobos_rx_iq_balance_calibration(dev, data, size);
        }
        else if (dev->rx_calibration_state == CALIBRATION_SANITY_CHECK)
        {
            fobos_rx_calibration_sanity_check(dev, data, size);
        }
    }
}
//==============================================================================
#define FOBOS_SWAP_IQ_HW 1
void fobos_rx_convert_samples(struct fobos_dev_t *dev, void *restrict data, size_t const size,
                              float *restrict dst_samples)
{
    size_t const complex_samples_count = size / 4;
    int16_t *restrict psample = (int16_t *)data;
    int rx_swap_iq = dev->rx_swap_iq ^ FOBOS_SWAP_IQ_HW;
    float sample = 0.0f;

    float const dc_re = dev->rx_dc_re;
    float const dc_im = dev->rx_dc_im;

    float re = 0.0F;
    float im = 0.0F;

    float const calibration_a11 = dev->rx_calibration_a11;
    float const calibration_a21 = dev->rx_calibration_a21;
    float const calibration_a22 = dev->rx_calibration_a22;

    if (dev->rx_direct_sampling)
    {
        rx_swap_iq = FOBOS_SWAP_IQ_HW;
    }
    size_t const chunks_count = complex_samples_count / 8;
    if (rx_swap_iq)
    {
        for (size_t i = 0; i < chunks_count; i++)
        {
            // 0
            re = ((float)to_signed(psample[0]) - dc_re) * SAMPLE_NORM;
            im = ((float)to_signed(psample[1]) - dc_im) * SAMPLE_NORM;

            dst_samples[0] = calibration_a21 * re + calibration_a22 * im;
            dst_samples[1] = calibration_a11 * re;

            // 1
            re = ((float)to_signed(psample[2]) - dc_re) * SAMPLE_NORM;
            im = ((float)to_signed(psample[3]) - dc_im) * SAMPLE_NORM;

            dst_samples[2] = calibration_a21 * re + calibration_a22 * im;
            dst_samples[3] = calibration_a11 * re;

            // 2
            re = ((float)to_signed(psample[4]) - dc_re) * SAMPLE_NORM;
            im = ((float)to_signed(psample[5]) - dc_im) * SAMPLE_NORM;

            dst_samples[4] = calibration_a21 * re + calibration_a22 * im;
            dst_samples[5] = calibration_a11 * re;

            // 3
            re = ((float)to_signed(psample[6]) - dc_re) * SAMPLE_NORM;
            im = ((float)to_signed(psample[7]) - dc_im) * SAMPLE_NORM;

            dst_samples[6] = calibration_a21 * re + calibration_a22 * im;
            dst_samples[7] = calibration_a11 * re;

            // 4
            re = ((float)to_signed(psample[8]) - dc_re) * SAMPLE_NORM;
            im = ((float)to_signed(psample[9]) - dc_im) * SAMPLE_NORM;

            dst_samples[8] = calibration_a21 * re + calibration_a22 * im;
            dst_samples[9] = calibration_a11 * re;

            // 5
            re = ((float)to_signed(psample[10]) - dc_re) * SAMPLE_NORM;
            im = ((float)to_signed(psample[11]) - dc_im) * SAMPLE_NORM;

            dst_samples[10] = calibration_a21 * re + calibration_a22 * im;
            dst_samples[11] = calibration_a11 * re;

            // 6
            re = ((float)to_signed(psample[12]) - dc_re) * SAMPLE_NORM;
            im = ((float)to_signed(psample[13]) - dc_im) * SAMPLE_NORM;

            dst_samples[12] = calibration_a21 * re + calibration_a22 * im;
            dst_samples[13] = calibration_a11 * re;

            // 7
            re = ((float)to_signed(psample[14]) - dc_re) * SAMPLE_NORM;
            im = ((float)to_signed(psample[15]) - dc_im) * SAMPLE_NORM;

            dst_samples[14] = calibration_a21 * re + calibration_a22 * im;
            dst_samples[15] = calibration_a11 * re;

            dst_samples += 16;
            psample += 16;
        }
    }
    else
    {
        for (size_t i = 0; i < chunks_count; i++)
        {
            // 0
            re = ((float)to_signed(psample[0]) - dc_re) * SAMPLE_NORM;
            im = ((float)to_signed(psample[1]) - dc_im) * SAMPLE_NORM;

            dst_samples[0] = calibration_a11 * re;
            dst_samples[1] = calibration_a21 * re + calibration_a22 * im;

            // 1
            re = ((float)to_signed(psample[2]) - dc_re) * SAMPLE_NORM;
            im = ((float)to_signed(psample[3]) - dc_im) * SAMPLE_NORM;

            dst_samples[2] = calibration_a11 * re;
            dst_samples[3] = calibration_a21 * re + calibration_a22 * im;

            // 2
            re = ((float)to_signed(psample[4]) - dc_re) * SAMPLE_NORM;
            im = ((float)to_signed(psample[5]) - dc_im) * SAMPLE_NORM;

            dst_samples[4] = calibration_a11 * re;
            dst_samples[5] = calibration_a21 * re + calibration_a22 * im;

            // 3
            re = ((float)to_signed(psample[6]) - dc_re) * SAMPLE_NORM;
            im = ((float)to_signed(psample[7]) - dc_im) * SAMPLE_NORM;

            dst_samples[6] = calibration_a11 * re;
            dst_samples[7] = calibration_a21 * re + calibration_a22 * im;

            // 4
            re = ((float)to_signed(psample[8]) - dc_re) * SAMPLE_NORM;
            im = ((float)to_signed(psample[9]) - dc_im) * SAMPLE_NORM;

            dst_samples[8] = calibration_a11 * re;
            dst_samples[9] = calibration_a21 * re + calibration_a22 * im;

            // 5
            re = ((float)to_signed(psample[10]) - dc_re) * SAMPLE_NORM;
            im = ((float)to_signed(psample[11]) - dc_im) * SAMPLE_NORM;

            dst_samples[10] = calibration_a11 * re;
            dst_samples[11] = calibration_a21 * re + calibration_a22 * im;

            // 6
            re = ((float)to_signed(psample[12]) - dc_re) * SAMPLE_NORM;
            im = ((float)to_signed(psample[13]) - dc_im) * SAMPLE_NORM;

            dst_samples[12] = calibration_a11 * re;
            dst_samples[13] = calibration_a21 * re + calibration_a22 * im;

            // 7
            re = ((float)to_signed(psample[14]) - dc_re) * SAMPLE_NORM;
            im = ((float)to_signed(psample[15]) - dc_im) * SAMPLE_NORM;

            dst_samples[14] = calibration_a11 * re;
            dst_samples[15] = calibration_a21 * re + calibration_a22 * im;

            dst_samples += 16;
            psample += 16;
        }
    }
}
//==============================================================================
int fobos_alloc_buffers(struct fobos_dev_t *dev)
{
    int result = fobos_check(dev);
    if (result != FOBOS_ERR_OK)
    {
        return result;
    }
    if (!dev->transfer)
    {
        dev->transfer = (struct libusb_transfer **)malloc(dev->transfer_buf_count * sizeof(struct libusb_transfer *));
        if (dev->transfer)
        {
            for (size_t i = 0; i < dev->transfer_buf_count; i++)
            {
                dev->transfer[i] = libusb_alloc_transfer(0);
            }
        }
    }
    if (dev->transfer_buf)
    {
        return FOBOS_ERR_NO_MEM;
    }
    dev->transfer_buf = (unsigned char **)malloc(dev->transfer_buf_count * sizeof(unsigned char *));
    if (dev->transfer_buf)
    {
        memset(dev->transfer_buf, 0, dev->transfer_buf_count * sizeof(unsigned char *));
    }
#if defined(ENABLE_ZEROCOPY) && defined(__linux__) && LIBUSB_API_VERSION >= 0x01000105
    printf_internal("Allocating %d zero-copy buffers\n", dev->transfer_buf_count);
    dev->use_zerocopy = 1;
    for (size_t i = 0; i < dev->transfer_buf_count; ++i)
    {
        dev->transfer_buf[i] = libusb_dev_mem_alloc(dev->libusb_devh, dev->transfer_buf_size);
        if (dev->transfer_buf[i])
        {
            if (dev->transfer_buf[i][0] ||
                memcmp(dev->transfer_buf[i], dev->transfer_buf[i] + 1, dev->transfer_buf_size - 1))
            {
                printf_internal("Kernel usbfs mmap() bug, falling back to buffers\n");
                dev->use_zerocopy = 0;
                break;
            }
        }
        else
        {
            printf_internal("Failed to allocate zero-copy buffer for transfer %d\n", i);
            dev->use_zerocopy = 0;
            break;
        }
    }
    if (!dev->use_zerocopy)
    {
        for (size_t i = 0; i < dev->transfer_buf_count; ++i)
        {
            if (dev->transfer_buf[i])
            {
                libusb_dev_mem_free(dev->libusb_devh, dev->transfer_buf[i], dev->transfer_buf_size);
            }
        }
    }
#endif
    if (!dev->use_zerocopy)
    {
        for (size_t i = 0; i < dev->transfer_buf_count; ++i)
        {
            dev->transfer_buf[i] = (unsigned char *)malloc(dev->transfer_buf_size);

            if (!dev->transfer_buf[i])
            {
                return FOBOS_ERR_NO_MEM;
            }
        }
    }
    return FOBOS_ERR_OK;
}
//==============================================================================
int fobos_free_buffers(struct fobos_dev_t *dev)
{
    int result = fobos_check(dev);
    if (result != 0)
    {
        return result;
    }
    if (dev->transfer)
    {
        for (size_t i = 0; i < dev->transfer_buf_count; ++i)
        {
            if (dev->transfer[i])
            {
                libusb_free_transfer(dev->transfer[i]);
            }
        }
        free(dev->transfer);
        dev->transfer = NULL;
    }
    if (dev->transfer_buf)
    {
        for (size_t i = 0; i < dev->transfer_buf_count; ++i)
        {
            if (dev->transfer_buf[i])
            {
                if (dev->use_zerocopy)
                {
#if defined(__linux__) && LIBUSB_API_VERSION >= 0x01000105
                    libusb_dev_mem_free(dev->libusb_devh, dev->transfer_buf[i], dev->transfer_buf_size);
#endif
                }
                else
                {
                    free(dev->transfer_buf[i]);
                }
            }
        }
        free(dev->transfer_buf);
        dev->transfer_buf = NULL;
    }
    return FOBOS_ERR_OK;
}
//==============================================================================
static void LIBUSB_CALL _libusb_callback(struct libusb_transfer *transfer)
{
    struct fobos_dev_t *dev = (struct fobos_dev_t *)transfer->user_data;
    if (LIBUSB_TRANSFER_COMPLETED == transfer->status)
    {
        if (transfer->actual_length == (int)dev->transfer_buf_size)
        {
            dev->rx_buff_counter++;

            if (dev->rx_calibration_state != CALIBRATION_DONE)
            {
                fobos_rx_proceed_calibration(dev, transfer->buffer, transfer->actual_length);
            }
            else
            {
                fobos_rx_convert_samples(dev, transfer->buffer, transfer->actual_length, dev->rx_buff);
                size_t complex_samples_count = transfer->actual_length / 4;
                if (dev->rx_cb)
                {
                    dev->rx_cb(dev->rx_buff, complex_samples_count, dev->rx_cb_ctx);
                }
            }
        }
        else
        {
            printf_internal("E");
            dev->rx_failures++;
        }
        libusb_submit_transfer(transfer);
        dev->transfer_errors = 0;
    }
    else if (LIBUSB_TRANSFER_CANCELLED != transfer->status)
    {
        printf_internal("transfer->status = %d\n", transfer->status);
#ifndef _WIN32
        if (LIBUSB_TRANSFER_ERROR == transfer->status)
        {
            dev->transfer_errors++;
        }
        if (dev->transfer_errors >= (int)dev->transfer_buf_count || LIBUSB_TRANSFER_NO_DEVICE == transfer->status)
        {
            dev->dev_lost = 1;
            fobos_rx_cancel_async(dev);
        }
#else
        dev->dev_lost = 1;
        fobos_rx_cancel_async(dev);
#endif
    }
}
//==============================================================================
int fobos_rx_read_async(struct fobos_dev_t *dev, fobos_rx_cb_t cb, void *ctx, uint32_t buf_count, uint32_t buf_length)
{
    int result = fobos_check(dev);
#ifdef FOBOS_PRINT_DEBUG
    printf_internal("%s(0x%08x, 0x%08x, 0x%08x, %d, %d)\n", __FUNCTION__, (unsigned int)dev, (unsigned int)cb,
                    (unsigned int)ctx, buf_count, buf_length);
#endif // FOBOS_PRINT_DEBUG
    if (result != FOBOS_ERR_OK)
    {
        return result;
    }
    if (FOBOS_IDLE != dev->rx_async_status)
    {
        return FOBOS_ERR_ASYNC_IN_SYNC;
    }
    result = FOBOS_ERR_OK;
    struct timeval tv0 = {0, 0};
    struct timeval tv1 = {1, 0};
    struct timeval tvx = {0, 10000};
    dev->rx_async_status = FOBOS_STARTING;
    dev->rx_async_cancel = 0;
    dev->rx_buff_counter = 0;
    dev->rx_cb = cb;
    dev->rx_cb_ctx = ctx;
    dev->rx_avg_re = 0.0f;
    dev->rx_avg_im = 0.0f;
    fobos_rx_set_calibration(dev, CALIBRATION_START);
    if (buf_count == 0)
    {
        buf_count = FOBOS_DEF_BUF_COUNT;
    }
    if (buf_count > FOBOS_MAX_BUF_COUNT)
    {
        buf_count = FOBOS_MAX_BUF_COUNT;
    }
    dev->transfer_buf_count = buf_count;

    if (buf_length == 0)
    {
        buf_length = FOBOS_DEF_BUF_LENGTH;
    }
    buf_length = 128 * (buf_length / 128); // complex samples count

    uint32_t transfer_buf_size = buf_length * 4;         // raw int16 buff size
    transfer_buf_size = 512 * (transfer_buf_size / 512); // len must be multiple of 512

    dev->transfer_buf_size = transfer_buf_size;

    result = fobos_alloc_buffers(dev);
    if (result != FOBOS_ERR_OK)
    {
        return result;
    }

    dev->rx_buff = (float *)malloc(buf_length * 2 * sizeof(float));

    fobos_fx3_command(dev, 0xE1, 1, 0); // start fx

    bitclear(dev->dev_gpo, FOBOS_DEV_ADC_SDI);
    fobos_rx_set_dev_gpo(dev, dev->dev_gpo);

    for (size_t i = 0; i < dev->transfer_buf_count; ++i)
    {
        libusb_fill_bulk_transfer(dev->transfer[i], dev->libusb_devh, LIBUSB_BULK_IN_ENDPOINT, dev->transfer_buf[i],
                                  dev->transfer_buf_size, _libusb_callback, (void *)dev, LIBUSB_BULK_TIMEOUT);

        result = libusb_submit_transfer(dev->transfer[i]);
        if (result < 0)
        {
            printf_internal("Failed to submit transfer #%i, err %i\n", i, result);
            dev->rx_async_status = FOBOS_CANCELING;
            break;
        }
    }

    dev->rx_async_status = FOBOS_RUNNING;
    while (FOBOS_IDLE != dev->rx_async_status)
    {
        fobos_rx_calibration_controller(dev);
        result = libusb_handle_events_timeout_completed(dev->libusb_ctx, &tv1, &dev->rx_async_cancel);
        if (result < 0)
        {
            printf_internal("libusb_handle_events_timeout_completed returned: %d\n", result);
            if (result == LIBUSB_ERROR_INTERRUPTED)
            {
                continue;
            }
            else
            {
                break;
            }
        }
        if (FOBOS_CANCELING == dev->rx_async_status)
        {
            printf_internal("FOBOS_CANCELING \n");
            dev->rx_async_status = FOBOS_IDLE;
            if (!dev->transfer)
            {
                break;
            }
            for (size_t i = 0; i < dev->transfer_buf_count; ++i)
            {
                // printf_internal(" ~%d", i);
                if (!dev->transfer[i])
                    continue;
                if (LIBUSB_TRANSFER_CANCELLED != dev->transfer[i]->status)
                {
                    struct libusb_transfer *xf = dev->transfer[i];
                    printf_internal(" ~%08x", xf->flags);

                    result = libusb_cancel_transfer(dev->transfer[i]);
                    libusb_handle_events_timeout_completed(dev->libusb_ctx, &tvx, NULL);
                    if (result < 0)
                    {
                        printf_internal("libusb_cancel_transfer[%d] returned: %d %s\n", i, result,
                                        libusb_error_name(result));
                        continue;
                    }
                    dev->rx_async_status = FOBOS_CANCELING;
                }
            }
            if (dev->dev_lost || FOBOS_IDLE == dev->rx_async_status)
            {
                libusb_handle_events_timeout_completed(dev->libusb_ctx, &tvx, NULL);
                break;
            }
        }
    }
    fobos_fx3_command(dev, 0xE1, 0, 0); // stop fx
    fobos_free_buffers(dev);
    free(dev->rx_buff);
    dev->rx_buff = NULL;
    bitset(dev->dev_gpo, FOBOS_DEV_ADC_SDI);
    fobos_rx_set_dev_gpo(dev, dev->dev_gpo);
    dev->rx_async_status = FOBOS_IDLE;
    dev->rx_async_cancel = 0;
    return result;
}
//==============================================================================
int fobos_rx_cancel_async(struct fobos_dev_t *dev)
{
    int result = fobos_check(dev);
#ifdef FOBOS_PRINT_DEBUG
    printf_internal("%s()\n", __FUNCTION__);
#endif // FOBOS_PRINT_DEBUG
    if (result != FOBOS_ERR_OK)
    {
        return result;
    }
    if (FOBOS_RUNNING == dev->rx_async_status)
    {
        dev->rx_async_status = FOBOS_CANCELING;
        dev->rx_async_cancel = 1;
    }
    return 0;
}
//==============================================================================
int fobos_rx_start_sync(struct fobos_dev_t *dev, uint32_t buf_length)
{
    size_t i = 0U;
    int actual = 0;
    int result = fobos_check(dev);
#ifdef FOBOS_PRINT_DEBUG
    printf_internal("%s()\n", __FUNCTION__);
#endif // FOBOS_PRINT_DEBUG
    if (result != FOBOS_ERR_OK)
    {
        return result;
    }
    if (FOBOS_IDLE != dev->rx_async_status)
    {
        return FOBOS_ERR_SYNC_IN_ASYNC;
    }
    if (dev->rx_sync_started)
    {
        return 0;
    }
    if (buf_length == 0)
    {
        buf_length = FOBOS_DEF_BUF_LENGTH;
    }
    buf_length = 128 * (buf_length / 128);
    dev->rx_buff = (float *)malloc(buf_length * 2 * sizeof(float));
    dev->transfer_buf_size = buf_length * 4;
    dev->rx_sync_buf = (unsigned char *)malloc(dev->transfer_buf_size);
    if (dev->rx_sync_buf == 0)
    {
        dev->transfer_buf_size = 0;
        return FOBOS_ERR_NO_MEM;
    }
    fobos_fx3_command(dev, 0xE1, 1, 0); // start fx
    bitclear(dev->dev_gpo, FOBOS_DEV_ADC_SDI);
    fobos_rx_set_dev_gpo(dev, dev->dev_gpo);
    fobos_rx_set_calibration(dev, CALIBRATION_START);
    while (dev->rx_calibration_state != CALIBRATION_DONE)
    {
        fobos_rx_calibration_controller(dev);

        result = libusb_bulk_transfer(dev->libusb_devh, LIBUSB_BULK_IN_ENDPOINT, dev->rx_sync_buf,
                                      dev->transfer_buf_size, &actual, LIBUSB_BULK_TIMEOUT);
        if (result == 0)
        {
            fobos_rx_proceed_calibration(dev, dev->rx_sync_buf, dev->transfer_buf_size);
        }
        else
        {
            fobos_rx_set_calibration(dev, CALIBRATION_DONE);
            break;
        }
    }
    dev->rx_sync_started = 1;
    return FOBOS_ERR_OK;
}
//==============================================================================
int fobos_rx_read_sync(struct fobos_dev_t *dev, float *buf, uint32_t *actual_buf_length)
{
    int actual = 0;
    int result = fobos_check(dev);
#ifdef FOBOS_PRINT_DEBUG
    printf_internal("%s()\n", __FUNCTION__);
#endif // FOBOS_PRINT_DEBUG
    if (result != FOBOS_ERR_OK)
    {
        return result;
    }
    if (dev->rx_sync_started != 1)
    {
        return FOBOS_ERR_SYNC_NOT_STARTED;
    }
    result = libusb_bulk_transfer(dev->libusb_devh, LIBUSB_BULK_IN_ENDPOINT, dev->rx_sync_buf, dev->transfer_buf_size,
                                  &actual, LIBUSB_BULK_TIMEOUT);
    if (result == FOBOS_ERR_OK)
    {
        fobos_rx_convert_samples(dev, dev->rx_sync_buf, actual, buf);
        if (actual_buf_length)
        {
            *actual_buf_length = actual / 4;
        }
    }
    return result;
}
//==============================================================================
int fobos_rx_stop_sync(struct fobos_dev_t *dev)
{
    int result = fobos_check(dev);
#ifdef FOBOS_PRINT_DEBUG
    printf_internal("%s()\n", __FUNCTION__);
#endif // FOBOS_PRINT_DEBUG
    if (result != FOBOS_ERR_OK)
    {
        return result;
    }
    result = 0;
    if (dev->rx_sync_started)
    {
        bitset(dev->dev_gpo, FOBOS_DEV_ADC_SDI);
        fobos_rx_set_dev_gpo(dev, dev->dev_gpo);
        free(dev->rx_sync_buf);
        dev->rx_sync_buf = NULL;
        free(dev->rx_buff);
        dev->rx_buff = NULL;
        dev->rx_sync_started = 0;
    }
    return result;
}
//==============================================================================
int fobos_rx_write_firmware(struct fobos_dev_t *dev, const char *file_name, int verbose)
{
    int result = fobos_check(dev);
#ifdef FOBOS_PRINT_DEBUG
    printf_internal("%s(%s)\n", __FUNCTION__, file_name);
#endif // FOBOS_PRINT_DEBUG
    if (result != FOBOS_ERR_OK)
    {
        return result;
    }
    if (dev->rx_sync_started || dev->rx_async_status != FOBOS_IDDLE)
    {
        return FOBOS_ERR_UNSUPPORTED;
    }
    FILE *f = fopen(file_name, "rb");
    if (f == 0)
    {
        return FOBOS_ERR_UNSUPPORTED;
    }
    fseek(f, 0, SEEK_END);
    size_t file_size = ftell(f);
    if ((file_size == 0) || (file_size > 0x3FFE0))
    {
        fclose(f);
        return FOBOS_ERR_UNSUPPORTED;
    }
    result = 0;
    size_t xx_size = 1024;
    size_t xx_count = (file_size + xx_size - 1) / xx_size;
    uint8_t *file_data = (uint8_t *)malloc(xx_count * xx_size);
    fseek(f, 0, SEEK_SET);
    fread(file_data, file_size, 1, f);
    fclose(f);
    memset(file_data + file_size, 0, xx_count * xx_size - file_size);
    uint16_t xsize;
    uint8_t req_code = 0xED;
    uint8_t *xx_data = file_data;
    if (verbose)
    {
        printf_internal("writing a firmware");
    }
    for (size_t i = 0; i < xx_count; i++)
    {
        if (verbose)
        {
            printf_internal(".");
        }
        xsize = libusb_control_transfer(dev->libusb_devh, CTRLO, req_code, 0, i, xx_data, xx_size, CTRL_TIMEOUT);
        if (xsize != xx_size)
        {
            result = FOBOS_ERR_CONTROL;
        }
        xx_data += xx_size;
    }
    if (verbose)
    {
        printf_internal("done.\n");
    }
    free(file_data);
    return result;
}
//==============================================================================
int fobos_rx_read_firmware(struct fobos_dev_t *dev, const char *file_name, int verbose)
{
    int result = fobos_check(dev);
#ifdef FOBOS_PRINT_DEBUG
    printf_internal("%s(%s)\n", __FUNCTION__, file_name);
#endif // FOBOS_PRINT_DEBUG
    if (result != FOBOS_ERR_OK)
    {
        return result;
    }
    if (dev->rx_sync_started || dev->rx_async_status != FOBOS_IDDLE)
    {
        return FOBOS_ERR_UNSUPPORTED;
    }
    FILE *f = fopen(file_name, "wb");
    if (f == 0)
    {
        return FOBOS_ERR_UNSUPPORTED;
    }
    result = 0;
    size_t xx_size = 1024;
    size_t xx_count = 130;
    uint8_t *xx_data = (uint8_t *)malloc(xx_size);
    uint16_t xsize;
    uint8_t req_code = 0xEC;
    if (verbose)
    {
        printf_internal("reading a firmware");
    }
    for (size_t i = 0; i < xx_count; i++)
    {
        if (verbose)
        {
            printf_internal(".");
        }
        xsize = libusb_control_transfer(dev->libusb_devh, CTRLI, req_code, 0, i, xx_data, xx_size, CTRL_TIMEOUT);
        if (xsize != xx_size)
        {
            result = FOBOS_ERR_CONTROL;
        }
        fwrite(xx_data, xx_size, 1, f);
    }
    fclose(f);
    if (verbose)
    {
        printf_internal("done.\n");
    }
    free(xx_data);
    return result;
}
//==============================================================================
const char *fobos_rx_error_name(int error)
{
    switch (error)
    {
    case FOBOS_ERR_OK:
        return "Ok";
    case FOBOS_ERR_NO_DEV:
        return "No device spesified, dev == NUL";
    case FOBOS_ERR_NOT_OPEN:
        return "Device is not open, please use fobos_rx_open() first";
    case FOBOS_ERR_NO_MEM:
        return "Memory allocation error";
    case FOBOS_ERR_ASYNC_IN_SYNC:
        return "Could not read in async mode while sync mode started";
    case FOBOS_ERR_SYNC_IN_ASYNC:
        return "Could not start sync mode while async reading";
    case FOBOS_ERR_SYNC_NOT_STARTED:
        return "Sync mode is not started";
    case FOBOS_ERR_UNSUPPORTED:
        return "Unsuppotred parameter or mode";
    case FOBOS_ERR_LIBUSB:
        return "libusb error";
    default:
        return "Unknown error";
    }
}
//==============================================================================
