//==============================================================================
//  Fobos SDR (agile) API library test application
//  fast scan mode
//  V.T.
//  LGPL-2.1+
//  2024.12.07
//  2026.03.12
//==============================================================================
#include <stdio.h>
#ifdef _WIN32
#include <Windows.h>
#endif
#include <string.h>
//#include <fobos_sdr.h>
#include <fobos/fobos.h>
#include <wav/wav_file.h>
//==============================================================================
typedef struct scan_ctx_t
{
    struct wav_file_t * wavs[FOBOS_MAX_FREQS_CNT];
    int channels_count;
    int split_channels;
    int skip_tuning;
    int buff_count;
    int max_buff_count;
    struct fobos_dev_t *dev;
} rx_ctx_t;
//==============================================================================
void read_samples_callback(float *buf, uint32_t buf_length, void *user)
{
    struct scan_ctx_t * scan = (struct scan_ctx_t *)user;
    scan->buff_count++;

    if (scan->buff_count % 256 == 0)
    {
        printf("+");
        fflush(stdout);
    }
    
    if (scan->buff_count >= scan->max_buff_count)
    {
        printf("canceling...");
        fobos_rx_cancel_async(scan->dev);
    }

    // check if the scaning mode is enabled
    if (fobos_rx_is_scanning(scan->dev) != 1)
    {
        printf("scanning mode is not enabled or was internaly dropped. canceling...");
        fobos_rx_cancel_async(scan->dev);
        return;
    }

    struct wav_file_t* wav = 0;
    // get the current scan channel index
    int channel = fobos_rx_get_scan_index(scan->dev);

    if (scan->split_channels)
    {
        if (channel == -1)
        {
            // device is tuning, skip this buf
        }
        else
        {
            // select the scan->wavs[channel] to write to
            wav = scan->wavs[channel];
        }
    }
    else
    {
        if ((channel == -1) && (scan->skip_tuning))
        {
            // device is tuning skip this buf
        }
        else
        {
            // select the one file
            wav = scan->wavs[0];
        }
    }

    if (wav)
    {
        // actually write ...
        wav_file_write_data(wav, (void*)buf, (size_t)buf_length * 2 * sizeof(float));
        wav_file_write_header(wav);
    }
}
//==============================================================================
//==============================================================================
void test_scanner(void)
{
    struct fobos_dev_t* dev = NULL;
    int result = 0;
    char lib_version[64];
    char drv_version[64];
    char serials[256] = {0};

    int index = 0; // the device index to open

    char hw_revision[64];
    char fw_version[64];
    char manufacturer[64];
    char product[64];
    char serial[64];

    fobos_rx_get_api_info(lib_version, drv_version);

    printf("API Info lib: %s drv: %s\n", lib_version, drv_version);

    int count = fobos_rx_get_device_count(); // just gets devices count

    printf("found devices: %d\n", count);

    count = fobos_rx_list_devices(serials); // enumerates all connected devices with serial numbers

    if (count > 0)
    {
        char* pserial = strtok(serials, " ");
        for (int i = 0; i < count; i++)
        {
            printf("   sn: %s\n", pserial);
            pserial = strtok(0, " ");
        }

        result = fobos_rx_open(&dev, index);

        if (result == 0)
        {
            result = fobos_rx_get_board_info(dev, hw_revision, fw_version, manufacturer, product, serial);
            if (result != 0)
            {
                printf("fobos_rx_get_board_info - error!\n");
            }
            else
            {
                printf("board info\n");
                printf("    hw_revision:  %s\n", hw_revision);
                printf("    fw_version:   %s\n", fw_version);
                printf("    manufacturer: %s\n", manufacturer);
                printf("    product:      %s\n", product);
                printf("    serial:       %s\n", serial);
            }

            printf("test: streaming\n");
            double frequency = 100000000.0;   // Hz
            double samplerate = 25000000.0;   // samples per second
            //double samplerate = 50000000.0;   // samples per second
            //double samplerate = 64000000.0;   // samples per second
            double auto_bandwidth = 0.8;      // bandwidth relative to sample rate
            unsigned int direct_sampling = 0; // 0 - RF, do not use 1 - HF1+HF2 when scanning
            unsigned int lna_gain = 0;        // low noise amplifier 0..2
            unsigned int vga_gain = 0;        // variable gain amplifier 0..15
            unsigned int clk_source = 0;      // 0 - internal clock (default); 1 - external

            result = fobos_rx_set_frequency(dev, frequency, NULL);
            if (result != 0)
            {
                printf("fobos_rx_set_frequency - error!\n");
            }

            result = fobos_rx_set_direct_sampling(dev, direct_sampling);
            if (result != 0)
            {
                printf("fobos_rx_set_direct_sampling - error!\n");
            }

            result = fobos_rx_set_lna_gain(dev, lna_gain);
            if (result != 0)
            {
                printf("fobos_rx_set_lna_gain - error!\n");
            }

            result = fobos_rx_set_vga_gain(dev, vga_gain);
            if (result != 0)
            {
                printf("fobos_rx_set_vga_gain - error!\n");
            }
            result = fobos_rx_set_samplerate(dev, samplerate, NULL);
            if (result != 0)
            {
                printf("fobos_rx_set_samplerate - error!\n");
            }

            result = fobos_rx_set_auto_bandwidth(dev, auto_bandwidth);
            if (result != 0)
            {
                printf("fobos_rx_set_auto_bandwidth - error!\n");
            }

            result = fobos_rx_set_clk_source(dev, clk_source);
            if (result != 0)
            {
                printf("fobos_rx_set_clk_source - error!\n");
            }


            //=================================================================//
            //                                                                 //
            //             actually scanning setup is here                     //
            //                                                                 //
            //=================================================================//

            // define the freqs list to scan
            //  - double, Hz 
            //  - one Hz precision
            //  - max 256 items
            //  - any order
            //  - any step
            double freqs[] =      // hardcoded for now
            {                     
                100000000.0,
                110000000.0,
                120000000.0,
                130000000.0
            };

            int freqs_count = 4;  // hardcoded for now

            // specify the amount of complex samples to be captured per one step
            //  - integer
            //  - even multiple of 8192
            //  - use at least 65536
            int samples_per_step = 65536;


            // activate the scanning mode
            //  - call it here, before streaming, or anywhere else
            result = fobos_rx_start_scan(dev, freqs, freqs_count);
            if (result != FOBOS_ERR_OK)
            {
                printf("could not activate the scanning mode\n");
            }
                
            // some scanning context 
            // - depending on the user case
            // - this code is for a simple example
            struct scan_ctx_t scan = { 0 };
            scan.buff_count = 0;
            scan.max_buff_count = 1024 * 8; // number of buffers to record
            scan.channels_count = freqs_count;
            scan.skip_tuning = 0;       // do not record while receiver being tunning
            scan.split_channels = 0;    // record each channel into separate wav file
            scan.dev = dev;

            if (scan.split_channels)
            {
                for (int i = 0; i < scan.channels_count; i++)
                {
                    struct wav_file_t* wav = wav_file_create();
                    wav->channels_count = 2;
                    wav->sample_rate = (int)samplerate;
                    wav->bytes_per_sample = 4;   // record in native float32 format
                    wav->audio_format = 3;

                    char file_name[256];
                    sprintf(file_name, "scan.ch%03d.iq.wav", i);
                    //sprintf(file_name, "f:/rff/scan.ch%03d.iq.wav", i);

                    result = wav_file_open(wav, file_name, "w");
                    if (result != 0)
                    {
                        printf("could not create file %s\n", file_name);
                    }
                    scan.wavs[i] = wav;
                }
            }
            else
            {
                struct wav_file_t* wav = wav_file_create();
                wav->channels_count = 2;
                wav->sample_rate = (int)samplerate;
                wav->bytes_per_sample = 4;   // record in native float32 format
                wav->audio_format = 3;

                const char* file_name = "scan.iq.wav";
                //const char* file_name = "f:/rff/scan.iq.wav";

                result = wav_file_open(wav, file_name, "w");
                if (result != 0)
                {
                    printf("could not create file %s\n", file_name);
                }
                scan.wavs[0] = wav;
            }

            // start the streaming in the usual way
            //  - async mode -> see read_samples_callback() implementation
            result = fobos_rx_read_async(dev, read_samples_callback, &scan, 16, samples_per_step);
            if (result == 0)
            {
                printf("fobos_rx_read_async - ok!\n");
            }
            else
            {
                printf("fobos_rx_read_async - error!\n");
            }
            //  - or use sync mode:
            //    fobos_rx_start_sync()
            //    fobos_rx_read_sync()
            //    fobos_rx_stop_sync()

            // disable the scanning mode
            // here or else where
            result = fobos_rx_stop_scan(dev);

            //=================================================================//

            for (int i = 0; i < scan.channels_count; i++)
            {
                wav_file_destroy(scan.wavs[i]);
            }

            fobos_rx_close(dev);
        }
        else
        {
            printf("could not open device! err (%i)\n", result);
        }
    }
    else
    {
        printf("no Fobos SDR (agile) compatible devices found!\n");
    }
}
//==============================================================================
int main(int argc, char** argv)
{
    printf("Fobos SDR (agile) API scanner test applications\n");
    for(int i = 0; i < argc; i++)
    {
        printf("arg[%d]=%s\n", i, argv[i]);
    }
    printf("machine: x%ld\n", sizeof(void*)*8);

    test_scanner();

#ifdef _WIN32
    system("pause");
#endif

    return 0;
}
//==============================================================================
