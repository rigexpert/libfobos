//==============================================================================
//  Fobos SDR API library test application
//  V.T.
//  LGPL-2.1+
//  2024.03.21
//  2024.04.08
//  2024.05.01
//  2026.06.12 - FOBOS_INFO_LEN
//==============================================================================
#include <stdio.h>
#include <string.h>
#include <fobos.h>
//==============================================================================
void get_devinfo()
{
    struct fobos_dev_t * dev = NULL;
    int result = 0;
    char lib_version[FOBOS_INFO_LEN];
    char drv_version[FOBOS_INFO_LEN];
    char serials[256] = {0};

    int index = 0;

    char hw_revision[FOBOS_INFO_LEN];
    char fw_version[FOBOS_INFO_LEN];
    char manufacturer[FOBOS_INFO_LEN];
    char product[FOBOS_INFO_LEN];
    char serial[FOBOS_INFO_LEN];
    
    fobos_rx_get_api_info(lib_version, drv_version);

    printf("API Info lib: %s drv: %s\n", lib_version, drv_version);

    int count = fobos_rx_list_devices(serials);
    char* pserial = strtok(serials, " ");

    printf("found devices: %d\n", count);

    if (count > 0)
    {
        for (index = 0; index < count; index++)
        {
            printf("  dev# %i  %s\n", index, pserial);
            pserial = strtok(0, " ");

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
                    printf("    hw_revision:  %s\n", hw_revision);
                    printf("    fw_version:   %s\n", fw_version);
                    printf("    manufacturer: %s\n", manufacturer);
                    printf("    product:      %s\n", product);
                    printf("    serial:       %s\n", serial);
                }
                fobos_rx_close(dev);
            }
            else
            {
                printf("    could not open device\n");
            }
        }
    }
}
//==============================================================================
int main(int argc, char** argv)
{
    printf("Fobos SDR get device info test app\n");
    get_devinfo();
    return 0;
}
//==============================================================================
