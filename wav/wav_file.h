//==============================================================================
// wav file r/w
// pure c implementation
// 2024.02.10
// V.V.
//==============================================================================
#ifndef __WAV_FILE_H__
#define __WAV_FILE_H__
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#ifndef MAX_PATH
#define MAX_PATH 260
#endif // !MAX_PATH

struct wav_file_t
{
    FILE *file;
    char file_name[MAX_PATH];
    char file_mode[3];
    int is_valid;
    size_t file_size;
    uint32_t audio_format;
    uint32_t channels_count;
    uint32_t sample_rate;
    uint32_t bytes_per_second;
    uint32_t bytes_per_sample;
    uint32_t bytes_per_sample_group;
    uint32_t samples_count;
    uint32_t sample_groups_count;
    uint32_t data_start;
};

struct wav_file_t * wav_file_create(void);
void wav_file_destroy(struct wav_file_t * ctx);
int wav_file_open(struct wav_file_t * ctx, const char * file_name, const char * mode);
int wav_file_close(struct wav_file_t * ctx);
int wav_file_write_header(struct wav_file_t * ctx);
int wav_file_read_header(struct wav_file_t * ctx);
size_t wav_file_write_data(struct wav_file_t * ctx, void * data, size_t size);
size_t wav_file_read_data(struct wav_file_t * ctx, void * data, size_t size);
//int wav_file_write_samples(struct wav_file_t * ctx, const char * data, size_t size);
//int wav_file_read_samples(struct wav_file_t * ctx, const char * data, size_t size);
size_t wav_file_samples_to_data(struct wav_file_t * ctx, const float * samples, size_t count, char * data, size_t * size);
size_t wav_file_data_to_samples(struct wav_file_t * ctx, const char * data, size_t size, float * samples, size_t * count);
#endif // __WAV_FILE_H__
