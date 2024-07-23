//==============================================================================
// wav file r/w
// pure c implementation
// 2024.02.10
// V.V.
//==============================================================================
#include <string.h>
#include "wav_file.h"
//==============================================================================
struct wav_file_t * wav_file_create(void)
{
    struct wav_file_t * result = (struct wav_file_t *)malloc(sizeof(struct wav_file_t));
    memset(result, 0, sizeof(struct wav_file_t));
    result->audio_format = 1;
    result->channels_count = 2;
    result->bytes_per_sample = 2;
    return result;
}
//==============================================================================
void wav_file_destroy(struct wav_file_t * ctx)
{
	if (ctx)
    {
		wav_file_close(ctx);
        free(ctx);
    }
}
//==============================================================================
int wav_file_open(struct wav_file_t * ctx, const char * file_name, const char * mode)
{
    if (ctx->file)
    {
        if (strcmp(file_name, ctx->file_name) == 0)
        {
            return 0;
        }
        wav_file_close(ctx);
    }
    strcpy(ctx->file_name, file_name);
    ctx->file_mode[0] = 'r';
    ctx->file_mode[1] = 'b';
    ctx->file_mode[2] = 0;
    if (mode)
    {
        if ((mode[0] == 'w') || (mode[0] == 'W'))
        {
            ctx->file_mode[0] = 'w';
        }
    }
    ctx->is_valid = 0;
    ctx->file = fopen(ctx->file_name, ctx->file_mode);
    if (ctx->file)
    {
        if (ctx->file_mode[0] == 'w')
        {
            ctx->is_valid = (wav_file_write_header(ctx) == 0);
        }
        else
        {
            ctx->is_valid = (wav_file_read_header(ctx) == 0);
        }
        return 0;
    }
    return -2;
}
//==============================================================================
int wav_file_close(struct wav_file_t * ctx)
{
    if (ctx->file)
    {
        if (ctx->file_mode[0] == 'w')
        {
            wav_file_write_header(ctx);
        }
        fclose(ctx->file);
        ctx->file = 0;
    }
    ctx->file_size = 0;
    ctx->is_valid = 0;
    return 0;
}
//==============================================================================
int wav_file_write_header(struct wav_file_t * ctx)
{
    if (ctx->file == 0)
    {
        return -1;
    }
    uint16_t b16;
    uint32_t b32;
    size_t pos = ftell(ctx->file);
    fseek(ctx->file, 0, SEEK_SET);
    //=============================================================================
    char ChunkID[4] = { 'R', 'I', 'F', 'F' };
    fwrite(ChunkID, sizeof(ChunkID), 1, ctx->file);// 0..3 (4 bytes) - "RIFF"
    if (ctx->file_size > INT32_MAX - 8)
    {
        b32 = 0;
    }
    else
    {
        if (ctx->file_size > 8)
        {
            b32 = ctx->file_size - 8;
        }
        else
        {
            b32 = 8;
        }
    }
    fwrite(&b32, sizeof(b32), 1, ctx->file);// 4..7 (4 bytes) - ChunkSize
    char FormatID[4] = { 'W', 'A', 'V', 'E' };
    fwrite(FormatID, sizeof(FormatID), 1, ctx->file);// 8..11 (4 bytes) - "WAVE"
    char SubChunk1ID[4] = { 'f', 'm', 't', ' ' };
    fwrite(SubChunk1ID, sizeof(SubChunk1ID), 1, ctx->file);// 12..15 (4 bytes) - "fmt "
    b32 = 16;
    fwrite(&b32, sizeof(b32), 1, ctx->file);// 16..19 (4 bytes) - ChunkSize = SizeOf(Wave Format Chunk) = 16
    b16 = ctx->audio_format;
    fwrite(&b16, sizeof(b16), 1, ctx->file); // 20..21 (2 bytes) - AudioFormat
    b16 = ctx->channels_count;
    fwrite(&b16, sizeof(b16), 1, ctx->file); // 22..23 (2 bytes) - channels_count
    b32 = ctx->sample_rate;
    fwrite(&b32, sizeof(b32), 1, ctx->file); // 24..27 (4 bytes) - SamplingFrequency
    int BytesPerSampleGorup = ctx->bytes_per_sample * ctx->channels_count;
    ctx->bytes_per_second = ctx->sample_rate * BytesPerSampleGorup;
    b32 = ctx->bytes_per_second;
    fwrite(&b32, sizeof(b32), 1, ctx->file); // 28..31 (4 bytes) - BytesPerSecond
    b16 = BytesPerSampleGorup;
    fwrite(&b16, sizeof(b16), 1, ctx->file);// 31..33 (2 bytes) - BytesPerSampleGorup aka BlockAlign
    b16 = ctx->bytes_per_sample * 8;
    fwrite(&b16, sizeof(b16), 1, ctx->file);// 34..35 (2 bytes) - BitsPerSample
    char SubChunk2ID[4] = { 'd', 'a', 't', 'a' };
    fwrite(SubChunk2ID, sizeof(SubChunk2ID), 1, ctx->file);// 36..39 (4 bytes) - "data"
    long DataSize = ctx->sample_groups_count * BytesPerSampleGorup;
    if (DataSize > INT32_MAX)
    {
        b32 = INT32_MAX;
    }
    else
    {
        b32 = DataSize;
    }
    fwrite(&b32, sizeof(b32), 1, ctx->file); // 40..43 (4 bytes) - ChunkSize
    ctx->data_start = ftell(ctx->file);
    if (pos < ctx->data_start)
    {
        fseek(ctx->file, ctx->data_start, SEEK_SET);
    }
    else
    {
        fseek(ctx->file, pos, SEEK_SET);
    }
    //==========================================================================
    return 0;
}
//==============================================================================
int wav_file_read_header(struct wav_file_t * ctx)
{
    if (!ctx->file)
    {
        return -1;
    }
    uint16_t b16;
    uint32_t b32;
    size_t rd = 0;
    char ChunkID[4] = { 0,0,0,0 };
    long prev_pos = ftell(ctx->file);
    fseek(ctx->file, 0, SEEK_END);
    ctx->file_size = ftell(ctx->file);
    fseek(ctx->file, 0, SEEK_SET);
    rd += fread(ChunkID, 4, 1, ctx->file);
    int result = 0;
    if ((ChunkID[0] == 'R') && (ChunkID[1] == 'I') && (ChunkID[2] == 'F') && (ChunkID[3] == 'F'))
    {
        rd += fread(&b32, sizeof(b32), 1, ctx->file);
        rd += fread(ChunkID, 4, 1, ctx->file);
        if ((ChunkID[0] == 'W') && (ChunkID[1] == 'A') && (ChunkID[2] == 'V') && (ChunkID[3] == 'E'))
        {
            while (1)
            {
                size_t chunc_start = ftell(ctx->file);

                rd += fread(ChunkID, 4, 1, ctx->file);
                rd += fread(&b32, sizeof(b32), 1, ctx->file);
                size_t ChunkSize = b32;
                if ((ChunkID[0] == 'f') && (ChunkID[1] == 'm') && (ChunkID[2] == 't') && (ChunkID[3] == ' '))
                {
                    rd += fread(&b16, sizeof(b16), 1, ctx->file);
                    ctx->audio_format = b16;
                    rd += fread(&b16, sizeof(b16), 1, ctx->file);
                    if (b16 == 0)
                    {
                        b16 = 1;
                    }
                    ctx->channels_count = b16;
                    rd += fread(&b32, sizeof(b32), 1, ctx->file);
                    ctx->sample_rate = b32;
                    rd += fread(&b32, sizeof(b32), 1, ctx->file);
                    ctx->bytes_per_second = b32;
                    rd += fread(&b16, sizeof(b16), 1, ctx->file);
                    if (b16)
                    {
                        ctx->bytes_per_sample_group = b16;
                    }
                    rd += fread(&b16, sizeof(b16), 1, ctx->file); // BitsPerSample
                    if (b16)
                    {
                        //BytesPerSample = b16 / 8;
                    }
                }
                // Other Chunks may be ...
                //
                //
                if ((ChunkID[0] == 'd') && (ChunkID[1] == 'a') && (ChunkID[2] == 't') && (ChunkID[3] == 'a'))
                {
                    ctx->data_start = ftell(ctx->file);
                    break;
                }
                chunc_start = chunc_start + ChunkSize + 8; // Next Chunk
                if (chunc_start >= ctx->file_size)
                {
                    break;
                }
                fseek(ctx->file, chunc_start, SEEK_SET);
            }
            if (ctx->data_start == 0)
            {
                result = -4; // No data
            }
        }
        else
        {
            result = -3; // No WAVE
        }
    }
    else
    {
        result = -2; // No "RIFF"
    }
    if (ctx->bytes_per_sample_group)
    {
        ctx->sample_groups_count = (ctx->file_size - ctx->data_start) / ctx->bytes_per_sample_group;
    }
    ctx->samples_count = ctx->sample_groups_count * ctx->channels_count;
    if (prev_pos)
    {
        fseek(ctx->file, prev_pos, SEEK_SET);
    }
    if (rd == 0)
    {
        result = -5;
    }
    return result;
}
//==============================================================================
size_t wav_file_write_data(struct wav_file_t * ctx, void * data, size_t size)
{
    if (ctx->file)
    {
        size_t wrote = fwrite(data, size, 1, ctx->file) * size;
        ctx->file_size = ftell(ctx->file);
        ctx->samples_count += (size / (size_t)ctx->bytes_per_sample);
        ctx->sample_groups_count = ctx->samples_count * ctx->channels_count;
        return wrote;
    }
    else
    {
        return 0;
    }
}
//==============================================================================
size_t wav_file_read_data(struct wav_file_t * ctx, void * data, size_t size)
{
    int Result = 0;
    if (ctx->file)
    {
        Result = fread(data, 1, size, ctx->file);
    }
    return Result;
}
//==============================================================================
size_t wav_file_samples_to_data(struct wav_file_t * ctx, const float * samples, size_t count, char * data, size_t * size)
{
    size_t data_size = 0;
    size_t i;
    switch (ctx->bytes_per_sample)
    {
        case 1:
        {
            data_size = count;
            if (samples && data)
            {
                int Sample = 0;
                for (i = 0; i < count; i++)
                {
                    Sample = (int)(samples[i] * 128.0f + 128.5f);
                    if (Sample > 255)
                    {
                        Sample = 255;
                    }
                    if (Sample < 0)
                    {
                        Sample = 0;
                    }
                    data[i] = Sample;
                }
            }
            break;
        }
        case 2:
        {
            data_size = count * 2;
            if (samples && data)
            {
                int16_t * b16 = (int16_t *)data;
                int Sample = 0;
                for (i = 0; i < count; i++)
                {
                    Sample = (int)(samples[i] * 32768.0f + 0.5f);
                    if (Sample>32767)
                    {
                        Sample = 32767;
                    }
                    if (Sample<-32768)
                    {
                        Sample = -32768;
                    }
                    b16[i] = Sample;
                }
            }
            break;
        }
        case 3:
        {
            data_size = count * 3;
            if (samples && data)
            {
                int Sample = 0;
                uint8_t * b8 = (uint8_t *)data;
                for (i = 0; i < count; i++)
                {
                    Sample = (int)(samples[i] * 8388608.0f + 0.5f);
                    if (Sample>8388607)
                    {
                        Sample = 8388607;
                    }
                    if (Sample<-8388608)
                    {
                        Sample = -8388608;
                    }
                    *b8++ = Sample & 0xFF;
                    *b8++ = (Sample >> 8) & 0xFF;
                    *b8++ = (Sample >> 16) & 0xFF;
                }
            }
            break;
        }
        case 4:
        {
            switch (ctx->audio_format)
            {
                case 1: // WAVE_FORMAT_PCM
                {
                    data_size = count * 4;
                    if (samples && data)
                    {
                        int32_t * b32 = (int32_t *)data;
                        for (i = 0; i < count; i++)
                        {
                            *b32++ = (int)(samples[i] * 2147483648.0f + 0.5f);
                        }
                    }
                    break;
                }
                case 2:
                {
                    data_size = 0;
                    break;
                }
                case 3: // WAVE_FORMAT_IEEE_FLOAT
                {
                    data_size = count * 4;
                    if ((void*)samples != (void*)data)
                    {
                        memcpy(data, samples,  data_size);
                    }
                    break;
                }
            }
            break;
        }
    }
    if (size)
    {
        *size = data_size;
    }
    return data_size;
}
//==============================================================================
size_t wav_file_data_to_samples(struct wav_file_t * ctx, const char * data, size_t size, float * samples, size_t * count)
{
    size_t samples_count = 0;
    size_t i;
    switch (ctx->bytes_per_sample)
    {
        case 1:
        {
            samples_count = size;
            if (data && samples)
            {
                for (i = 0; i < samples_count; i++)
                {
                    samples[i] = data[i] * 0.0078125f;
                }
            }
            break;
        }
        case 2:
        {
            samples_count = size / 2;
            if (data && samples)
            {
                int16_t * b16 = (int16_t *)data;
                for (i = 0; i < samples_count; i++)
                {
                    samples[i] = b16[i] * 0.000030517578125f;
                }
            }
            break;
        }
        case 3:
        {
            samples_count = size / 2;
            if (data && samples)
            {
                int Sample = 0;
                int S0 = 0;
                int S1 = 0;
                int S2 = 0;
                uint8_t * b8 = (uint8_t *)data;
                for (i = 0; i < samples_count; i++)
                {
                    S0 = *b8++;
                    S1 = *b8++;
                    S2 = *b8++;
                    Sample = S0 + (S1 << 8) + (S2 << 16);
                    samples[i] = Sample * 0.00000011920928955078125f;
                }
            }
            break;
        }
        case 4:
        {
            samples_count = size / 4;
            if (data && samples)
            {
                switch (ctx->audio_format)
                {
                    case 1: // WAVE_FORMAT_PCM
                    {
                        int32_t * b32 = (int32_t *)data;
                        for (i = 0; i < samples_count; i++)
                        {
                            samples[i] = b32[i] * 0.0000000004656612873077392578125f;
                        }
                        break;
                    }
                    case 2:
                    {
                        samples_count = 0;
                        break;
                    }
                    case 3: // WAVE_FORMAT_IEEE_FLOAT
                    {
                        if ((void*)samples != (void*)data)
                        {
                            memcpy(samples, data, samples_count * 4);
                        }
                        break;
                    }
                }
            }
            break;
        }
    }
    if (count)
    {
        *count = samples_count;
    }
    return samples_count;
}
//==============================================================================
