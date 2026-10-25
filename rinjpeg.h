/*
 * RinOS JPEG Decoder ✿
 * 軽量JPEGデコーダー
 * Bounded sequential (SOF0 / SOF1) and progressive (SOF2) JPEG 対応
 */

#ifndef RINJPEG_H
#define RINJPEG_H

#ifdef __cplusplus
#define RJPEG_ZERO_INIT {}
#else
#define RJPEG_ZERO_INIT {0}
#endif

#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

/* ═══════════════════════════════════════════════════════════════
 * 定数
 * ═══════════════════════════════════════════════════════════════*/

#define RJPEG_OK             0
#define RJPEG_ERROR         -1
#define RJPEG_DATA_ERROR    -2
#define RJPEG_UNSUPPORTED   -3
#define RJPEG_CANCELLED     -4

#define RJPEG_MAX_INPUT_BYTES (64u * 1024u * 1024u)
#define RJPEG_MAX_DIMENSION 8192
#define RJPEG_MAX_PIXELS (4096u * 4096u)
#define RJPEG_MAX_BLOCKS_PER_COMPONENT 8
#define RJPEG_MAX_PROGRESSIVE_COEFFICIENT_BYTES (32u * 1024u * 1024u)

/* JPEGマーカー */
#define RJPEG_SOI   0xFFD8  /* Start of Image */
#define RJPEG_EOI   0xFFD9  /* End of Image */
#define RJPEG_SOF0  0xFFC0  /* Baseline DCT */
#define RJPEG_SOF1  0xFFC1  /* Extended sequential DCT */
#define RJPEG_SOF2  0xFFC2  /* Progressive DCT */
#define RJPEG_DHT   0xFFC4  /* Define Huffman Table */
#define RJPEG_DQT   0xFFDB  /* Define Quantization Table */
#define RJPEG_DRI   0xFFDD  /* Define Restart Interval */
#define RJPEG_SOS   0xFFDA  /* Start of Scan */
#define RJPEG_APP0  0xFFE0  /* JFIF marker */
#define RJPEG_COM   0xFFFE  /* Comment */

/* ═══════════════════════════════════════════════════════════════
 * 構造体
 * ═══════════════════════════════════════════════════════════════*/

typedef struct {
    uint8_t table[64];
} RJpegQuantTable;

typedef struct {
    uint8_t  bits[17];      /* ビット長ごとのコード数 */
    uint8_t  values[256];   /* ハフマン値 */
    uint16_t codes[256];    /* ハフマンコード */
    uint8_t  sizes[256];    /* コードサイズ */
    int      count;
} RJpegHuffTable;

typedef struct {
    int id;
    int h_samp, v_samp;     /* サンプリング係数 */
    int qt_id;              /* 量子化テーブルID */
    int dc_id, ac_id;       /* ハフマンテーブルID */
    int32_t dc_pred;        /* DC予測値 */
} RJpegComponent;

typedef struct {
    uint8_t component_count;
    uint8_t component_index[3];
    uint8_t dc_id[3];
    uint8_t ac_id[3];
    uint8_t spectral_start;
    uint8_t spectral_end;
    uint8_t successive_high;
    uint8_t successive_low;
    int restart_interval;
} RJpegScan;

typedef int (*RJpegCancellationFunction)(void* context);

typedef struct {
    const uint8_t* data;
    size_t size;
    size_t pos;
    
    int width, height;
    int num_components;
    int progressive;
    RJpegComponent comp[4];
    int8_t progressive_approximation[3][64];
    
    RJpegQuantTable qt[4];
    RJpegHuffTable huff_dc[4];
    RJpegHuffTable huff_ac[4];

    uint8_t qt_present;
    uint8_t huff_dc_present;
    uint8_t huff_ac_present;
    uint8_t reserved;
    
    int restart_interval;
    
    /* ビットストリーム */
    uint32_t bit_buf;
    int bit_count;
    int failed;
    RJpegCancellationFunction cancellation;
    void* cancellation_context;
} RJpegDecoder;

static inline int rjpeg_cancelled(const RJpegDecoder* decoder) {
    return decoder != NULL && decoder->cancellation != NULL &&
           decoder->cancellation(decoder->cancellation_context) != 0;
}

/* ═══════════════════════════════════════════════════════════════
 * ユーティリティ
 * ═══════════════════════════════════════════════════════════════*/

static inline int rjpeg_take16(RJpegDecoder* d, uint16_t* value_out) {
    if (!d || !value_out || d->pos > d->size || d->size - d->pos < 2u) {
        if (d) d->failed = 1;
        return 0;
    }
    *value_out = ((uint16_t)d->data[d->pos] << 8) |
                 (uint16_t)d->data[d->pos + 1u];
    d->pos += 2u;
    return 1;
}

static inline int rjpeg_take8(RJpegDecoder* d, uint8_t* value_out) {
    if (!d || !value_out || d->pos >= d->size) {
        if (d) d->failed = 1;
        return 0;
    }
    *value_out = d->data[d->pos++];
    return 1;
}

/* ビットストリーム読み取り（スタッフィングバイト対応） */
static inline int rjpeg_next_byte(RJpegDecoder* d) {
    if (!d || d->failed || d->pos >= d->size) {
        if (d) d->failed = 1;
        return -1;
    }
    uint8_t b = d->data[d->pos++];
    if (b == 0xFF) {
        if (d->pos >= d->size) {
            d->failed = 1;
            return -1;
        }
        uint8_t next = d->data[d->pos];
        if (next == 0x00) {
            d->pos++;  /* スタッフィングバイトをスキップ */
        } else {
            /* Restart markers are accepted only at a validated MCU boundary. */
            d->failed = 1;
            return -1;
        }
    }
    return b;
}

static inline int rjpeg_get_bits(RJpegDecoder* d, int n, int* value_out) {
    if (!d || !value_out || n < 0 || n > 16) {
        if (d) d->failed = 1;
        return 0;
    }
    while (d->bit_count < n) {
        int b = rjpeg_next_byte(d);
        if (b < 0) return 0;
        d->bit_buf = (d->bit_buf << 8) | (uint32_t)b;
        d->bit_count += 8;
    }
    d->bit_count -= n;
    *value_out = n == 0
                     ? 0
                     : (int)((d->bit_buf >> d->bit_count) &
                             ((1u << (unsigned)n) - 1u));
    return 1;
}

/* ═══════════════════════════════════════════════════════════════
 * ハフマンデコード
 * ═══════════════════════════════════════════════════════════════*/

static inline int rjpeg_build_huff(RJpegHuffTable* h) {
    int code = 0;
    int idx = 0;
    int available = 1;
    
    for (int bits = 1; bits <= 16; bits++) {
        available <<= 1;
        if ((int)h->bits[bits] > available) return 0;
        available -= (int)h->bits[bits];
        for (int i = 0; i < h->bits[bits]; i++) {
            if (idx >= 256) return 0;
            h->codes[idx] = code;
            h->sizes[idx] = bits;
            idx++;
            code++;
        }
        code <<= 1;
    }
    h->count = idx;
    return idx != 0;
}

static inline int rjpeg_huff_decode(RJpegDecoder* d,
                                    const RJpegHuffTable* h,
                                    int* symbol_out) {
    int code = 0;
    if (!d || !h || !symbol_out || h->count <= 0 || h->count > 256)
        return 0;
    for (int bits = 1; bits <= 16; bits++) {
        int bit = 0;
        if (!rjpeg_get_bits(d, 1, &bit)) return 0;
        code = (code << 1) | bit;
        
        for (int i = 0; i < h->count; i++) {
            if (h->sizes[i] == bits && h->codes[i] == code) {
                *symbol_out = h->values[i];
                return 1;
            }
        }
    }
    d->failed = 1;
    return 0;
}

/* 符号付き値の展開 */
static inline int rjpeg_extend(int val, int bits) {
    if (bits == 0) return 0;
    int half = 1 << (bits - 1);
    if (val < half) {
        return val - (2 * half - 1);
    }
    return val;
}

/* ═══════════════════════════════════════════════════════════════
 * IDCT (逆離散コサイン変換)
 * ═══════════════════════════════════════════════════════════════*/

/* ジグザグ順序 */
static const uint8_t g_rjpeg_zigzag[64] = {
     0,  1,  8, 16,  9,  2,  3, 10,
    17, 24, 32, 25, 18, 11,  4,  5,
    12, 19, 26, 33, 40, 48, 41, 34,
    27, 20, 13,  6,  7, 14, 21, 28,
    35, 42, 49, 56, 57, 50, 43, 36,
    29, 22, 15, 23, 30, 37, 44, 51,
    58, 59, 52, 45, 38, 31, 39, 46,
    53, 60, 61, 54, 47, 55, 62, 63
};

/* C(u) * cos((2x + 1)u*pi/16), Q14 fixed point. */
static const int16_t g_rjpeg_idct_basis[8][8] = {
    { 11585,  11585,  11585,  11585,  11585,  11585,  11585,  11585 },
    { 16069,  13623,   9102,   3196,  -3196,  -9102, -13623, -16069 },
    { 15137,   6270,  -6270, -15137, -15137,  -6270,   6270,  15137 },
    { 13623,  -3196, -16069,  -9102,   9102,  16069,   3196, -13623 },
    { 11585, -11585, -11585,  11585,  11585, -11585, -11585,  11585 },
    {  9102, -16069,   3196,  13623, -13623,  -3196,  16069,  -9102 },
    {  6270, -15137,  15137,  -6270,  -6270,  15137, -15137,   6270 },
    {  3196,  -9102,  13623, -16069,  16069, -13623,   9102,  -3196 }
};

static inline int rjpeg_idct_round_q30(int64_t value) {
    const int64_t denominator = (int64_t)1 << 30;
    const int64_t half = (int64_t)1 << 29;
    if (value < 0) return (int)(-((-value + half) / denominator));
    return (int)((value + half) / denominator);
}

static inline void rjpeg_idct(int16_t* block, const uint8_t* qt) {
    int coefficients[64];
    int64_t horizontal[64];

    /* Dequantize into natural coefficient order. */
    for (int i = 0; i < 64; i++) {
        coefficients[g_rjpeg_zigzag[i]] =
            (int)block[i] * (int)qt[g_rjpeg_zigzag[i]];
    }

    /* A separable, full 8x8 inverse transform in Q14/Q30 arithmetic. */
    for (int y = 0; y < 8; y++) {
        for (int x = 0; x < 8; x++) {
            int64_t sum = 0;
            for (int u = 0; u < 8; u++) {
                sum += (int64_t)coefficients[y * 8 + u] *
                       g_rjpeg_idct_basis[u][x];
            }
            horizontal[y * 8 + x] = sum;
        }
    }
    for (int y = 0; y < 8; y++) {
        for (int x = 0; x < 8; x++) {
            int64_t sum = 0;
            int value;
            for (int v = 0; v < 8; v++) {
                sum += horizontal[v * 8 + x] *
                       g_rjpeg_idct_basis[v][y];
            }
            value = rjpeg_idct_round_q30(sum) + 128;
            if (value < 0) value = 0;
            if (value > 255) value = 255;
            block[y * 8 + x] = (int16_t)value;
        }
    }
}

/* ═══════════════════════════════════════════════════════════════
 * ブロックデコード
 * ═══════════════════════════════════════════════════════════════*/

static inline int rjpeg_decode_block(RJpegDecoder* d, int16_t* block,
                                     RJpegComponent* comp,
                                     int perform_idct) {
    /* ブロック初期化 */
    for (int i = 0; i < 64; i++) block[i] = 0;
    
    /* DC係数 */
    RJpegHuffTable* dc_huff = &d->huff_dc[comp->dc_id];
    int dc_len = 0;
    if (!rjpeg_huff_decode(d, dc_huff, &dc_len) || dc_len < 0 ||
        dc_len > 11)
        return RJPEG_DATA_ERROR;
    if (dc_len > 0) {
        int dc_val = 0;
        if (!rjpeg_get_bits(d, dc_len, &dc_val))
            return RJPEG_DATA_ERROR;
        dc_val = rjpeg_extend(dc_val, dc_len);
        if (dc_val > 0 && comp->dc_pred > INT16_MAX - dc_val)
            return RJPEG_DATA_ERROR;
        if (dc_val < 0 && comp->dc_pred < INT16_MIN - dc_val)
            return RJPEG_DATA_ERROR;
        comp->dc_pred += (int32_t)dc_val;
    }
    block[0] = (int16_t)comp->dc_pred;
    
    /* AC係数 */
    RJpegHuffTable* ac_huff = &d->huff_ac[comp->ac_id];
    int i = 1;
    while (i < 64) {
        int code = 0;
        if (!rjpeg_huff_decode(d, ac_huff, &code))
            return RJPEG_DATA_ERROR;
        if (code == 0) break;  /* EOB */
        
        int zeros = (code >> 4) & 0x0F;
        int ac_len = code & 0x0F;
        
        if (ac_len == 0) {
            if (zeros == 15) {
                i += 16;  /* ZRL */
                if (i > 64) return RJPEG_DATA_ERROR;
            } else {
                return RJPEG_DATA_ERROR;
            }
        } else {
            if (ac_len > 10) return RJPEG_DATA_ERROR;
            i += zeros;
            if (i >= 64) return RJPEG_DATA_ERROR;
            int ac_val = 0;
            if (!rjpeg_get_bits(d, ac_len, &ac_val))
                return RJPEG_DATA_ERROR;
            block[i] = (int16_t)rjpeg_extend(ac_val, ac_len);
            i++;
        }
    }
    
    /* IDCT */
    if (perform_idct)
        rjpeg_idct(block, d->qt[comp->qt_id].table);
    
    return RJPEG_OK;
}

/* ═══════════════════════════════════════════════════════════════
 * マーカー解析
 * ═══════════════════════════════════════════════════════════════*/

static inline int rjpeg_begin_segment(RJpegDecoder* d, size_t* end_out) {
    uint16_t length = 0;
    if (!rjpeg_take16(d, &length) || length < 2u ||
        d->pos > d->size || (size_t)(length - 2u) > d->size - d->pos)
        return 0;
    *end_out = d->pos + (size_t)(length - 2u);
    return 1;
}

static inline int rjpeg_next_marker(RJpegDecoder* d, uint16_t* marker_out) {
    uint8_t code = 0;
    size_t skipped = 0u;
    if (!d || !marker_out || d->pos >= d->size ||
        d->data[d->pos++] != 0xffu)
        return 0;
    while (d->pos < d->size && d->data[d->pos] == 0xffu) {
        d->pos++;
        if ((++skipped & 4095u) == 0u && rjpeg_cancelled(d))
            return RJPEG_CANCELLED;
    }
    if (!rjpeg_take8(d, &code) || code == 0u) return 0;
    *marker_out = (uint16_t)(0xff00u | (uint16_t)code);
    return 1;
}

static inline int rjpeg_parse_dqt(RJpegDecoder* d, size_t end) {
    while (d->pos < end) {
        uint8_t info = 0;
        if (!rjpeg_take8(d, &info)) return RJPEG_DATA_ERROR;
        int precision = (info >> 4) & 0x0f;
        int id = info & 0x0f;
        if (precision != 0 || id >= 4 || end - d->pos < 64u)
            return precision != 0 ? RJPEG_UNSUPPORTED : RJPEG_DATA_ERROR;
        for (int i = 0; i < 64; i++) {
            uint8_t value = 0;
            if (!rjpeg_take8(d, &value) || value == 0u)
                return RJPEG_DATA_ERROR;
            d->qt[id].table[g_rjpeg_zigzag[i]] = value;
        }
        d->qt_present |= (uint8_t)(1u << (unsigned)id);
    }
    return d->pos == end ? RJPEG_OK : RJPEG_DATA_ERROR;
}

static inline int rjpeg_huff_symbol_valid(int type, uint8_t value) {
    if (type == 0) return value <= 11u;
    int zeros = (value >> 4) & 0x0f;
    int bits = value & 0x0f;
    if (bits == 0) return zeros == 0 || zeros == 15;
    return bits <= 10;
}

static inline int rjpeg_parse_dht(RJpegDecoder* d, size_t end) {
    while (d->pos < end) {
        uint8_t info = 0;
        if (!rjpeg_take8(d, &info)) return RJPEG_DATA_ERROR;
        int type = (info >> 4) & 0x0f;
        int id = info & 0x0f;
        if (type > 1 || id >= 4 || end - d->pos < 16u)
            return RJPEG_DATA_ERROR;
        RJpegHuffTable* h = type == 0 ? &d->huff_dc[id] : &d->huff_ac[id];
        h->bits[0] = 0u;
        int total = 0;
        for (int i = 1; i <= 16; i++) {
            if (!rjpeg_take8(d, &h->bits[i])) return RJPEG_DATA_ERROR;
            total += (int)h->bits[i];
        }
        if (total <= 0 || total > 256 ||
            (size_t)total > end - d->pos)
            return RJPEG_DATA_ERROR;
        for (int i = 0; i < total; i++) {
            if (!rjpeg_take8(d, &h->values[i]) ||
                !rjpeg_huff_symbol_valid(type, h->values[i]))
                return RJPEG_DATA_ERROR;
        }
        if (!rjpeg_build_huff(h)) return RJPEG_DATA_ERROR;
        if (type == 0)
            d->huff_dc_present |= (uint8_t)(1u << (unsigned)id);
        else
            d->huff_ac_present |= (uint8_t)(1u << (unsigned)id);
    }
    return d->pos == end ? RJPEG_OK : RJPEG_DATA_ERROR;
}

static inline int rjpeg_sampling_factors_supported(int horizontal,
                                                    int vertical) {
    return (horizontal == 1 || horizontal == 2 || horizontal == 4) &&
           (vertical == 1 || vertical == 2 || vertical == 4) &&
           horizontal * vertical <= RJPEG_MAX_BLOCKS_PER_COMPONENT;
}

static inline int rjpeg_parse_sof(RJpegDecoder* d, size_t end,
                                  int progressive) {
    uint8_t precision = 0;
    uint8_t component_count = 0;
    uint16_t height = 0;
    uint16_t width = 0;
    RJpegComponent components[3] = RJPEG_ZERO_INIT;
    if (!rjpeg_take8(d, &precision) || !rjpeg_take16(d, &height) ||
        !rjpeg_take16(d, &width) || !rjpeg_take8(d, &component_count))
        return RJPEG_DATA_ERROR;
    if (precision != 8u) return RJPEG_UNSUPPORTED;
    if (width == 0u || height == 0u ||
        (component_count != 1u && component_count != 3u) ||
        end - d->pos != (size_t)component_count * 3u)
        return RJPEG_DATA_ERROR;
    for (int i = 0; i < (int)component_count; i++) {
        uint8_t id = 0;
        uint8_t sampling = 0;
        uint8_t table = 0;
        int horizontal_sampling;
        int vertical_sampling;
        if (!rjpeg_take8(d, &id) || !rjpeg_take8(d, &sampling) ||
            !rjpeg_take8(d, &table) || id == 0u || table >= 4u)
            return RJPEG_DATA_ERROR;
        horizontal_sampling = (sampling >> 4) & 0x0f;
        vertical_sampling = sampling & 0x0f;
        if (horizontal_sampling == 0 || vertical_sampling == 0)
            return RJPEG_DATA_ERROR;
        for (int previous = 0; previous < i; previous++) {
            if (components[previous].id == (int)id)
                return RJPEG_DATA_ERROR;
        }
        components[i].id = id;
        components[i].h_samp = horizontal_sampling;
        components[i].v_samp = vertical_sampling;
        components[i].qt_id = table;
    }
    if (component_count == 1u) {
        if (components[0].h_samp != 1 || components[0].v_samp != 1)
            return RJPEG_UNSUPPORTED;
    } else if (!rjpeg_sampling_factors_supported(
                   components[0].h_samp, components[0].v_samp) ||
               components[1].h_samp != 1 ||
               components[1].v_samp != 1 ||
               components[2].h_samp != 1 ||
               components[2].v_samp != 1) {
        return RJPEG_UNSUPPORTED;
    }
    d->width = width;
    d->height = height;
    d->num_components = component_count;
    d->progressive = progressive;
    for (int i = 0; i < (int)component_count; i++) d->comp[i] = components[i];
    if (progressive) {
        for (int component = 0; component < (int)component_count; component++) {
            for (int coefficient = 0; coefficient < 64; coefficient++)
                d->progressive_approximation[component][coefficient] = -1;
        }
    }
    return d->pos == end ? RJPEG_OK : RJPEG_DATA_ERROR;
}

static inline int rjpeg_parse_sos(RJpegDecoder* d, size_t end,
                                  RJpegScan* scan_out) {
    uint8_t scan_count = 0;
    uint8_t selected_components = 0u;
    if (!d || !scan_out || !rjpeg_take8(d, &scan_count) ||
        scan_count == 0u || scan_count > (uint8_t)d->num_components ||
        end - d->pos != (size_t)scan_count * 2u + 3u)
        return RJPEG_DATA_ERROR;
    for (int i = 0; i < (int)scan_count; i++) {
        uint8_t id = 0;
        uint8_t tables = 0;
        int component_index = -1;
        if (!rjpeg_take8(d, &id) || !rjpeg_take8(d, &tables))
            return RJPEG_DATA_ERROR;
        for (int component = 0; component < d->num_components; component++) {
            if (id == (uint8_t)d->comp[component].id) {
                component_index = component;
                break;
            }
        }
        if (component_index < 0 ||
            (selected_components &
             (uint8_t)(1u << (unsigned)component_index)) != 0u)
            return RJPEG_DATA_ERROR;
        int dc = (tables >> 4) & 0x0f;
        int ac = tables & 0x0f;
        if (dc >= 4 || ac >= 4 ||
            (d->qt_present &
             (uint8_t)(1u << (unsigned)d->comp[component_index].qt_id)) ==
                0u)
            return RJPEG_DATA_ERROR;
        selected_components |= (uint8_t)(1u << (unsigned)component_index);
        scan_out->component_index[i] = (uint8_t)component_index;
        scan_out->dc_id[i] = (uint8_t)dc;
        scan_out->ac_id[i] = (uint8_t)ac;
    }
    uint8_t spectral_start = 0;
    uint8_t spectral_end = 0;
    uint8_t approximation = 0;
    if (!rjpeg_take8(d, &spectral_start) ||
        !rjpeg_take8(d, &spectral_end) ||
        !rjpeg_take8(d, &approximation))
        return RJPEG_DATA_ERROR;
    uint8_t successive_high = (uint8_t)(approximation >> 4);
    uint8_t successive_low = (uint8_t)(approximation & 0x0fu);
    if (!d->progressive) {
        if (spectral_start != 0u || spectral_end != 63u || approximation != 0u)
            return RJPEG_UNSUPPORTED;
    } else {
        if (successive_high > 13u || successive_low > 13u ||
            (successive_high != 0u &&
             successive_high != (uint8_t)(successive_low + 1u)))
            return RJPEG_UNSUPPORTED;
        if (spectral_start == 0u) {
            if (spectral_end != 0u) return RJPEG_DATA_ERROR;
        } else if (spectral_start > spectral_end || spectral_end > 63u ||
                   scan_count != 1u) {
            return RJPEG_DATA_ERROR;
        }
    }
    for (int slot = 0; slot < (int)scan_count; slot++) {
        int component = scan_out->component_index[slot];
        int dc = scan_out->dc_id[slot];
        int ac = scan_out->ac_id[slot];
        int uses_dc = !d->progressive ||
                      (spectral_start == 0u && successive_high == 0u);
        int uses_ac = !d->progressive || spectral_start != 0u;
        if ((uses_dc &&
             (d->huff_dc_present & (uint8_t)(1u << (unsigned)dc)) == 0u) ||
            (uses_ac &&
             (d->huff_ac_present & (uint8_t)(1u << (unsigned)ac)) == 0u))
            return RJPEG_DATA_ERROR;
        if (d->progressive) {
            int first = spectral_start == 0u ? 0 : (int)spectral_start;
            int last = spectral_start == 0u ? 0 : (int)spectral_end;
            for (int coefficient = first; coefficient <= last; coefficient++) {
                int8_t previous =
                    d->progressive_approximation[component][coefficient];
                if ((successive_high == 0u && previous != -1) ||
                    (successive_high != 0u &&
                     previous != (int8_t)successive_high))
                    return RJPEG_DATA_ERROR;
            }
        }
    }
    if (d->progressive) {
        for (int slot = 0; slot < (int)scan_count; slot++) {
            int component = scan_out->component_index[slot];
            int first = spectral_start == 0u ? 0 : (int)spectral_start;
            int last = spectral_start == 0u ? 0 : (int)spectral_end;
            for (int coefficient = first; coefficient <= last; coefficient++)
                d->progressive_approximation[component][coefficient] =
                    (int8_t)successive_low;
        }
    }
    if (d->pos != end) return RJPEG_DATA_ERROR;
    scan_out->component_count = scan_count;
    scan_out->spectral_start = spectral_start;
    scan_out->spectral_end = spectral_end;
    scan_out->successive_high = successive_high;
    scan_out->successive_low = successive_low;
    scan_out->restart_interval = d->restart_interval;
    return RJPEG_OK;
}

static inline int rjpeg_parse_dri(RJpegDecoder* d) {
    size_t end = 0u;
    uint16_t interval = 0;
    if (!d || !rjpeg_begin_segment(d, &end) || end - d->pos != 2u ||
        !rjpeg_take16(d, &interval) || d->pos != end) {
        return RJPEG_DATA_ERROR;
    }
    d->restart_interval = (int)interval;
    return RJPEG_OK;
}

/* ═══════════════════════════════════════════════════════════════
 * YCbCr -> RGB変換
 * ═══════════════════════════════════════════════════════════════*/

static inline void rjpeg_ycbcr_to_rgb(int y, int cb, int cr, uint8_t* r, uint8_t* g, uint8_t* b) {
    cb -= 128;
    cr -= 128;
    
    int ri = y + ((cr * 359) >> 8);
    int gi = y - ((cb * 88 + cr * 183) >> 8);
    int bi = y + ((cb * 454) >> 8);
    
    *r = (ri < 0) ? 0 : (ri > 255) ? 255 : ri;
    *g = (gi < 0) ? 0 : (gi > 255) ? 255 : gi;
    *b = (bi < 0) ? 0 : (bi > 255) ? 255 : bi;
}

static inline int rjpeg_parse_headers(const uint8_t* data,
                                      size_t size,
                                      RJpegDecoder* decoder,
                                      size_t* scan_position_out,
                                      RJpegScan* first_scan_out) {
    if (!data || !decoder || !scan_position_out || !first_scan_out ||
        size < 4u ||
        size > RJPEG_MAX_INPUT_BYTES || data[0] != 0xffu ||
        data[1] != 0xd8u)
        return RJPEG_DATA_ERROR;
    decoder->data = data;
    decoder->size = size;
    decoder->pos = 2u;
    int saw_frame = 0;
    for (;;) {
        uint16_t marker = 0;
        size_t end = 0;
        if (rjpeg_cancelled(decoder)) return RJPEG_CANCELLED;
        {
            int marker_status = rjpeg_next_marker(decoder, &marker);
            if (marker_status == RJPEG_CANCELLED) return marker_status;
            if (marker_status != 1) return RJPEG_DATA_ERROR;
        }
        if (marker == RJPEG_SOF0 || marker == RJPEG_SOF1 ||
            marker == RJPEG_SOF2) {
            if (saw_frame || !rjpeg_begin_segment(decoder, &end))
                return RJPEG_DATA_ERROR;
            int result = rjpeg_parse_sof(
                decoder, end, marker == RJPEG_SOF2 ? 1 : 0);
            if (result != RJPEG_OK) return result;
            if (decoder->width > RJPEG_MAX_DIMENSION ||
                decoder->height > RJPEG_MAX_DIMENSION ||
                (uint64_t)(uint32_t)decoder->width *
                        (uint64_t)(uint32_t)decoder->height >
                    (uint64_t)RJPEG_MAX_PIXELS)
                return RJPEG_UNSUPPORTED;
            saw_frame = 1;
        } else if (marker == RJPEG_DHT) {
            if (!rjpeg_begin_segment(decoder, &end)) return RJPEG_DATA_ERROR;
            int result = rjpeg_parse_dht(decoder, end);
            if (result != RJPEG_OK) return result;
        } else if (marker == RJPEG_DQT) {
            if (!rjpeg_begin_segment(decoder, &end)) return RJPEG_DATA_ERROR;
            int result = rjpeg_parse_dqt(decoder, end);
            if (result != RJPEG_OK) return result;
        } else if (marker == RJPEG_DRI) {
            int result = rjpeg_parse_dri(decoder);
            if (result != RJPEG_OK) return result;
        } else if (marker == RJPEG_SOS) {
            if (!saw_frame || !rjpeg_begin_segment(decoder, &end))
                return RJPEG_DATA_ERROR;
            int result = rjpeg_parse_sos(decoder, end, first_scan_out);
            if (result != RJPEG_OK) return result;
            *scan_position_out = decoder->pos;
            return RJPEG_OK;
        } else if ((marker >= 0xffe0u && marker <= 0xffefu) ||
                   marker == RJPEG_COM) {
            if (!rjpeg_begin_segment(decoder, &end)) return RJPEG_DATA_ERROR;
            decoder->pos = end;
        } else if ((marker >= 0xffc0u && marker <= 0xffcfu) ||
                   marker == 0xffccu) {
            return RJPEG_UNSUPPORTED;
        } else {
            /* Header restart, nested SOI, premature EOI, and unknown markers. */
            return RJPEG_DATA_ERROR;
        }
    }
}

static inline int rjpeg_consume_restart(RJpegDecoder* decoder,
                                        RJpegComponent* components,
                                        int component_count,
                                        uint8_t expected_marker) {
    uint32_t mask;
    if (!decoder || !components || component_count <= 0 ||
        component_count > 3 || decoder->failed || expected_marker > 7u ||
        decoder->bit_count < 0 || decoder->bit_count > 7) {
        return RJPEG_DATA_ERROR;
    }
    if (decoder->bit_count != 0) {
        mask = (1u << (unsigned)decoder->bit_count) - 1u;
        if ((decoder->bit_buf & mask) != mask) return RJPEG_DATA_ERROR;
    }
    decoder->bit_buf = 0u;
    decoder->bit_count = 0;
    if (decoder->pos >= decoder->size ||
        decoder->data[decoder->pos++] != 0xffu) {
        return RJPEG_DATA_ERROR;
    }
    while (decoder->pos < decoder->size &&
           decoder->data[decoder->pos] == 0xffu) {
        decoder->pos++;
        if ((decoder->pos & 4095u) == 0u && rjpeg_cancelled(decoder))
            return RJPEG_CANCELLED;
    }
    if (decoder->pos >= decoder->size ||
        decoder->data[decoder->pos++] !=
            (uint8_t)(0xd0u + expected_marker)) {
        return RJPEG_DATA_ERROR;
    }
    for (int component = 0; component < component_count; component++) {
        components[component].dc_pred = 0;
    }
    return RJPEG_OK;
}

static inline int rjpeg_finish_entropy(RJpegDecoder* decoder,
                                       uint16_t* marker_out) {
    if (!decoder || decoder->failed || decoder->bit_count < 0 ||
        decoder->bit_count > 7 || !marker_out)
        return RJPEG_DATA_ERROR;
    if (decoder->bit_count != 0) {
        uint32_t mask = (1u << (unsigned)decoder->bit_count) - 1u;
        if ((decoder->bit_buf & mask) != mask) return RJPEG_DATA_ERROR;
    }
    decoder->bit_buf = 0u;
    decoder->bit_count = 0;
    if (decoder->pos >= decoder->size ||
        decoder->data[decoder->pos++] != 0xffu)
        return RJPEG_DATA_ERROR;
    while (decoder->pos < decoder->size &&
           decoder->data[decoder->pos] == 0xffu) {
        decoder->pos++;
        if ((decoder->pos & 4095u) == 0u && rjpeg_cancelled(decoder))
            return RJPEG_CANCELLED;
    }
    if (decoder->pos >= decoder->size || decoder->data[decoder->pos] == 0u)
        return RJPEG_DATA_ERROR;
    *marker_out = (uint16_t)(0xff00u | decoder->data[decoder->pos++]);
    return RJPEG_OK;
}

static inline int rjpeg_frame_sampling(const RJpegDecoder* decoder,
                                       int* max_horizontal_out,
                                       int* max_vertical_out) {
    int max_horizontal_sampling;
    int max_vertical_sampling;
    if (!decoder || !max_horizontal_out || !max_vertical_out ||
        (decoder->num_components != 1 && decoder->num_components != 3))
        return RJPEG_DATA_ERROR;
    max_horizontal_sampling = decoder->comp[0].h_samp;
    max_vertical_sampling = decoder->comp[0].v_samp;
    if (!rjpeg_sampling_factors_supported(max_horizontal_sampling,
                                          max_vertical_sampling) ||
        (decoder->num_components == 1 &&
         (max_horizontal_sampling != 1 || max_vertical_sampling != 1)) ||
        (decoder->num_components == 3 &&
         (decoder->comp[1].h_samp != 1 ||
          decoder->comp[1].v_samp != 1 ||
          decoder->comp[2].h_samp != 1 ||
          decoder->comp[2].v_samp != 1)))
        return RJPEG_DATA_ERROR;
    for (int component = 0; component < decoder->num_components;
         component++) {
        int horizontal_sampling = decoder->comp[component].h_samp;
        int vertical_sampling = decoder->comp[component].v_samp;
        if (horizontal_sampling <= 0 || vertical_sampling <= 0 ||
            horizontal_sampling > max_horizontal_sampling ||
            vertical_sampling > max_vertical_sampling ||
            max_horizontal_sampling % horizontal_sampling != 0 ||
            max_vertical_sampling % vertical_sampling != 0 ||
            horizontal_sampling * vertical_sampling >
                RJPEG_MAX_BLOCKS_PER_COMPONENT)
            return RJPEG_DATA_ERROR;
    }
    *max_horizontal_out = max_horizontal_sampling;
    *max_vertical_out = max_vertical_sampling;
    return RJPEG_OK;
}

static inline void rjpeg_publish_component_block(
    const RJpegDecoder* decoder, uint32_t* pixels, int component,
    int max_horizontal_sampling, int max_vertical_sampling,
    int block_x, int block_y, const int16_t* block) {
    int scale_x = max_horizontal_sampling / decoder->comp[component].h_samp;
    int scale_y = max_vertical_sampling / decoder->comp[component].v_samp;
    unsigned shift = component == 0 ? 16u : component == 1 ? 8u : 0u;
    uint32_t channel_mask = 0xffu << shift;
    for (int local_y = 0; local_y < 8; local_y++) {
        int component_y = block_y * 8 + local_y;
        int first_y = component_y * scale_y;
        if (first_y >= decoder->height) break;
        for (int local_x = 0; local_x < 8; local_x++) {
            int component_x = block_x * 8 + local_x;
            int first_x = component_x * scale_x;
            uint32_t sample = (uint32_t)(uint16_t)block[local_y * 8 + local_x];
            if (first_x >= decoder->width) break;
            for (int pixel_y = first_y;
                 pixel_y < first_y + scale_y && pixel_y < decoder->height;
                 pixel_y++) {
                for (int pixel_x = first_x;
                     pixel_x < first_x + scale_x && pixel_x < decoder->width;
                     pixel_x++) {
                    size_t index = (size_t)pixel_y * (size_t)decoder->width +
                                   (size_t)pixel_x;
                    pixels[index] = (pixels[index] & ~channel_mask) |
                                    (sample << shift);
                }
            }
        }
    }
}

static inline int rjpeg_finalize_pixels(const RJpegDecoder* decoder,
                                        uint32_t* pixels) {
    for (int row = 0; row < decoder->height; ++row) {
        size_t index;
        const size_t row_start = (size_t)row * (size_t)decoder->width;
        const size_t row_end = row_start + (size_t)decoder->width;
        if (rjpeg_cancelled(decoder)) return RJPEG_CANCELLED;
        for (index = row_start; index < row_end; ++index) {
            uint8_t first = (uint8_t)(pixels[index] >> 16);
            uint8_t second = (uint8_t)(pixels[index] >> 8);
            uint8_t third = (uint8_t)pixels[index];
            uint8_t red = first;
            uint8_t green = first;
            uint8_t blue = first;
            if (decoder->num_components == 3)
                rjpeg_ycbcr_to_rgb(first, second, third,
                                   &red, &green, &blue);
            pixels[index] = 0xff000000u | ((uint32_t)red << 16) |
                            ((uint32_t)green << 8) | (uint32_t)blue;
        }
    }
    return RJPEG_OK;
}

static inline int rjpeg_decode_scan_pass(RJpegDecoder* decoder,
                                         const RJpegScan* scan,
                                         uint32_t* pixels,
                                         int publish_pixels,
                                         uint16_t* next_marker_out) {
    int16_t block[64] = RJPEG_ZERO_INIT;
    RJpegComponent active_components[3] = RJPEG_ZERO_INIT;
    int max_horizontal_sampling;
    int max_vertical_sampling;
    int mcu_width;
    int mcu_height;
    uint32_t decoded_mcus = 0u;
    uint8_t expected_restart = 0u;
    uint8_t active_components_seen = 0u;
    if (!decoder || !scan || !next_marker_out ||
        (publish_pixels != 0 && publish_pixels != 1) ||
        (publish_pixels != 0 && !pixels) || scan->component_count == 0u ||
        scan->component_count > (uint8_t)decoder->num_components)
        return RJPEG_DATA_ERROR;
    if (rjpeg_frame_sampling(decoder, &max_horizontal_sampling,
                             &max_vertical_sampling) != RJPEG_OK) {
        return RJPEG_DATA_ERROR;
    }
    for (int slot = 0; slot < (int)scan->component_count; slot++) {
        int component = scan->component_index[slot];
        if (component < 0 || component >= decoder->num_components ||
            (active_components_seen & (uint8_t)(1u << (unsigned)component)) !=
                0u || scan->dc_id[slot] >= 4u || scan->ac_id[slot] >= 4u) {
            return RJPEG_DATA_ERROR;
        }
        active_components_seen |= (uint8_t)(1u << (unsigned)component);
        active_components[slot] = decoder->comp[component];
        active_components[slot].dc_id = scan->dc_id[slot];
        active_components[slot].ac_id = scan->ac_id[slot];
        active_components[slot].dc_pred = 0;
    }
    if (scan->component_count == 1u) {
        int component_width = (decoder->width * active_components[0].h_samp +
                               max_horizontal_sampling - 1) /
                              max_horizontal_sampling;
        int component_height = (decoder->height * active_components[0].v_samp +
                                max_vertical_sampling - 1) /
                               max_vertical_sampling;
        mcu_width = (component_width + 7) / 8;
        mcu_height = (component_height + 7) / 8;
    } else {
        mcu_width = (decoder->width + max_horizontal_sampling * 8 - 1) /
                    (max_horizontal_sampling * 8);
        mcu_height = (decoder->height + max_vertical_sampling * 8 - 1) /
                     (max_vertical_sampling * 8);
    }
    if (mcu_width <= 0 || mcu_height <= 0) return RJPEG_DATA_ERROR;
    for (int mcu_y = 0; mcu_y < mcu_height; mcu_y++) {
        if (rjpeg_cancelled(decoder)) return RJPEG_CANCELLED;
        for (int mcu_x = 0; mcu_x < mcu_width; mcu_x++) {
            for (int slot = 0; slot < (int)scan->component_count; slot++) {
                int block_count = scan->component_count == 1u
                                      ? 1
                                      : active_components[slot].h_samp *
                                            active_components[slot].v_samp;
                if (block_count <= 0 ||
                    block_count > RJPEG_MAX_BLOCKS_PER_COMPONENT) {
                    return RJPEG_DATA_ERROR;
                }
                for (int block_index = 0; block_index < block_count;
                     block_index++) {
                    if (rjpeg_decode_block(decoder, block,
                                           &active_components[slot],
                                           publish_pixels) != RJPEG_OK) {
                        return RJPEG_DATA_ERROR;
                    }
                    if (publish_pixels) {
                        int block_x = scan->component_count == 1u
                                          ? mcu_x
                                          : mcu_x * active_components[slot].h_samp +
                                                block_index % active_components[slot].h_samp;
                        int block_y = scan->component_count == 1u
                                          ? mcu_y
                                          : mcu_y * active_components[slot].v_samp +
                                                block_index / active_components[slot].h_samp;
                        rjpeg_publish_component_block(
                            decoder, pixels, scan->component_index[slot],
                            max_horizontal_sampling, max_vertical_sampling,
                            block_x, block_y, block);
                    }
                }
            }
            decoded_mcus++;
            if (scan->restart_interval > 0 &&
                decoded_mcus % (uint32_t)scan->restart_interval == 0u &&
                !(mcu_y == mcu_height - 1 && mcu_x == mcu_width - 1)) {
                {
                    int restart_result = rjpeg_consume_restart(
                        decoder, active_components,
                        (int)scan->component_count, expected_restart);
                    if (restart_result != RJPEG_OK) return restart_result;
                }
                expected_restart = (uint8_t)((expected_restart + 1u) & 7u);
            }
        }
    }
    return rjpeg_finish_entropy(decoder, next_marker_out);
}

typedef struct {
    int32_t* coefficients;
    size_t coefficient_count;
    size_t component_offset[3];
    int padded_width[3];
    int padded_height[3];
    int scan_width[3];
    int scan_height[3];
} RJpegProgressiveWorkspace;

static inline int rjpeg_progressive_workspace_init(
    const RJpegDecoder* decoder, RJpegProgressiveWorkspace* workspace) {
    int max_horizontal_sampling;
    int max_vertical_sampling;
    int mcu_width;
    int mcu_height;
    size_t total_coefficients = 0u;
    if (!decoder || !workspace ||
        rjpeg_frame_sampling(decoder, &max_horizontal_sampling,
                             &max_vertical_sampling) != RJPEG_OK)
        return RJPEG_DATA_ERROR;
    mcu_width = (decoder->width + max_horizontal_sampling * 8 - 1) /
                (max_horizontal_sampling * 8);
    mcu_height = (decoder->height + max_vertical_sampling * 8 - 1) /
                 (max_vertical_sampling * 8);
    if (mcu_width <= 0 || mcu_height <= 0) return RJPEG_DATA_ERROR;
    for (int component = 0; component < decoder->num_components; component++) {
        const RJpegComponent* current = &decoder->comp[component];
        int component_width =
            (decoder->width * current->h_samp + max_horizontal_sampling - 1) /
            max_horizontal_sampling;
        int component_height =
            (decoder->height * current->v_samp + max_vertical_sampling - 1) /
            max_vertical_sampling;
        size_t block_count;
        workspace->component_offset[component] = total_coefficients;
        workspace->padded_width[component] = mcu_width * current->h_samp;
        workspace->padded_height[component] = mcu_height * current->v_samp;
        workspace->scan_width[component] = (component_width + 7) / 8;
        workspace->scan_height[component] = (component_height + 7) / 8;
        if (workspace->scan_width[component] <= 0 ||
            workspace->scan_height[component] <= 0 ||
            workspace->scan_width[component] >
                workspace->padded_width[component] ||
            workspace->scan_height[component] >
                workspace->padded_height[component])
            return RJPEG_DATA_ERROR;
        if ((size_t)workspace->padded_width[component] >
                SIZE_MAX / (size_t)workspace->padded_height[component])
            return RJPEG_UNSUPPORTED;
        block_count = (size_t)workspace->padded_width[component] *
                      (size_t)workspace->padded_height[component];
        if (block_count > (SIZE_MAX - total_coefficients) / 64u)
            return RJPEG_UNSUPPORTED;
        total_coefficients += block_count * 64u;
    }
    if (total_coefficients == 0u ||
        total_coefficients >
            (size_t)RJPEG_MAX_PROGRESSIVE_COEFFICIENT_BYTES /
                sizeof(*workspace->coefficients))
        return RJPEG_UNSUPPORTED;
    workspace->coefficient_count = total_coefficients;
    workspace->coefficients = (int32_t*)calloc(
        total_coefficients, sizeof(*workspace->coefficients));
    return workspace->coefficients ? RJPEG_OK : RJPEG_ERROR;
}

static inline int32_t* rjpeg_progressive_block(
    RJpegProgressiveWorkspace* workspace, int component, int block_x,
    int block_y) {
    size_t block_index;
    if (!workspace || component < 0 || component >= 3 || block_x < 0 ||
        block_y < 0 || block_x >= workspace->padded_width[component] ||
        block_y >= workspace->padded_height[component])
        return NULL;
    block_index = workspace->component_offset[component] +
                  ((size_t)block_y *
                       (size_t)workspace->padded_width[component] +
                   (size_t)block_x) *
                      64u;
    if (block_index > workspace->coefficient_count ||
        workspace->coefficient_count - block_index < 64u)
        return NULL;
    return workspace->coefficients + block_index;
}

static inline int rjpeg_progressive_read_eob_run(RJpegDecoder* decoder,
                                                  int run_bits,
                                                  uint32_t* run_out) {
    int extra = 0;
    if (!decoder || !run_out || run_bits < 0 || run_bits > 14 ||
        (run_bits != 0 && !rjpeg_get_bits(decoder, run_bits, &extra)))
        return 0;
    *run_out = (1u << (unsigned)run_bits) + (uint32_t)extra;
    return 1;
}

static inline int rjpeg_progressive_refine_coefficient(
    RJpegDecoder* decoder, int32_t* coefficient, int32_t bit_value) {
    int bit = 0;
    if (!decoder || !coefficient || bit_value <= 0 ||
        !rjpeg_get_bits(decoder, 1, &bit))
        return 0;
    if (bit == 0) return 1;
    if (((uint32_t)*coefficient & (uint32_t)bit_value) == 0u)
        *coefficient += *coefficient >= 0 ? bit_value : -bit_value;
    return 1;
}

static inline int rjpeg_decode_progressive_block(
    RJpegDecoder* decoder, const RJpegScan* scan, RJpegComponent* component,
    int32_t* coefficients, uint32_t* eob_run) {
    const int first = (int)scan->spectral_start;
    const int last = (int)scan->spectral_end;
    const int low = (int)scan->successive_low;
    const int32_t bit_value = (int32_t)1 << (unsigned)low;
    if (first == 0) {
        if (scan->successive_high == 0u) {
            int category = 0;
            int difference = 0;
            int64_t value;
            if (!rjpeg_huff_decode(decoder,
                                  &decoder->huff_dc[component->dc_id],
                                  &category) ||
                category < 0 || category > 11)
                return RJPEG_DATA_ERROR;
            if (category != 0) {
                if (!rjpeg_get_bits(decoder, category, &difference))
                    return RJPEG_DATA_ERROR;
                difference = rjpeg_extend(difference, category);
            }
            if ((difference > 0 &&
                 component->dc_pred > INT32_MAX - difference) ||
                (difference < 0 &&
                 component->dc_pred < INT32_MIN - difference))
                return RJPEG_DATA_ERROR;
            component->dc_pred += (int32_t)difference;
            value = (int64_t)component->dc_pred * bit_value;
            if (value < INT32_MIN || value > INT32_MAX)
                return RJPEG_DATA_ERROR;
            coefficients[0] = (int32_t)value;
        } else {
            int bit = 0;
            if (!rjpeg_get_bits(decoder, 1, &bit)) return RJPEG_DATA_ERROR;
            if (bit != 0 &&
                ((uint32_t)coefficients[0] & (uint32_t)bit_value) == 0u)
                coefficients[0] += bit_value;
        }
        return RJPEG_OK;
    }

    if (scan->successive_high == 0u) {
        int coefficient = first;
        if (*eob_run != 0u) {
            (*eob_run)--;
            return RJPEG_OK;
        }
        while (coefficient <= last) {
            int symbol = 0;
            int zeros;
            int size;
            if (!rjpeg_huff_decode(decoder,
                                  &decoder->huff_ac[component->ac_id],
                                  &symbol))
                return RJPEG_DATA_ERROR;
            zeros = (symbol >> 4) & 0x0f;
            size = symbol & 0x0f;
            if (size == 0) {
                if (zeros == 15) {
                    coefficient += 16;
                    if (coefficient > last + 1) return RJPEG_DATA_ERROR;
                    continue;
                }
                if (!rjpeg_progressive_read_eob_run(decoder, zeros,
                                                    eob_run) ||
                    *eob_run == 0u)
                    return RJPEG_DATA_ERROR;
                (*eob_run)--;
                break;
            }
            if (size > 10) return RJPEG_DATA_ERROR;
            coefficient += zeros;
            if (coefficient > last) return RJPEG_DATA_ERROR;
            {
                int bits = 0;
                int32_t value;
                if (!rjpeg_get_bits(decoder, size, &bits))
                    return RJPEG_DATA_ERROR;
                value = (int32_t)rjpeg_extend(bits, size) * bit_value;
                coefficients[g_rjpeg_zigzag[coefficient]] = value;
            }
            coefficient++;
        }
        return RJPEG_OK;
    }

    {
        int coefficient = first;
        int32_t new_value = 0;
        int has_new_value = 0;
        if (*eob_run == 0u) {
            int symbol = 0;
            int zeros;
            int size;
            if (!rjpeg_huff_decode(decoder,
                                  &decoder->huff_ac[component->ac_id],
                                  &symbol))
                return RJPEG_DATA_ERROR;
            zeros = (symbol >> 4) & 0x0f;
            size = symbol & 0x0f;
            if (size == 0 && zeros != 15) {
                if (!rjpeg_progressive_read_eob_run(decoder, zeros,
                                                    eob_run) ||
                    *eob_run == 0u)
                    return RJPEG_DATA_ERROR;
                while (coefficient <= last) {
                    int index = g_rjpeg_zigzag[coefficient];
                    if (coefficients[index] != 0 &&
                        !rjpeg_progressive_refine_coefficient(
                            decoder, &coefficients[index], bit_value))
                        return RJPEG_DATA_ERROR;
                    coefficient++;
                }
                (*eob_run)--;
                return RJPEG_OK;
            }
            if (size != 0) {
                int sign = 0;
                if (size != 1 || !rjpeg_get_bits(decoder, 1, &sign))
                    return RJPEG_DATA_ERROR;
                has_new_value = 1;
                new_value = sign != 0 ? bit_value : -bit_value;
            } else {
                zeros = 16;
            }
            while (coefficient <= last) {
                int index = g_rjpeg_zigzag[coefficient];
                if (coefficients[index] != 0) {
                    if (!rjpeg_progressive_refine_coefficient(
                            decoder, &coefficients[index], bit_value))
                        return RJPEG_DATA_ERROR;
                } else if (zeros > 0) {
                    zeros--;
                    if (zeros == 0) {
                        coefficient++;
                        break;
                    }
                } else {
                    break;
                }
                coefficient++;
            }
            if (zeros != 0) return RJPEG_DATA_ERROR;
            if (has_new_value) {
                if (coefficient > last ||
                    coefficients[g_rjpeg_zigzag[coefficient]] != 0)
                    return RJPEG_DATA_ERROR;
                coefficients[g_rjpeg_zigzag[coefficient]] = new_value;
                coefficient++;
            }
        }
        if (*eob_run != 0u) {
            while (coefficient <= last) {
                int index = g_rjpeg_zigzag[coefficient];
                if (coefficients[index] != 0 &&
                    !rjpeg_progressive_refine_coefficient(
                        decoder, &coefficients[index], bit_value))
                    return RJPEG_DATA_ERROR;
                coefficient++;
            }
            (*eob_run)--;
        }
    }
    return RJPEG_OK;
}

static inline int rjpeg_decode_progressive_scan(
    RJpegDecoder* decoder, const RJpegScan* scan,
    RJpegProgressiveWorkspace* workspace, uint16_t* next_marker_out) {
    RJpegComponent active_components[3] = RJPEG_ZERO_INIT;
    int max_horizontal_sampling;
    int max_vertical_sampling;
    int mcu_width;
    int mcu_height;
    uint32_t decoded_mcus = 0u;
    uint32_t eob_run = 0u;
    uint8_t expected_restart = 0u;
    uint8_t active_seen = 0u;
    if (!decoder || !scan || !workspace || !next_marker_out ||
        scan->component_count == 0u ||
        scan->component_count > (uint8_t)decoder->num_components ||
        rjpeg_frame_sampling(decoder, &max_horizontal_sampling,
                             &max_vertical_sampling) != RJPEG_OK)
        return RJPEG_DATA_ERROR;
    for (int slot = 0; slot < (int)scan->component_count; slot++) {
        int component = scan->component_index[slot];
        if (component < 0 || component >= decoder->num_components ||
            (active_seen & (uint8_t)(1u << (unsigned)component)) != 0u)
            return RJPEG_DATA_ERROR;
        active_seen |= (uint8_t)(1u << (unsigned)component);
        active_components[slot] = decoder->comp[component];
        active_components[slot].dc_id = scan->dc_id[slot];
        active_components[slot].ac_id = scan->ac_id[slot];
        active_components[slot].dc_pred = 0;
    }
    if (scan->component_count == 1u) {
        int component = scan->component_index[0];
        mcu_width = workspace->scan_width[component];
        mcu_height = workspace->scan_height[component];
    } else {
        if (scan->spectral_start != 0u) return RJPEG_DATA_ERROR;
        mcu_width = (decoder->width + max_horizontal_sampling * 8 - 1) /
                    (max_horizontal_sampling * 8);
        mcu_height = (decoder->height + max_vertical_sampling * 8 - 1) /
                     (max_vertical_sampling * 8);
    }
    if (mcu_width <= 0 || mcu_height <= 0) return RJPEG_DATA_ERROR;
    for (int mcu_y = 0; mcu_y < mcu_height; mcu_y++) {
        if (rjpeg_cancelled(decoder)) return RJPEG_CANCELLED;
        for (int mcu_x = 0; mcu_x < mcu_width; mcu_x++) {
            for (int slot = 0; slot < (int)scan->component_count; slot++) {
                int component = scan->component_index[slot];
                int block_count = scan->component_count == 1u
                                      ? 1
                                      : active_components[slot].h_samp *
                                            active_components[slot].v_samp;
                if (block_count <= 0 ||
                    block_count > RJPEG_MAX_BLOCKS_PER_COMPONENT)
                    return RJPEG_DATA_ERROR;
                for (int block_index = 0; block_index < block_count;
                     block_index++) {
                    int block_x = scan->component_count == 1u
                                      ? mcu_x
                                      : mcu_x * active_components[slot].h_samp +
                                            block_index %
                                                active_components[slot].h_samp;
                    int block_y = scan->component_count == 1u
                                      ? mcu_y
                                      : mcu_y * active_components[slot].v_samp +
                                            block_index /
                                                active_components[slot].h_samp;
                    int32_t* coefficients = rjpeg_progressive_block(
                        workspace, component, block_x, block_y);
                    if (!coefficients ||
                        rjpeg_decode_progressive_block(
                            decoder, scan, &active_components[slot],
                            coefficients, &eob_run) != RJPEG_OK)
                        return RJPEG_DATA_ERROR;
                }
            }
            decoded_mcus++;
            if (scan->restart_interval > 0 &&
                decoded_mcus % (uint32_t)scan->restart_interval == 0u &&
                !(mcu_y == mcu_height - 1 && mcu_x == mcu_width - 1)) {
                int restart_result;
                if (eob_run != 0u) return RJPEG_DATA_ERROR;
                restart_result = rjpeg_consume_restart(
                    decoder, active_components, (int)scan->component_count,
                    expected_restart);
                if (restart_result != RJPEG_OK) return restart_result;
                expected_restart = (uint8_t)((expected_restart + 1u) & 7u);
            }
        }
    }
    if (eob_run != 0u) return RJPEG_DATA_ERROR;
    return rjpeg_finish_entropy(decoder, next_marker_out);
}

static inline int rjpeg_decode_progressive_image_pass(
    RJpegDecoder* decoder, const RJpegScan* first_scan, uint32_t* pixels,
    int publish_pixels) {
    RJpegProgressiveWorkspace workspace = RJPEG_ZERO_INIT;
    RJpegScan scan;
    int result;
    if (!decoder || !first_scan || (publish_pixels != 0 && !pixels) ||
        (publish_pixels != 0 && publish_pixels != 1))
        return RJPEG_DATA_ERROR;
    result = rjpeg_progressive_workspace_init(decoder, &workspace);
    if (result != RJPEG_OK) return result;
    scan = *first_scan;
    for (;;) {
        uint16_t marker = 0;
        result = rjpeg_decode_progressive_scan(decoder, &scan, &workspace,
                                               &marker);
        if (result != RJPEG_OK) goto cleanup;
        if (marker == RJPEG_EOI) {
            if (decoder->pos != decoder->size) {
                result = RJPEG_DATA_ERROR;
                goto cleanup;
            }
            for (int component = 0; component < decoder->num_components;
                 component++) {
                if (decoder->progressive_approximation[component][0] < 0) {
                    result = RJPEG_DATA_ERROR;
                    goto cleanup;
                }
            }
            if (publish_pixels) {
                for (int component = 0; component < decoder->num_components;
                     component++) {
                    for (int block_y = 0;
                         block_y < workspace.padded_height[component];
                         block_y++) {
                        if (rjpeg_cancelled(decoder)) {
                            result = RJPEG_CANCELLED;
                            goto cleanup;
                        }
                        for (int block_x = 0;
                             block_x < workspace.padded_width[component];
                             block_x++) {
                            int32_t* coefficients = rjpeg_progressive_block(
                                &workspace, component, block_x, block_y);
                            int16_t block[64];
                            if (!coefficients) {
                                result = RJPEG_DATA_ERROR;
                                goto cleanup;
                            }
                            for (int coefficient = 0; coefficient < 64;
                                 coefficient++) {
                                if (coefficients[coefficient] < INT16_MIN ||
                                    coefficients[coefficient] > INT16_MAX) {
                                    result = RJPEG_DATA_ERROR;
                                    goto cleanup;
                                }
                                block[coefficient] =
                                    (int16_t)coefficients[coefficient];
                            }
                            rjpeg_idct(
                                block,
                                decoder->qt[decoder->comp[component].qt_id]
                                    .table);
                            rjpeg_publish_component_block(
                                decoder, pixels, component,
                                decoder->comp[0].h_samp,
                                decoder->comp[0].v_samp, block_x, block_y,
                                block);
                        }
                    }
                }
                result = rjpeg_finalize_pixels(decoder, pixels);
            } else {
                result = RJPEG_OK;
            }
            goto cleanup;
        }
        for (;;) {
            size_t end = 0u;
            if (marker == RJPEG_SOS) {
                if (!rjpeg_begin_segment(decoder, &end)) {
                    result = RJPEG_DATA_ERROR;
                    goto cleanup;
                }
                result = rjpeg_parse_sos(decoder, end, &scan);
                if (result != RJPEG_OK) goto cleanup;
                break;
            }
            if (marker == RJPEG_DHT) {
                if (!rjpeg_begin_segment(decoder, &end)) {
                    result = RJPEG_DATA_ERROR;
                    goto cleanup;
                }
                result = rjpeg_parse_dht(decoder, end);
            } else if (marker == RJPEG_DQT) {
                if (!rjpeg_begin_segment(decoder, &end)) {
                    result = RJPEG_DATA_ERROR;
                    goto cleanup;
                }
                result = rjpeg_parse_dqt(decoder, end);
            } else if (marker == RJPEG_DRI) {
                result = rjpeg_parse_dri(decoder);
            } else if ((marker >= RJPEG_APP0 && marker <= 0xffefu) ||
                       marker == RJPEG_COM) {
                if (!rjpeg_begin_segment(decoder, &end)) {
                    result = RJPEG_DATA_ERROR;
                    goto cleanup;
                }
                decoder->pos = end;
                result = RJPEG_OK;
            } else {
                result = RJPEG_DATA_ERROR;
                goto cleanup;
            }
            if (result != RJPEG_OK) goto cleanup;
            {
                int marker_status = rjpeg_next_marker(decoder, &marker);
                if (marker_status == RJPEG_CANCELLED) {
                    result = marker_status;
                    goto cleanup;
                }
                if (marker_status != 1) {
                    result = RJPEG_DATA_ERROR;
                    goto cleanup;
                }
            }
        }
    }
cleanup:
    if (workspace.coefficients) {
        volatile uint8_t* bytes = (volatile uint8_t*)workspace.coefficients;
        size_t byte_count = workspace.coefficient_count *
                            sizeof(*workspace.coefficients);
        for (size_t index = 0; index < byte_count; index++) bytes[index] = 0u;
    }
    free(workspace.coefficients);
    return result;
}

static inline int rjpeg_decode_image_pass(RJpegDecoder* decoder,
                                          const RJpegScan* first_scan,
                                          uint32_t* pixels,
                                          int publish_pixels) {
    RJpegScan scan;
    uint8_t seen_components = 0u;
    uint8_t expected_components;
    if (!decoder || !first_scan || (publish_pixels != 0 && !pixels) ||
        (publish_pixels != 0 && publish_pixels != 1) ||
        decoder->num_components <= 0 || decoder->num_components > 3) {
        return RJPEG_DATA_ERROR;
    }
    if (decoder->progressive)
        return rjpeg_decode_progressive_image_pass(
            decoder, first_scan, pixels, publish_pixels);
    scan = *first_scan;
    expected_components = (uint8_t)((1u << (unsigned)decoder->num_components) -
                                    1u);
    for (;;) {
        uint16_t marker = 0;
        uint8_t scan_components = 0u;
        int result;
        for (int slot = 0; slot < (int)scan.component_count; slot++) {
            int component = scan.component_index[slot];
            if (component < 0 || component >= decoder->num_components ||
                (scan_components &
                 (uint8_t)(1u << (unsigned)component)) != 0u ||
                (seen_components &
                 (uint8_t)(1u << (unsigned)component)) != 0u) {
                return RJPEG_DATA_ERROR;
            }
            scan_components |= (uint8_t)(1u << (unsigned)component);
        }
        result = rjpeg_decode_scan_pass(decoder, &scan, pixels,
                                        publish_pixels, &marker);
        if (result != RJPEG_OK) return result;
        seen_components |= scan_components;
        if (marker == RJPEG_EOI) {
            if (seen_components != expected_components ||
                decoder->pos != decoder->size) {
                return RJPEG_DATA_ERROR;
            }
            if (publish_pixels) {
                result = rjpeg_finalize_pixels(decoder, pixels);
                if (result != RJPEG_OK) return result;
            }
            return RJPEG_OK;
        }
        if (seen_components == expected_components) return RJPEG_DATA_ERROR;
        for (;;) {
            size_t end = 0u;
            if (marker == RJPEG_SOS) {
                if (!rjpeg_begin_segment(decoder, &end))
                    return RJPEG_DATA_ERROR;
                result = rjpeg_parse_sos(decoder, end, &scan);
                if (result != RJPEG_OK) return result;
                break;
            }
            if (marker == RJPEG_DHT) {
                if (!rjpeg_begin_segment(decoder, &end))
                    return RJPEG_DATA_ERROR;
                result = rjpeg_parse_dht(decoder, end);
            } else if (marker == RJPEG_DQT) {
                if (!rjpeg_begin_segment(decoder, &end))
                    return RJPEG_DATA_ERROR;
                result = rjpeg_parse_dqt(decoder, end);
            } else if (marker == RJPEG_DRI) {
                result = rjpeg_parse_dri(decoder);
            } else if ((marker >= RJPEG_APP0 && marker <= 0xffefu) ||
                       marker == RJPEG_COM) {
                if (!rjpeg_begin_segment(decoder, &end))
                    return RJPEG_DATA_ERROR;
                decoder->pos = end;
                result = RJPEG_OK;
            } else {
                return RJPEG_DATA_ERROR;
            }
            if (result != RJPEG_OK) return result;
            {
                int marker_status = rjpeg_next_marker(decoder, &marker);
                if (marker_status == RJPEG_CANCELLED) return marker_status;
                if (marker_status != 1) return RJPEG_DATA_ERROR;
            }
        }
    }
}

/* ═══════════════════════════════════════════════════════════════
 * 公開API
 * ═══════════════════════════════════════════════════════════════*/

/*
 * JPEG画像情報取得
 */
static inline void rjpeg_decoder_clear(RJpegDecoder* decoder) {
    uint8_t* bytes = (uint8_t*)decoder;
    size_t index;
    if (!decoder) return;
    for (index = 0u; index < sizeof(*decoder); ++index) bytes[index] = 0u;
}

static inline int rjpeg_get_info_cancellable_with_scratch(
                                               const uint8_t* data,
                                               size_t size,
                                               int* width,
                                               int* height,
                                               RJpegDecoder* decoder,
                                               RJpegCancellationFunction cancellation,
                                               void* cancellation_context) {
    if (width) *width = 0;
    if (height) *height = 0;
    if (!data || !width || !height || !decoder) return RJPEG_ERROR;
    rjpeg_decoder_clear(decoder);
    decoder->cancellation = cancellation;
    decoder->cancellation_context = cancellation_context;
    size_t scan_position = 0;
    RJpegScan first_scan = RJPEG_ZERO_INIT;
    int result = rjpeg_parse_headers(
        data, size, decoder, &scan_position, &first_scan);
    if (result != RJPEG_OK) return result;
    *width = decoder->width;
    *height = decoder->height;
    return RJPEG_OK;
}

static inline int rjpeg_get_info_with_scratch(const uint8_t* data,
                                               size_t size,
                                               int* width,
                                               int* height,
                                               RJpegDecoder* decoder) {
    return rjpeg_get_info_cancellable_with_scratch(
        data, size, width, height, decoder, NULL, NULL);
}

static inline int rjpeg_get_info_cancellable(
    const uint8_t* data, size_t size, int* width, int* height,
    RJpegCancellationFunction cancellation, void* cancellation_context) {
    RJpegDecoder decoder = RJPEG_ZERO_INIT;
    if (width) *width = 0;
    if (height) *height = 0;
    return rjpeg_get_info_cancellable_with_scratch(
        data, size, width, height, &decoder, cancellation,
        cancellation_context);
}

static inline int rjpeg_get_info(const uint8_t* data, size_t size,
                                  int* width, int* height) {
    RJpegDecoder decoder = RJPEG_ZERO_INIT;
    return rjpeg_get_info_with_scratch(data, size, width, height, &decoder);
}

/*
 * JPEGデコード（bounded grayscale／three-component profile）
 */
static inline int rjpeg_decode_cancellable_with_scratch(
                                            const uint8_t* data,
                                            size_t size,
                                            uint32_t* pixels,
                                            size_t pixel_capacity,
                                            int max_width,
                                            int max_height,
                                            RJpegDecoder* decoder,
                                            RJpegCancellationFunction cancellation,
                                            void* cancellation_context) {
    if (!data || !pixels || max_width <= 0 || max_height <= 0 || !decoder)
        return RJPEG_ERROR;
    rjpeg_decoder_clear(decoder);
    decoder->cancellation = cancellation;
    decoder->cancellation_context = cancellation_context;
    size_t scan_position = 0;
    RJpegScan first_scan = RJPEG_ZERO_INIT;
    int result = rjpeg_parse_headers(
        data, size, decoder, &scan_position, &first_scan);
    if (result != RJPEG_OK) return result;
    if (decoder->width > max_width || decoder->height > max_height)
        return RJPEG_ERROR;
    size_t pixel_count =
        (size_t)(unsigned)decoder->width *
        (size_t)(unsigned)decoder->height;
    if (pixel_count > pixel_capacity) return RJPEG_ERROR;

    /* First pass validates every entropy-coded block and every scan. */
    result = rjpeg_decode_image_pass(decoder, &first_scan, NULL, 0);
    if (result != RJPEG_OK) return result;

    /* Reparse so table/DRI mutations between scans are reproduced exactly. */
    rjpeg_decoder_clear(decoder);
    decoder->cancellation = cancellation;
    decoder->cancellation_context = cancellation_context;
    result = rjpeg_parse_headers(data, size, decoder, &scan_position,
                                 &first_scan);
    if (result != RJPEG_OK) return result;
    for (int row = 0; row < decoder->height; ++row) {
        if (rjpeg_cancelled(decoder)) return RJPEG_CANCELLED;
        memset(pixels + (size_t)row * (size_t)decoder->width, 0,
               (size_t)decoder->width * sizeof(*pixels));
    }

    /* The deterministic second pass is the only pass that publishes pixels. */
    return rjpeg_decode_image_pass(decoder, &first_scan, pixels, 1);
}

static inline int rjpeg_decode_with_scratch(const uint8_t* data,
                                            size_t size,
                                            uint32_t* pixels,
                                            size_t pixel_capacity,
                                            int max_width,
                                            int max_height,
                                            RJpegDecoder* decoder) {
    return rjpeg_decode_cancellable_with_scratch(
        data, size, pixels, pixel_capacity, max_width, max_height, decoder,
        NULL, NULL);
}

static inline int rjpeg_decode_cancellable(
    const uint8_t* data, size_t size, uint32_t* pixels,
    size_t pixel_capacity, int max_width, int max_height,
    RJpegCancellationFunction cancellation, void* cancellation_context) {
    RJpegDecoder decoder = RJPEG_ZERO_INIT;
    return rjpeg_decode_cancellable_with_scratch(
        data, size, pixels, pixel_capacity, max_width, max_height, &decoder,
        cancellation, cancellation_context);
}

static inline int rjpeg_decode(const uint8_t* data, size_t size,
                                uint32_t* pixels, size_t pixel_capacity,
                                int max_width, int max_height) {
    RJpegDecoder decoder = RJPEG_ZERO_INIT;
    return rjpeg_decode_with_scratch(data, size, pixels, pixel_capacity,
                                     max_width, max_height, &decoder);
}

#undef RJPEG_ZERO_INIT

#endif /* RINJPEG_H */
