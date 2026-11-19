#include "serial_frame.h"
#include <stdio.h>
#include <string.h>
#ifdef ESP_PLATFORM
#include "esp_rom_crc.h"
#endif

uint32_t serial_crc(uint32_t crc, const void *data, size_t size)
{
#ifdef ESP_PLATFORM
    return esp_rom_crc32_le(crc, data, size);
#else
    const uint8_t *p = data;
    crc = ~crc;
    while (size--) {
        crc ^= *p++;
        for (unsigned n = 0; n < 8; ++n) {
            crc = (crc >> 1) ^ ((crc & 1) ? 0xEDB88320u : 0);
        }
    }
    return ~crc;
#endif
}

static const char alphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
static bool session_valid(char c)
{
    return (c >= '0' && c <= '9') || (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z');
}
static int digit(char c)
{
    if (c >= '0' && c <= '9') {
        return c - '0';
    }
    if (c >= 'A' && c <= 'F') {
        return c - 'A' + 10;
    }
    return -1;
}
static bool hex(const char *s, unsigned count, uint32_t *value)
{
    *value = 0;
    for (unsigned n = 0; n < count; ++n) {
        int d = digit(s[n]);
        if (d < 0) {
            return false;
        }
        *value = (*value << 4) | (unsigned)d;
    }
    return true;
}
static int base64_value(char c)
{
    if (c >= 'A' && c <= 'Z') {
        return c - 'A';
    }
    if (c >= 'a' && c <= 'z') {
        return c - 'a' + 26;
    }
    if (c >= '0' && c <= '9') {
        return c - '0' + 52;
    }
    return c == '+' ? 62 : c == '/' ? 63 : -1;
}

size_t serial_frame_encode(const serial_frame *f, char out[SERIAL_FRAME_MAX])
{
    if (!session_valid(f->session) || f->start > f->end || f->end > f->total ||
        f->end - f->start > SERIAL_CHUNK_MAX) {
        return 0;
    }
    int header = snprintf(out, SERIAL_FRAME_MAX, "GOON|1|%c|%08lX|%04X|%04X|%04X|00000000|",
                          f->session, (unsigned long)f->id, f->start, f->end, f->total);
    if (header != 42) {
        return 0;
    }
    size_t cursor = 42, count = f->end - f->start;
    for (size_t n = 0; n < count; n += 3) {
        unsigned remain = (unsigned)(count - n);
        uint32_t v = (uint32_t)f->data[n] << 16;
        if (remain > 1) {
            v |= (uint32_t)f->data[n + 1] << 8;
        }
        if (remain > 2) {
            v |= f->data[n + 2];
        }
        out[cursor++] = alphabet[v >> 18];
        out[cursor++] = alphabet[(v >> 12) & 63];
        out[cursor++] = remain > 1 ? alphabet[(v >> 6) & 63] : '=';
        out[cursor++] = remain > 2 ? alphabet[v & 63] : '=';
    }
    uint32_t crc = serial_crc(0, out, 33);
    crc = serial_crc(crc, out + 41, cursor - 41);
    char check[9];
    snprintf(check, sizeof(check), "%08lX", (unsigned long)crc);
    memcpy(out + 33, check, 8);
    out[cursor++] = '\n';
    return cursor;
}

bool serial_frame_decode(const char *s, size_t size, serial_frame *f)
{
    if (size < 43 || size > SERIAL_FRAME_MAX || s[size - 1] != '\n' || memcmp(s, "GOON|1|", 7) ||
        !session_valid(s[7])) {
        return false;
    }
    const unsigned separators[] = {8, 17, 22, 27, 32, 41};
    for (unsigned n = 0; n < sizeof(separators) / sizeof(separators[0]); ++n) {
        if (s[separators[n]] != '|') {
            return false;
        }
    }
    uint32_t start, end, total, check;
    if (!hex(s + 9, 8, &f->id) || !hex(s + 18, 4, &start) || !hex(s + 23, 4, &end) ||
        !hex(s + 28, 4, &total) || !hex(s + 33, 8, &check) || start > end || end > total ||
        end - start > SERIAL_CHUNK_MAX) {
        return false;
    }
    size_t count = end - start, encoded = 4 * ((count + 2) / 3);
    if (size != 43 + encoded) {
        return false;
    }
    uint32_t crc = serial_crc(0, s, 33);
    crc = serial_crc(crc, s + 41, size - 42);
    if (crc != check) {
        return false;
    }
    size_t written = 0;
    for (size_t n = 0; n < encoded; n += 4) {
        const char *p = s + 42 + n;
        int a = base64_value(p[0]), b = base64_value(p[1]);
        int c = p[2] == '=' ? 0 : base64_value(p[2]), d = p[3] == '=' ? 0 : base64_value(p[3]);
        if (a < 0 || b < 0 || c < 0 || d < 0 || (p[2] == '=' && p[3] != '=') ||
            ((p[2] == '=' || p[3] == '=') && n + 4 != encoded) || (p[2] == '=' && (b & 15)) ||
            (p[3] == '=' && p[2] != '=' && (c & 3))) {
            return false;
        }
        uint32_t value = (unsigned)a << 18 | (unsigned)b << 12 | (unsigned)c << 6 | (unsigned)d;
        if (written < count) {
            f->data[written++] = (uint8_t)(value >> 16);
        }
        if (p[2] != '=' && written < count) {
            f->data[written++] = (uint8_t)(value >> 8);
        }
        if (p[3] != '=' && written < count) {
            f->data[written++] = (uint8_t)value;
        }
        unsigned produced = p[2] == '=' ? 1 : p[3] == '=' ? 2 : 3;
        if ((n / 4) * 3 + produced > count) {
            return false;
        }
    }
    if (written != count) {
        return false;
    }
    f->session = s[7];
    f->start = (uint16_t)start;
    f->end = (uint16_t)end;
    f->total = (uint16_t)total;
    return true;
}

int serial_assemble(serial_assembly *s, const serial_frame *f)
{
    if (!f->total || f->total > sizeof(s->data) || f->end > f->total || f->start >= f->end ||
        f->end - f->start > SERIAL_CHUNK_MAX) {
        return -1;
    }
    if (!s->active || s->id != f->id) {
        if (s->active && !s->delivered) {
            return -1;
        }
        s->id = f->id;
        s->total = f->total;
        s->received = 0;
        s->active = true;
        s->delivered = false;
        memset(s->coverage, 0, (f->total + 7u) / 8u);
    }
    if (s->total != f->total || f->start >= f->end) {
        return -1;
    }
    for (uint32_t n = f->start; n < f->end; ++n) {
        unsigned bit = 1u << (n & 7);
        if (s->coverage[n >> 3] & bit) {
            if (s->data[n] != f->data[n - f->start]) {
                return -1;
            }
        } else {
            s->coverage[n >> 3] |= (uint8_t)bit;
            s->data[n] = f->data[n - f->start];
            ++s->received;
        }
    }
    if (s->delivered) {
        return 2;
    }
    if (s->received == s->total) {
        s->delivered = true;
        return 1;
    }
    return 0;
}
