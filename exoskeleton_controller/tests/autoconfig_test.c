#include "autoconfig_schema.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

int main(void)
{
    char text[8192] = {0};
    FILE *file = fopen("tests/autoconfig_fixture.json", "rb");
    assert(file);
    size_t size = fread(text, 1, sizeof(text) - 1, file);
    fclose(file);
    saved_config c;
    assert(autoconfig_parse(text, size, &c));
    assert(c.bluetooth && c.manual_velocity < -12.29f && c.device.owner == -1);
    assert(c.angle.ratio == 2 && c.angle.direction == -1);
    assert(c.device.boards[0].high_pass == 200 && c.device.processing[1].hop == 128);
    char *gain = strstr(text, "\"ina_gain\":1");
    assert(gain);
    gain[11] = '3';
    assert(!autoconfig_parse(text, size, &c));
    gain[11] = '1';
    assert(autoconfig_parse(text, size, &c));
    char *version = strstr(text, "\"schema_version\":2");
    assert(version);
    version[17] = '1';
    assert(autoconfig_parse(text, size, &c));
    assert(!c.bluetooth && c.manual_velocity == 0 && c.angle.ratio == 1);
    char nested[40];
    memset(nested, '[', 17);
    memset(nested + 17, ']', 17);
    nested[34] = 0;
    assert(!autoconfig_json_bounded(nested, 34));
    assert(!autoconfig_json_bounded("{\"x\":0}\0garbage", 16));
    assert(!autoconfig_parse("{} trailing", 11, &c));
    puts("Autoconfig v1/v2, gains, Bluetooth persistence, deep JSON and trailing bytes passed");
    return 0;
}
