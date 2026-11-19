#include "serial_frame.h"
#include <assert.h>
#include <stdio.h>

int main(void)
{
    serial_assembly assembly = {0};
    serial_frame f = {.session = '1', .id = 1, .start = 0, .end = 351, .total = 1025};
    assert(sizeof(assembly.data) == 1024);
    assert(serial_assemble(&assembly, &f) == -1);
    f.total = 1024;
    assert(serial_assemble(&assembly, &f) == 0);
    f.start = 351;
    f.end = 702;
    assert(serial_assemble(&assembly, &f) == 0);
    f.start = 702;
    f.end = 1024;
    assert(serial_assemble(&assembly, &f) == 1);
    puts("Firmware 1024-byte assembly capacity and oversized-frame rejection passed");
    return 0;
}
