// CRC-16 calculation using CCITT polynomial 0x11021.
// Peter J. 2026-04-10
//
// To build and test:
// $ gcc crc16_test.c crc16.c
// $ ./a.out
// bytes result= 15159
// words result= 15159
// $

#include <stdint.h>
#include <stdio.h>
#include "crc16.h"

int main(void)
{
    #define N 16
    uint8_t data[N] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                       0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f};
    #define M 8
    uint16_t words[M] = {0x0001, 0x0203, 0x0405, 0x0607,
                         0x0809, 0x0a0b, 0x0c0d, 0x0e0f};
    uint16_t result = crc16b(data, N, 0xffff);
    printf("bytes result= %d\n", result);
    uint16_t resultw = crc16w(words, M, 0xffff);
    printf("words result= %d\n", resultw);
}
