#include <stdint.h>

#include "goldseq_L1_32.h"

static inline uint16_t bitget(uint16_t x, uint16_t n)
{
    return ((x & (1 << n)) != 0) ? 1 : 0;
}

static inline uint16_t bitset(uint16_t x, uint16_t n, uint16_t val)
{
    val = val != 0;
    return (bitget(x, n) != val) ? (x ^ (1 << n)) : x;
}

void goldseq_L1_32(uint16_t n1, uint16_t n2, uint32_t *seq)
{
    int i;
    uint16_t G1 = 0xFFFF;
    uint16_t G2 = 0xFFFF;

    uint16_t G1temp;
    uint16_t G2temp;

    if (n1 > 10)
        G2 = n1;

    for (i = 0; i < 1023; i++)
    {
        if ((i & 0x1F) == 0)
        {
            seq[i >> 5] = 0;
        }
        if (n1 > 10)
        {
            seq[i >> 5] |= (bitget(G2, 10)) << (i & 0x1F);
        } else
        {
            seq[i >> 5] |= ((bitget(G1, 10) + bitget(G2, n1) + bitget(G2, n2)) % 2) << (i & 0x1F);
        }

        G1temp = (bitget(G1, 3) + bitget(G1, 10)) % 2;
        G2temp = bitget(G2, 2) + bitget(G2, 3) + bitget(G2, 6) + bitget(G2, 8) + bitget(G2, 9) + bitget(G2, 10);
        G2temp = G2temp % 2;

        G1 = (G1 << 1) % 0x10000;
        G1 = bitset(G1, 1, G1temp);
        G2 = (G2 << 1) % 0x10000;
        G2 = bitset(G2, 1, G2temp);

    }
}
