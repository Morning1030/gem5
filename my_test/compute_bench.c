#include <stdio.h>
#include <stdint.h>

#define ITERS 1000000UL

int main(void)
{
    volatile uint64_t a = 1;
    volatile uint64_t b = 2;
    volatile uint64_t c = 3;

    for (uint64_t i = 0; i < ITERS; i++) {
        a = a + b;
        b = b ^ c;
        c = c + a;
    }

    printf("Compute tiny benchmark finished.\n");
    printf("Result: %lu %lu %lu\n", a, b, c);

    return 0;
}
