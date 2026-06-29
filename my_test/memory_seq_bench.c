#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#define N (1 * 1024 * 1024)   // 1M elements
#define REPEAT 2

int main()
{
    int *a = (int *)malloc(sizeof(int) * N);
    int *b = (int *)malloc(sizeof(int) * N);
    int *c = (int *)malloc(sizeof(int) * N);

    if (a == NULL || b == NULL || c == NULL) {
        printf("Memory allocation failed.\n");
        return 1;
    }
    for (uint64_t i = 0; i < N; i++) {
        a[i] = (int)i;
        b[i] = (int)(i * 2);
        c[i] = 0;
    }

    volatile uint64_t checksum = 0;

    for (int r = 0; r < REPEAT; r++) {
        for (uint64_t i = 0; i < N; i++) {
            c[i] = a[i] + b[i];
        }

        for (uint64_t i = 0; i < N; i++) {
            checksum += c[i];
        }
    }

    printf("Sequential memory benchmark finished.\n");
    printf("Checksum: %lu\n", checksum);

    free(a);
    free(b);
    free(c);

    return 0;
}
