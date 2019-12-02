#include <stdint.h>
#include <inttypes.h>
#include <stdbool.h>

#define REQUIRE(X) assert(X)

typedef struct
{
    float data[2][2];
    uint8_t height;
    uint8_t width;
} SimpleMatrix;

void matrixAdd(SimpleMatrix * result, const SimpleMatrix * a, const SimpleMatrix * b);
void matrixSubtract(SimpleMatrix * result, const SimpleMatrix * a, const SimpleMatrix * b);
void matrixTranspose(SimpleMatrix * result, const SimpleMatrix * a);
void matrixMultiply(SimpleMatrix * result, const SimpleMatrix * a, const SimpleMatrix * b);
void matrixInvert(SimpleMatrix * result, const SimpleMatrix * a);
bool matrixEqual(const SimpleMatrix * a, const SimpleMatrix * b);
void matrixCopy(SimpleMatrix * result, const SimpleMatrix * b);
void matrixPrint(const SimpleMatrix * a);

