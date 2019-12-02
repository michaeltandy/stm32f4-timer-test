#include "matrix.h"
#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <assert.h>
#include <string.h> 
#include <stdbool.h>

#define CLOCK_RATE_MHZ (uint64_t)180
#define CYCLES_PER_MICROSECOND CLOCK_RATE_MHZ
#define ONE_BILLION 1000000000

#define REQUIRE(X) assert(X)

static void initResult(SimpleMatrix * result, uint8_t height, uint8_t width) {
    memset(result, 0, sizeof(SimpleMatrix));
    result->height = height;
    result->width = width;
}

void matrixAdd(SimpleMatrix * result, const SimpleMatrix * a, const SimpleMatrix * b)
{
    REQUIRE(a->height == b->height);
    REQUIRE(a->width == b->width);
    initResult(result, a->height, a->width);

    for (int i=0 ; i < a->height ; i++) {
        for (int j=0 ; j < a->width ; j++) {
            result->data[i][j] = a->data[i][j]+b->data[i][j];
        }
    }
}

void matrixSubtract(SimpleMatrix * result, const SimpleMatrix * a, const SimpleMatrix * b)
{
    REQUIRE(a->height == b->height);
    REQUIRE(a->width == b->width);
    initResult(result, a->height, a->width);
    
    for (int i=0 ; i < a->height ; i++) {
        for (int j=0 ; j < a->width ; j++) {
            result->data[i][j] = a->data[i][j]-b->data[i][j];
        }
    }
}

void matrixTranspose(SimpleMatrix * result, const SimpleMatrix * a)
{
    initResult(result, a->width, a->height);
    
    for (int i=0 ; i < a->height ; i++) {
        for (int j=0 ; j < a->width ; j++) {
            result->data[j][i] = a->data[i][j];
        }
    }
}

void matrixMultiply(SimpleMatrix * result, const SimpleMatrix * a, const SimpleMatrix * b)
{
    REQUIRE(a->width == b->height);
    initResult(result, a->height, b->width);
    
    for (int i=0 ; i < a->height ; i++) {
        for (int j=0 ; j < a->width ; j++) {
            result->data[i][j] = 0;
            for (int k=0 ; k<a->width ; k++) {
                result->data[i][j] += a->data[i][k]*b->data[k][j];
            }
        }
    }
}

static void invertOneByOne(SimpleMatrix * result, const SimpleMatrix * a)
{
    initResult(result, 1, 1);
    result->data[0][0] = 1.0f/a->data[0][0];
}

static void invertTwoByTwo(SimpleMatrix * result, const SimpleMatrix * mat)
{
    initResult(result, 2, 2);

    float a = mat->data[0][0];
    float b = mat->data[0][1];
    float c = mat->data[1][0];
    float d = mat->data[1][1];
        
    float det = 1.0f/(a*d-b*c);

    result->data[0][0] = det*d;
    result->data[0][1] = det*-b;
    result->data[1][0] = det*-c;
    result->data[1][1] = det*a;
}

void matrixInvert(SimpleMatrix * result, const SimpleMatrix * a)
{
    REQUIRE(a->width == a->height);
    
    if (a->width == 1) {
        invertOneByOne(result, a);
    } else if (a->width == 2) {
        invertTwoByTwo(result, a);
    } else {
         REQUIRE(0);
    }
}

bool matrixEqual(const SimpleMatrix * a, const SimpleMatrix * b)
{
    if(a->height != b->height || a->width != b->width)
        return false;

    for (int i=0 ; i < a->height ; i++) {
        for (int j=0 ; j < a->width ; j++) {
            if (a->data[i][j] != b->data[i][j])
                return false;
        }
    }
    return true;
}

void matrixCopy(SimpleMatrix * result, const SimpleMatrix * a)
{
    initResult(result, a->height, a->width);

    for (int i=0 ; i < a->height ; i++) {
        for (int j=0 ; j < a->width ; j++) {
            result->data[i][j] = a->data[i][j];
        }
    }
}


void matrixPrint(const SimpleMatrix * a)
{
    char buffer[255];
    int writeIdx = 0;
    writeIdx += snprintf(&buffer[writeIdx], sizeof(buffer)-writeIdx, "[\n");

    for (int i=0 ; i < a->height ; i++) {
        writeIdx += snprintf(&buffer[writeIdx], sizeof(buffer)-writeIdx, " [ ");
        for (int j=0 ; j < a->width ; j++) {
            writeIdx += snprintf(&buffer[writeIdx], sizeof(buffer)-writeIdx, "%f,\t", a->data[i][j]);
        }
        writeIdx -= 2;
        writeIdx += snprintf(&buffer[writeIdx], sizeof(buffer)-writeIdx, " ],\n");
    }
    writeIdx -= 2;
    writeIdx += snprintf(&buffer[writeIdx], sizeof(buffer)-writeIdx, "\n]\n");
    printf("%s",buffer);
}

int test()
{
    SimpleMatrix SQUARE_1234 = {{{1,2},{3,4}}, 2, 2};
    SimpleMatrix SQUARE_7654 = {{{7,6},{5,4}}, 2, 2};

    {
        SimpleMatrix expected = {{{2,4},{6,8}}, 2, 2};
        SimpleMatrix result = {0};
        matrixAdd(&result, &SQUARE_1234, &SQUARE_1234);
        matrixPrint(&result);
        REQUIRE(matrixEqual(&result, &expected));
    }

    {
        SimpleMatrix expected = {{{6,4},{2,0}}, 2, 2};
        SimpleMatrix result = {0};
        matrixSubtract(&result, &SQUARE_7654, &SQUARE_1234);
        matrixPrint(&result);
        REQUIRE(matrixEqual(&result, &expected));
    }

    {
        SimpleMatrix expected = {{{1,3},{2,4}}, 2, 2};
        SimpleMatrix result = {0};
        matrixTranspose(&result, &SQUARE_1234);
        matrixPrint(&result);
        REQUIRE(matrixEqual(&result, &expected));
    }
    
    {
        SimpleMatrix expected = {{{17, 14}, {41, 34}}, 2, 2};
        SimpleMatrix result = {0};
        matrixMultiply(&result, &SQUARE_1234, &SQUARE_7654);
        matrixPrint(&result);
        REQUIRE(matrixEqual(&result, &expected));
    }

    {
        SimpleMatrix expected = {{{-2, 1}, {3.0/2, -1.0/2}}, 2, 2};
        SimpleMatrix result = {0};
        matrixInvert(&result, &SQUARE_1234);
        matrixPrint(&result);
        REQUIRE(matrixEqual(&result, &expected));
    }

    {
        SimpleMatrix result = {0};
        matrixCopy(&result, &SQUARE_1234);
        matrixPrint(&result);
        REQUIRE(matrixEqual(&result, &SQUARE_1234));
    }

    return 0;
}

