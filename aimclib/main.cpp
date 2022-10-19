/* Copyright EPFL 2021
 * Joshua Klein
 * 
 * Simple main.cpp tester program for AIMC library functions.
 * 
 * Compile with:
 * > g++ -std=c++17 -O3 main.cpp
 * 
 */

#include <bitset>
#include <iostream>
#include <stdlib.h>
#include <string>

#include "aimc.hh"

#define LIMIT 10
#define TEST_INT8 true
#define TEST_DOUBLE !TEST_INT8
#define CHECK_CONV2D true

using namespace std;


void
checkNormal()
{
    cout << "Doing normal intrinsics check...\n";

    uint32_t tmp;
    int8_t val;

    // Write params to AIMC core.
#if TEST_INT8
    int8_t ** aimcMatrix = new int8_t*[LIMIT];

    for (int i = 0; i < LIMIT; i++) {
        aimcMatrix[i] = new int8_t[LIMIT];

        for (int j = 0; j < LIMIT; j++) {
            aimcMatrix[i][j] = i+j;
        }
    }

    mapMatrix(1, 1, LIMIT, LIMIT, aimcMatrix);
#elif TEST_DOUBLE

#endif

#if USE_CHECKER
    aimc.printCrossbar(0);
#endif

    // Write to input memory.
#if TEST_INT8
    int8_t inputArray[] = {1, -1, 3, 12, -15, 0, -2, 0, 1};
    queueVector(sizeof(inputArray) / sizeof(inputArray)[0], inputArray);
#elif TEST_DOUBLE
    double max = 163534623.432423;
    double inputArray[] = {2313.4532, -21432321.231312, 6356532.134542, 2312.2222, 99999999.2324, max};
    queueVector(sizeof(inputArray) / sizeof(inputArray)[0], max, inputArray);
#endif

#if USE_CHECKER
    aimc.printInputMemory(0);
#endif

    // Perform MVM.
    aimcProcess();

#if USE_CHECKER
    aimc.printOutputMemory(0);
#endif

    // Dequeue and confirm output.
    cout << "Dequeued array:\n";

#if TEST_INT8
    int8_t outputArray[9];
    dequeueVector(9, outputArray);
    for (auto item : outputArray) {
        cout << (int)item << '\t';
    } cout << endl;
#elif TEST_DOUBLE
    double outputArray[9];
    dequeueVector(9, max, outputArray);
    for (auto item : outputArray) {
        cout << item << '\t';
    } cout << endl;
#endif

    // Cleanup.
#if TEST_INT8
    for (int i = 0; i < LIMIT; i++) {
        delete[] aimcMatrix[i];
    } delete[] aimcMatrix;
#elif TEST_DOUBLE

#endif

    return;
}


void
checkConvolution()
{
    cout << "Doing Convolution extension check...\n";

    // Convolution setup (emulating first layer of AlexNet).
    int weights_h = 363;
    int weights_w = 96;
    int weights_size = weights_h * weights_w;
    int8_t * weights = new int8_t[weights_size];
    for (int i = 0; i < weights_size; i++) weights[i] = (int8_t)rand();

    int in_d = 3;
    int in_h = 227 + 1;
    int in_w = 227 + 1;
    int in_size = in_d * in_h * in_w;
    float * in_buffer = new float[in_size];
    for (int i = 0; i < in_size; i++) in_buffer[i] = (float)rand();

    int out_d = 96;
    int out_h = 55;
    int out_w = 55;
    int out_size = out_d * out_h * out_w;
    float * output = new float[out_size];
    for (int i = 0; i < out_size; i++) output[i] = (float)rand();

    float * bias = new float[out_size];
    for (int i = 0; i < out_size; i++) bias[i] = 1 / (float)rand();

    int kernel_h = 11;
    int kernel_w = 11;

    int stride = 4;
    int padding = 0;

    // Map convolutional layer to AIMC core.
    mapMatrix(0, 0, weights_h, weights_w, weights);

#if USE_CHECKER
    aimc.printCrossbar(0);
#endif

    // Perform convolution.
    cout << "Starting convolution...\n";
    Conv2d(in_buffer, in_d, in_w, in_h, output, out_d, out_w, out_h, bias,
        kernel_w, kernel_h, stride, padding);
    cout << "Convolution finished!\n";

    // Print output (55 x 55 x 1 slice).
    int output_layer_to_print_l = 3;
    int output_layer_to_print_u = 4;

    cout << "Output segments:\n";
    for (int k = 0; k < out_d; k++) {
        if (output_layer_to_print_l <= k && k <= output_layer_to_print_u) {
            cout << "Layer " << k << ":\n";
            for (int i = 0; i < out_h; i++) {
                for (int j = 0; j < out_w; j++) {
                    cout << output[(i * out_w + j) + (k * out_h * out_w)] << "\t";
                } cout << endl;
            } cout << endl;
        }
    }

    // Cleanup and return.
    delete[] bias;
    delete[] in_buffer;
    delete[] output;
    delete[] weights;

    return;
}

int
main(int argc, char * argv[])
{
    cout << "Starting AIMC check program...\n";

#if CHECK_CONV2D
    checkConvolution();
#else
    checkNormal();
#endif

    // Finished!
    cout << "Done!\n";

    return 0;
}
