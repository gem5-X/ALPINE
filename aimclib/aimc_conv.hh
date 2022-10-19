/* 
 * Copyright EPFL 2021
 * Joshua Klein
 * 
 * This file contains functions for mapping a convolutional layer as well as
 * performing an AIMC-enabled convolution.
 *
 * As an initial implementation, this part of the library is designed for
 * comatability with the CNN solver.
 *
 */

#ifndef __AIMC_CONV_HH__
#define __AIMC_CONV_HH__

#include "math.h"

#define DEBUG_CONV false

typedef float C_NN_t;
typedef float C_NN_tc;


/* Macromethod for performing the MVM portion of a convolution using AIMC core.
 *
 * NOTE: This method assumes in_buffer has already been padded.
 */
inline void
aimcConv2d(C_NN_tc * in_buffer, int c_in, int w_in, int h_in, C_NN_tc * output,
    int c_out, int w_out, int h_out, C_NN_t * bias, int kernel_width,
    int kernel_height, int stride, int padding, int tid)
{
    int n = 0;
    int kernel_size = kernel_width * kernel_height;
    w_in = w_in + 2 * padding;
    h_in = h_in + 2 * padding;
    int plane_size = w_in * h_in;
    int in_size = plane_size * c_in;

    C_NN_t * inputSlice = new C_NN_t[kernel_size * c_in];
    C_NN_t * outputSlice = new C_NN_t[c_out];
    C_NN_t max = 0;

#if DEBUG_CONV
    int iter_count = 0;
#endif

    // First find the maximum of the entire input.
    for (int i = 0; i < in_size; i++) {
        if (fabs(in_buffer[i]) > max) {
            max = fabs(in_buffer[i]);
        }
    }

    // Perform convolution, iterating over all output slices.
    for (int r = 0; r < h_out; r++) {
        for (int c = 0; c < w_out; c++) {
            // Grab input frame for AIMC queueing.
            for (int cin = 0; cin < c_in; cin++) {
                for (int i = 0; i < kernel_height; i++) {
                    for (int j = 0; j < kernel_width; j++) {
                        inputSlice[(cin * kernel_size) + (i * kernel_width + j)]
                            = in_buffer[cin * plane_size + r * stride * w_in +
                            c * stride + w_in * i + j];
                    }
                }
            }

            // Perform AIMC operations.
            queueVector(kernel_size, max, inputSlice, tid);  
            aimcProcess(tid);
            dequeueVector(c_out, max, outputSlice, tid);

#if DEBUG_CONV
            iter_count++;
#endif

            // Store output and apply biases.
            for (int i = 0; i < c_out; i++) {
                // TODO: Any transformations necessary here?
                output[n++] = bias[i] + outputSlice[i];
            }
        }
    }

#if DEBUG_CONV
    cout << "Performed " << iter_count << " AIMC loops for convolution.\n";
#endif

    // Clean up and return.
    delete[] inputSlice;
    delete[] outputSlice;

    return;
}


#endif // __AIMC_CONV_HH__
