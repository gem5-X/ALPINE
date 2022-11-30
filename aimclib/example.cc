/* Copyright EPFL 2022
 * Joshua Klein
 * 
 * Example single 1k*1k perceptron layer using AIMClib using random weights and
 * inputs.  An example of how to compile this example is included within
 * build_example.sh.  The GCC/G++ build command line determines if the AIMClib
 * checker (gem5 model emulation) is used or not.
 *
 */

#include "aimc.hh"

int main(int argc, char * argv[])
{
    // Test bench parameters.
    int n_x         = 1024; // MLP input/output dimensions.
    int T_x         = 10;   // Number of inferences.

    // Set up and initialize vectors/matrices.
    int8_t ** input     = new int8_t*[T_x];
    int8_t *  W1        = new int8_t[n_x*n_x];
    int8_t ** output    = new int8_t*[T_x];
    for (int i = 0; i < T_x; i++) {
        input[i] = new int8_t[n_x];
        output[i] = new int8_t[n_x];
        for (int j = 0; j < n_x; j++) {
            input[i][j] = (int8_t)rand();
            output[i][j] = (int8_t)rand();
        }
    }
    for (int i = 0; i < n_x*n_x; i++)
        W1[i] = (int8_t)rand();

    // Map weights to AIMC tile.
    mapMatrix(0, 0, n_x, n_x, W1);

    // Do inference.
    for (int i = 0; i < T_x; i++) 
    {
        // Queue input for next inference in first layer.
        queueVector(n_x, input[i]);
        
        // Do MVM.
        aimcProcess();
        
        // Dequeue output from AIMC tile MVM.
        dequeueVector(n_x, output[i]);
    }
    
    // Cleanup and return.
    for (int i = 0; i < T_x; i++) {
        delete[] input[i];
        delete[] output[i];
    }
    delete[] input;
    delete[] output;
    delete[] W1;

    return 0;
}
