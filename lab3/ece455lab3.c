#include <stdio.h>
#include <stdbool.h>
#include <math.h>

// Floating point equality: a == b iff |a-b| < epsilon
bool approxEqual(double a, double b) {
    static const double epsilon = 1e-6; 
    return fabs(a - b) < epsilon;
}

/********** DATA REDUNDANCY **********/

struct RedundantInt {
    int value;
    int inverted;
};

// Returns a new RedundantInt as a value
void write_int(struct RedundantInt *redundant_int, int value) {
    redundant_int->value = value;
    redundant_int->inverted = ~value;
}

// Returns a success code: 0 = success, -1 = fault detected 
int read_int(struct RedundantInt *redundant_int, int *output) {
    if (redundant_int->value == ~redundant_int->inverted) {
        *output = redundant_int->value;
        return 0;
    } else {
        return -1;
    }
}

struct RedundantDouble {
    double value;
    long inverted;
};

void write_double(struct RedundantDouble *redundant_double, double value) {
    redundant_double->value = value;
    redundant_double->inverted = ~((long) trunc(1e6 * value));
}

// Returns a success code: 0 = success, -1 = fault detected 
int read_double(struct RedundantDouble *redundant_double, double *output) {
    if (redundant_double->value == (~redundant_double->inverted / 1e6)) {
        *output = redundant_double->value;
        return 0;
    } else {
        return -1;
    }
}

/********** SQRT IMPLEMENTATIONS **********/

double sqrt_newton_raphson(double value) {
    double curr = value;
    while (!approxEqual(curr * curr, value)) {
        curr = (curr + value / curr) / 2.0;
    }
    return curr;
}

double sqrt_binary_search(double value) {
    int start = 0;
    int end = (int) floor(value); 
    int mid; 
    double res;
  
    while (start <= end) { 
        mid = (start + end) / 2; 
        if (mid * mid == value) { 
            res = mid; 
            break; 
        } 
  
        if (mid * mid < value) { 
            start = mid + 1; 
            res = mid; 
        } else { 
            end = mid - 1; 
        } 
    } 
  
    static const int precision = 6;
    double inc = 0.1;
    for (int i = 0; i < precision; i++) { 
        while (res * res <= value) { 
            res += inc; 
        } 
  
        res -= inc; 
        inc /= 10; 
    } 
    return res;
}


/********** VOTING SYSTEM **********/

// Returns a success code: 0 = success, -1 = computations don't agree 
int sqrt_voting_system(double value, double *output) {
    static const int iters = 3;
    double results[iters];
    for (int i = 0; i < iters; i++) {
        results[i] = sqrt_newton_raphson(value);
    }

    // Check that at least two results agree
    if (approxEqual(results[0], results[1]) || approxEqual(results[0], results[2])) {
        *output = results[0];
        return 0;
    }
    if (approxEqual(results[1], results[2])) {
        *output = results[1];
        return 0;
    }
    
    // No agreement
    return -1;
}

/********** HETEROGENEOUS COMPUTATIONS **********/

// Returns a success code: 0 = success, -1 = computations don't agree 
int sqrt_heterogenous_computations(int value, double *output) {
    double res1 = sqrt(value);
    double res2 = sqrt_newton_raphson(value);
    double res3 = sqrt_binary_search(value);

    if (approxEqual(res1, res2) || approxEqual(res1, res3)) {
        *output = res1;
        return 0;
    }
    if (approxEqual(res2, res3)) {
        *output = res2;
        return 0;
    }

    // No agreement
    return -1;
}

/********** RESULT VERIFICATION **********/

// Returns a success code: 0 = verification success, -1 = verification failed
int sqrt_result_verification(double (*sqrt)(double), double value, double *output) {
    double res = (*sqrt)(value);
    if (approxEqual(res * res, value)) {
        *output = res;
        return 0;
    }
    return -1;
}

int main(int argc, char const *argv[]) {
    struct RedundantInt redundant_int;
    write_int(&redundant_int, 5);
    int output_i = 0;
    int code = read_int(&redundant_int, &output_i);
    printf("code = %d, output = %d\n", code, output_i);

    struct RedundantDouble redundant_double;
    write_double(&redundant_double, 3.14159);
    double output_d = 0;
    code = read_double(&redundant_double, &output_d);
    printf("code = %d, output = %f\n", code, output_d);

    double output;
    code = sqrt_voting_system(16, &output);
    printf("code = %d, output = %f\n", code, output);

    code = sqrt_heterogenous_computations(16, &output);
    printf("code = %d, output = %f\n", code, output);

    code = sqrt_result_verification(sqrt_newton_raphson, 16, &output);
    printf("code = %d, output = %f\n", code, output);
    return 0;
}