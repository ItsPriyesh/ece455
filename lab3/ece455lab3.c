#include <stdio.h>
#include <stdbool.h>
#include <math.h>

struct RedundantInt {
    int value;
    int inverted;
};

// Returns a success code: 0 = success, -1 = fault detected 
int read_int(struct RedundantInt *redundant_int, int *output) {
    if (redundant_int->value == ~redundant_int->inverted) {
        *output = redundant_int->value;
        return 0;
    } else {
        return -1;
    }
}

// Returns a new RedundantInt as a value
struct RedundantInt write_int(int value) {
    struct RedundantInt res;
    res.value = value;
    res.inverted = ~value;
    return res;
}

// TODO
double sqrt_newton_raphson(int value) {
    return value;
}

double sqrt_binary_search(int value) {
    int start = 0;
    int end = value; 
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

bool approxEqual(double a, double b) {
    static const double epsilon = 1e-6; 
    return fabs(a - b) < epsilon;
}

// Returns a success code: 0 = success, -1 = computations don't agree 
int sqrt_voting_system(int value, int *output) {
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

int sqrt_heterogenous_computations(int value, int *output) {
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

int main(int argc, char const *argv[])
{
    struct RedundantInt red = write_int(5);
    printf("red.value = %d\n", red.value);
    int output = 0;
    int code = read_int(&red, &output);

    printf("code = %d, output = %d\n", code, output);

    printf("%f\n", sqrt_binary_search(69));
    return 0;
}