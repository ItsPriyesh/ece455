# include <stdio.h>

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

// Returns a success code: 0 = success, -1 = computations don't agree 
int sqrt_voting_system(int value, int *output) {
	static const int iters = 3;
	double results[iters];
	for (int i = 0; i < iters; i++) {
		results[i] = sqrt_newton_raphson(value);
	}

	// Check that at least two results agree
	if (approxEqual(results[0], results[1])) {
		*output = results[0];
		return 0;
	}
	if (approxEqual(results[0], results[2])) {
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

bool approxEqual(double a, double b) {
	static const double epsilon = 1e-6; 
	return abs(a - b) < epsilon;
}

int main(int argc, char const *argv[])
{
	struct RedundantInt red = write_int(5);
	printf("red.value = %d\n", red.value);
	int output = 0;
	int code = read_int(&red, &output);

	printf("code = %d, output = %d\n", code, output);
	return 0;
}