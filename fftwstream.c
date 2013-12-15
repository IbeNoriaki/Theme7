#include <stdio.h>
#include <stdlib.h>
#include <fftw3.h>
#include <math.h>

#define SIZE 1024

int main(int argc, char *argv[])
{
	/*
	if (argc < 2) {
		printf("Must specify file name\n");
		exit(EXIT_FAILURE);
	}
*/
	char *fileName = "data/thermo.dat";
	char fileBuf[100];
	//File parsing bitches
	FILE *fp = fopen(fileName, "r");
	int n;

	if(!fp) {
		printf("No file\n");
		return 0;
	}
	fscanf(fp, "; Sample Rate %d", &n);
	printf("Sample rate: %d\n", n);

	fgets(fileBuf, 100, fp);
	fgets(fileBuf, 100, fp);

	fftw_complex in[SIZE];
	fftw_complex out[SIZE];
	fftw_plan p;

	int i = 0;
	double x;
	char *tempBuf;
	double time, amplitude;
	/*
	while((tempBuf = fgets(fileBuf, 100, fp))!=NULL) {
		for(i = 0; i < SIZE; i++) {
			sscanf(fileBuf, "%*[ ]%lf%*[ ]%lf", &time, &amplitude);
			printf("Double rec: count: %d %lf, %lf\n", i, time, amplitude);
			in[i][0] = amplitude;
			in[i][1] = 0;
		}
	
		p = fftw_plan_dft_1d(i+1, in, out, FFTW_FORWARD, FFTW_ESTIMATE);
		fftw_execute(p);
		for (i = 0; i < SIZE; i++) {
			printf("Count: %d\t", i);
			x = sqrt(out[i][0] * out[i][0] + out[i][1] * out[i][1]);
			printf("Fourier: %f\n", x);
		}
	}
	*/
	while(1) {
		tempBuf = fgets(fileBuf, 100, fp);
		if(tempBuf) {
			sscanf(fileBuf, "%*[ ]%lf%*[ ]%lf", &time, &amplitude);
			printf("Double: count: %d %lf, %lf\n", i, time, amplitude);
			in[i][0] = amplitude;
			in[i][1] = 0;
		}
		i++;

		if(i==1024 || tempBuf == NULL) {
			p = fftw_plan_dft_1d(i, in, out, FFTW_FORWARD, FFTW_ESTIMATE);
			fftw_execute(p);

			for(i = 0; i < 1024; i++) {
				x = sqrt(out[i][0] * out[i][0] + out[i][1] * out[i][1]);
				printf("Fourier: count: %d\t%f\n", i, x);
			}

			//resetting i;
			i=0;
			printf("Restarting the fourier loop\n");

			if(!tempBuf)
				break;
			break;
		}
	}
	printf("Ended file loop\n");
	/*
	while((tempBuf = fgets(fileBuf, 100, fp))!=NULL) {
		printf("File: %s", fileBuf);
		double tempD, tempD1;
		sscanf(fileBuf, "%*[ ]%lf%*[ ]%lf", &tempD, &tempD1);
		printf("Double: count: %d %lf, %lf\n", i, tempD, tempD1);

		in[i][0] = tempD1;
		in[i][1] = 0;
		i++;

		if(i%1023 == 0) {	//Want to update the view every tenth of a sec approx
					// sample rate/10 ~~ 1024
			printf("Segfaults here:\n");
			p = fftw_plan_dft_1d(n, in, out, FFTW_FORWARD, FFTW_ESTIMATE);
			printf("Or here\n");
			fftw_execute(p);

			printf("or in this fking array");
			for (i = 0; i < 1024; i++) {
				x = sqrt(out[i][0] * out[i][0] + out[i][1] * out[i][1]);
				printf("Fourier: %f\n", x);
			}

			//resetting things
			in = startinfft;
			out = startoutfft;
			i=0;
		}
	}
	*/

	fftw_destroy_plan(p);
	/*fftw_free(in);
	printf("in?\n");
	fftw_free(out);
	printf("out?\n");
*/
	fclose(fp);

	return 0;
}
