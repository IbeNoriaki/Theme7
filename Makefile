CC=gcc
CFLAGS=-Wall -g -DFFT_PTHREAD -pthread 
ALL= stream

all: $(ALL)

stream: fftwstream.c
	$(CC) $(CFLAGS) $(LDFLAGS) fftwstream.c -lm -lfftw3 -o stream
clean: 
	rm -f $(ALL) *.o
