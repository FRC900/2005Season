#define _compressor_c

#include "compressor.h"

static struct {
	unsigned isAtPressure:1;
	unsigned isPressureLow:1;
	unsigned runCompressor:1;
	unsigned :5;
} compressor_data;

void compressor_setPressureSensor( char isPressureLow ) {
}
