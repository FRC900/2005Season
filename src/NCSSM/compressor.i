#line 1 "compressor.c"
#line 2 "compressor.c"



static struct {
	unsigned isAtPressure:1;
	unsigned isPressureLow:1;
	unsigned runCompressor:1;
	unsigned :5;
} compressor_data;

void compressor_setPressureSensor( char isPressureLow ) {
}
