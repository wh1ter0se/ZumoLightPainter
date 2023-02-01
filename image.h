#include "lodepng.h"

#include <stdio.h>
#include <stdlib.h>

class PixelSegment {
    public:
    double dist;
    int pxCount;
    List<List<int>> px;

    PixelSegment(distInches, pixelCount, pixels);
    
}

/*
Example 1
Decode from disk to raw pixels with a single function call
*/
// THIS IS AN UNMODIFIED EXAMPLE
// We can modify this to load our PNG as an array of RGBA values
void decodePNG(const char* filename) {
  unsigned error;
  unsigned char* image = 0;
  unsigned width, height;

  error = lodepng_decode32_file(&image, &width, &height, filename);
  if(error) printf("error %u: %s\n", error, lodepng_error_text(error));

  /*use image here*/

  free(image);
}