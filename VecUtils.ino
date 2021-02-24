Vector3 addVectors(Vector3 v1, Vector3 v2){
  Vector3 output;
  output.x = v1.x + v2.x;
  output.y = v1.y + v2.y;
  output.z = v1.z + v2.z;
  return output;
}

Vector3 multiplyScalar(Vector3 in, float scale) {
  in.x *= scale;
  in.y *= scale;
  in.z *= scale;
  return in;
}


void copyLUT(float* src, float* dest, int sz){
  for(int x = 0; x < sz * 3; x++){
    dest[x] = src[x];
  }
}

// Constrain a float array to be between 0 and 1
// Can normalize in place (src == dest)
void normalize(float* src, float* dest, int sz) {
  float maxVal = -1000000000000000;
  float minVal = 1000000000000000;
  for (int x = 0; x < sz; x++) {
    float v = src[x];
    maxVal = max(maxVal, v);
    minVal = min(minVal, v);
  }
  if (minVal == maxVal) { // Case of all values equal - output all zeroes
    for (int x = 0; x < sz; x++)
      dest[x] = 0;
    return;
  }
  float scale = 1 / (maxVal - minVal);
  for (int x = 0; x < sz; x++) {
    dest[x] = ((src[x] - minVal) * scale);
  }
}

// Force a LUT to satisfy a+b+c == 0
// Can compute in place (src == dest)
void zeroLUT(float* src, float* dest, int lutSz) {
  for (int i = 0; i < lutSz; i++) {
    float x = src[i * 3];
    float y = src[i * 3 + 1];
    float z = src[i * 3 + 2];
    float avg = (x + y + z) / 3;
    x -= avg;
    y -= avg;
    z -= avg;
    dest[i * 3] = x;
    dest[i * 3 + 1] = y;
    dest[i * 3 + 2] = z;
  }
}

// Compress a LUT so it can fit in EEPROM
// LUT will be zeroed and normalized during compression
// Uses interpolation and quantization for compression
// To (approximately) reverse this, use loadLUT()
void saveLUT(float* lut, int lutSz, byte* dest, int destSz) {
  //  float temp[lutSz * 3];
  zeroLUT(lut, lut, lutSz);
  normalize(lut, lut, lutSz * 3);
  int i = 0;
  for (int j = 0; j < destSz; j++) {
    int thisStepEnd = (j + 1) * (lutSz * 1.0f / destSz);
    // Averaging
    float x = 0;
    float y = 0;
    float z = 0;
    int num = 0;
    for (; i < thisStepEnd && i < lutSz; i++) {
      x += lut[i * 3];
      y += lut[i * 3 + 1];
      z += lut[i * 3 + 2];
      num++;
    }
    x /= num;
    y /= num;
    z /= num;
    float avg = (x+y+z) / 3;
    x += (0.5 - avg);
    y += (0.5 - avg);
    z += (0.5 - avg);

    // Quantization into 3 bytes (12 bit precision)
    // MSB xxxxxxxx xxxxyyyy yyyyyyyy LSB
    //    Serial.printf("X and Y: %f and %f\n", x, y);
    int ix = (x + (1 / 8192)) * 4096;
    int iy = (y + (1 / 8192)) * 4096;
    ix = constrain(ix, 0, 4095);
    iy = constrain(iy, 0, 4095);
    dest[j * 3] = (ix >> 4) & 0xFF;
    dest[j * 3 + 1] = ((ix & 0x0F) << 4) | ((iy >> 8) & 0x0F);
    dest[j * 3 + 2] = iy & 0xFF;
  }
  zeroLUT(lut, lut, lutSz);
}

// Expand a LUT compressed by saveLUT
// Note that compSz and lutSz are numbers of vectors, not bytes
void loadLUT(byte* comp, int compSz, float* lut, int lutSz) {
  for (int i = 0; i < lutSz; i++) {
    float compIdx = i * (compSz * 1.0f / lutSz);
    int c = ceil(compIdx);
    c %= compSz;
    int f = floor(compIdx);
    float pct = fmod(compIdx, 1);
//    Serial.printf("Index: %d, Lower: %d, Upper: %d, pct: %f\n",i, f, c, pct);
    int xLow = comp[f * 3] << 4 | ((comp[f * 3 + 1] >> 4) & 0x0F);
    int yLow = (comp[f * 3 + 1] & 0x0F) << 8 | comp[f * 3 + 2];
    int xHigh = comp[c * 3] << 4 | ((comp[c * 3 + 1] >> 4) & 0x0F);
    int yHigh = (comp[c * 3 + 1] & 0x0F) << 8 | comp[c * 3 + 2];
    float x = (xHigh * pct + xLow * (1 - pct)) / 4096.0f;
    float y = (yHigh * pct + yLow * (1 - pct)) / 4096.0f;
    lut[i * 3] = x;
    lut[i * 3 + 1] = y;
    lut[i * 3 + 2] = 1.5 - (x + y);
  }
  zeroLUT(lut, lut, lutSz);
}
