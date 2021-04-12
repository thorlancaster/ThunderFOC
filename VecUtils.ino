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
