#include <cstdio>
#include <cstdlib>
int main() {
  FILE *f = fopen("assets/msyh.ttc", "rb");
  if (!f) return 1;
  fseek(f, 0, SEEK_END); long sz = ftell(f); fseek(f, 0, SEEK_SET);
  unsigned char *buf = (unsigned char*)malloc(sz);
  fread(buf, 1, sz, f); fclose(f);
  unsigned ver, count, off;
  ver = (buf[4]<<8)|buf[5]; count = (buf[6]<<8)|buf[7];
  off = (buf[12]<<24)|(buf[13]<<16)|(buf[14]<<8)|buf[15];
  printf("fonts=%u first_offset=%u\n", count, off);
  if (off >= (unsigned)sz) { printf("bad offset\n"); return 1; }
  FILE *out = fopen("assets/msyh_regular.ttf", "wb");
  fwrite(buf + off, 1, sz - off, out);
  fclose(out);
  printf("wrote %ld bytes\n", sz - off);
  free(buf);
  return 0;
}
