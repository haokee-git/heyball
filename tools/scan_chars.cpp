#include <cstdio>
#include <set>
int main() {
  std::set<int> chars;
  const char *files[] = {"src/main.cpp", "src/core.cpp", "src/core.hpp", nullptr};
  for (int fi = 0; files[fi]; ++fi) {
    FILE *f = fopen(files[fi], "rb");
    if (!f) continue;
    int c;
    while ((c = fgetc(f)) != EOF) {
      if (c >= 0x4E00 && c <= 0x9FFF) chars.insert(c);
      if (c >= 0x3000 && c <= 0x303F) chars.insert(c);
      if (c >= 0xFF00 && c <= 0xFF5F) chars.insert(c);  
    }
    fclose(f);
  }
  printf("count=%zu\n", chars.size());
  for (int c : chars) printf("0x%04X,", c);
  printf("\n");
  return 0;
}
