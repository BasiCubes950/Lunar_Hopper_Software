#ifndef D4_TEST_ARDUINO_H
#define D4_TEST_ARDUINO_H
#include <string>
unsigned long millis();
struct MockSerial {
  std::string input;
  std::string output;
  void begin(int) {}
  int available() { return static_cast<int>(input.size()); }
  int read() { char c = input.front(); input.erase(0, 1); return c; }
  void print(const char* s) { output += s; }
  void println(const char* s) { output += s; output += '\n'; }
  void println(int n) { output += std::to_string(n); output += '\n'; }
};
extern MockSerial Serial;
#endif
