// Host-side driver for firmware/bot/logic.h. Reads events on stdin, prints outputs on stdout:
//   rx <ms> <hex bytes>   feed one packet through parse_packet + Safety::on_command
//   tick <ms>             print "<left_permille> <right_permille> <marker>"
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include "logic.h"

int main(int argc, char** argv) {
  int bot_id = argc > 1 ? atoi(argv[1]) : 1;
  swarm::Safety safety;
  char kind[8];
  unsigned long ms;
  char hex[256];
  while (scanf("%7s %lu", kind, &ms) == 2) {
    if (kind[0] == 'r') {
      if (scanf("%255s", hex) != 1) return 2;
      std::vector<char> bytes;
      std::string h = hex;
      if (h != "-") {  // "-" encodes an empty datagram
        for (size_t i = 0; i + 1 < h.size(); i += 2) bytes.push_back((char)strtol(h.substr(i, 2).c_str(), nullptr, 16));
      }
      swarm::Command c;
      if (swarm::parse_packet(bytes.data(), bytes.size(), bot_id, c)) safety.on_command(c, (uint32_t)ms);
    } else {
      swarm::Outputs o = safety.tick((uint32_t)ms);
      printf("%d %d %d\n", o.left_permille, o.right_permille, (int)o.marker);
    }
  }
  return 0;
}
