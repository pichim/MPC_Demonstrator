#include "protocol.h"
#include <iostream>

void require(bool condition) {
    if (!condition) throw std::runtime_error("Protocol test failed");
}
void rejects(const protocol::Frame& frame) {
    try { protocol::decode(frame); }
    catch (const std::runtime_error&) { return; }
    throw std::runtime_error("Invalid reply accepted");
}
int main() {
    // Golden bytes generated independently by python/main.py (struct.pack <3f).
    const protocol::Frame read{87,0,0,0,0,0,0,0,0,0,0,0,0,97};
    const protocol::Frame command{85,10,215,163,61,0,0,128,63,0,0,0,0,142};
    const protocol::Frame reply{69,0,0,160,63,0,0,32,192,0,0,0,62,97};
    require(protocol::frame(protocol::Read) == read);
    require(protocol::frame(protocol::Command, {0.08f,1.0f,0.0f}) == command);
    require(protocol::decode(reply) == protocol::Measurements{1.25f,-2.5f,0.125f});
    rejects(command);
    for (unsigned byte = 0; byte < reply.size(); ++byte) {
        for (unsigned bit = 0; bit < 8; ++bit) {
            auto corrupt = reply;
            corrupt[byte] ^= 1u << bit;
            rejects(corrupt);
        }
    }
    for (unsigned field = 0; field < 3; ++field) {
        for (float bad : {std::numeric_limits<float>::infinity(),
                          -std::numeric_limits<float>::infinity(),
                          std::numeric_limits<float>::quiet_NaN()}) {
            protocol::Measurements values{};
            values[field] = bad;
            rejects(protocol::frame(protocol::Reply, values));
        }
    }
    std::cout << "PASS golden frames, single-bit corruption, invalid header, non-finite values\n";
}
