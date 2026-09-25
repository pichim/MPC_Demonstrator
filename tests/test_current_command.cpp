#include "CurrentCommand.h"
#include <cassert>
#include <limits>

int main()
{
    constexpr uint32_t timeout = 300000;
    CurrentCommand command;
    assert(!command.enabled && command.current == 0.0f);
    const float enabled[] = {0.08f, 1.0f, 0.0f};
    const float disabled[] = {0.08f, 0.0f, 0.0f};

    command.accept(enabled, 1000);
    command.expire(1000 + timeout - 1, timeout);
    assert(command.enabled && command.current == 0.08f);
    command.expire(1000 + timeout, timeout);
    assert(!command.enabled && command.current == 0.0f);
    // A later ticker wrap must not revive an expired command.
    command.expire(1000, timeout);
    assert(!command.enabled);

    // A newly consumed but already old command must expire from reception time.
    command.accept(enabled, 1000);
    command.expire(1000 + timeout + 1, timeout);
    assert(!command.enabled);

    // A fresh command replaces the held one and establishes its own expiry.
    command.accept(enabled, 500000);
    command.expire(500000 + timeout - 1, timeout);
    assert(command.enabled);
    command.accept(enabled, 700000);
    command.expire(500000 + timeout, timeout);
    assert(command.enabled);
    command.expire(700000 + timeout, timeout);
    assert(!command.enabled);

    const uint32_t near_wrap = UINT32_MAX - 100;
    command.accept(enabled, near_wrap);
    command.expire(near_wrap + timeout - 1, timeout);
    assert(command.enabled);
    command.expire(near_wrap + timeout, timeout);
    assert(!command.enabled);

    command.accept(enabled, 0);
    command.accept(disabled, 1);
    assert(!command.enabled && command.current == 0.0f);
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    const float invalid[][3] = {
        {nan, 1, 0}, {inf, 1, 0}, {-inf, 1, 0},
        {0.08f, nan, 0}, {0.08f, 2, 0}, {0.08f, -1, 0},
        {0.08f, 1, nan}, {0.08f, 1, 1}
    };
    for (const auto &values : invalid) {
        command.accept(enabled, 0);
        command.accept(values, 1);
        assert(!command.enabled && command.current == 0.0f);
    }
}
