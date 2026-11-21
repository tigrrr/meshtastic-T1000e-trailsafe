// Host test for ButtonMessageLogic.h: g++ -std=c++17 -I overlay/src/modules/optional/ButtonMessage test/test_logic.cpp
#include "ButtonMessageLogic.h"
#include <cassert>
#include <cstdio>

using namespace bm;

int main()
{
    char out[64];
    bool crit = true;
    const char *list = "Check-in OK|!HELP - need assistance|I am OK";
    assert(messageAt(list, 0, out, sizeof out, &crit) && !strcmp(out, "Check-in OK") && !crit);
    assert(messageAt(list, 1, out, sizeof out, &crit) && !strcmp(out, "HELP - need assistance") && crit);
    assert(messageAt(list, 2, out, sizeof out, &crit) && !strcmp(out, "I am OK") && !crit);
    assert(!messageAt(list, 3, out, sizeof out, &crit));
    assert(!messageAt("", 0, out, sizeof out, &crit));
    assert(!messageAt("a||b", 1, out, sizeof out, &crit));
    assert(messageAt("abcdef", 0, out, 4, &crit) && !strcmp(out, "abc")); // truncation

    const uint32_t GUARD = 10000, ACK = 60000, RETRY = 120000, MAX = 3;
    Logic l(GUARD, ACK, RETRY, MAX);

    // Button events are ignored until the boot guard expires, except after a button wake.
    assert(!l.acceptsClicks(GUARD - 1) && l.acceptsClicks(GUARD));
    assert(Logic(0, ACK, RETRY, MAX).acceptsClicks(0));

    // A mesh ACK confirms the broadcast and releases the wait.
    assert(l.onMessage(false, 20000) == Action::SEND && l.state == State::WAIT);
    assert(l.onEvent(Event::NONE) == Action::NONE && l.state == State::WAIT);
    assert(l.nextWakeMs(20001) == ACK - 1);
    assert(l.onEvent(Event::ACK) == Action::TONE_ACK && l.state == State::IDLE);
    assert(l.nextWakeMs(20002) == UINT32_MAX);

    // A normal message times out without an ACK.
    assert(l.onMessage(false, 30000) == Action::SEND);
    assert(l.onTick(30000 + ACK - 1) == Action::NONE);
    assert(l.onTick(30000 + ACK) == Action::TONE_FAIL && l.state == State::IDLE);

    // A NAK ends a normal attempt immediately.
    assert(l.onMessage(false, 40000) == Action::SEND);
    assert(l.onEvent(Event::NAK) == Action::TONE_FAIL && l.state == State::IDLE);

    // A NAK leaves critical retries active. A mesh ACK ends them.
    assert(l.onMessage(true, 50000) == Action::SEND && l.state == State::CRITICAL && l.attempts == 1);
    assert(l.onEvent(Event::NAK) == Action::TONE_FAIL && l.state == State::CRITICAL);
    assert(l.onTick(50000 + RETRY - 1) == Action::NONE);
    assert(l.onTick(50000 + RETRY) == Action::RESEND && l.attempts == 2);
    assert(l.onEvent(Event::ACK) == Action::TONE_ACK && l.state == State::IDLE);

    // critical: a new send replaces it
    assert(l.onMessage(true, 60000) == Action::SEND && l.state == State::CRITICAL);
    assert(l.onMessage(false, 60500) == Action::SEND && l.state == State::WAIT && !l.critical);
    assert(l.onEvent(Event::ACK) == Action::TONE_ACK);

    // critical: max attempts then fail
    uint32_t t = 70000;
    assert(l.onMessage(true, t) == Action::SEND);
    for (uint32_t i = 1; i < MAX; i++) {
        t += RETRY;
        assert(l.onTick(t) == Action::RESEND && l.attempts == i + 1);
    }
    t += RETRY;
    assert(l.onTick(t) == Action::TONE_FAIL && l.state == State::IDLE);

    // events while idle are ignored
    assert(l.onEvent(Event::ACK) == Action::NONE);

    puts("test_logic: ok");
    return 0;
}
