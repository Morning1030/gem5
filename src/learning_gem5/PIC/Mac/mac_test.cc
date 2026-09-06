#include "learning_gem5/PIC/Mac/mac_test.hh"

#include <array>
#include <cstddef>
#include <cstdint>
#include <random>

#include "base/logging.hh"
#include "learning_gem5/PIC/Mac/pic_mac.hh"
#include "sim/sim_exit.hh"

namespace gem5
{
namespace pic
{

MacTest::MacTest(const MacTestParams &params)
    : ClockedObject(params), randomCases(params.random_cases)
{
}

uint32_t
MacTest::goldenPopcount(uint64_t value)
{
    uint32_t count = 0;
    for (unsigned bit = 0; bit < 64; ++bit)
        count += static_cast<uint32_t>((value >> bit) & 1ULL);
    return count;
}

uint32_t
MacTest::goldenMac(uint64_t left, uint64_t right,
                   uint8_t bias, bool negative)
{
    const uint8_t rtlBias = bias & 0x0f;
    const uint32_t count = goldenPopcount(left & right);
    const uint32_t shifted = static_cast<uint32_t>(count << rtlBias);
    return negative ? static_cast<uint32_t>(0u - shifted) : shifted;
}

void
MacTest::runPrimitiveDirectedTests()
{
    struct Case {
        uint64_t left;
        uint64_t right;
        uint8_t bias;
        bool negative;
        uint32_t expected;
    };

    const std::array<Case, 8> cases = {{
        {0x0ULL, 0xffffffffffffffffULL, 0, false, 0u},
        {0x1ULL, 0x1ULL, 0, false, 1u},
        {0xfULL, 0xfULL, 3, false, 32u},
        {0xfULL, 0xfULL, 3, true, 0xffffffe0u},
        {0xaaaaaaaaaaaaaaaaULL, 0x5555555555555555ULL, 7, false, 0u},
        {0xffffffffffffffffULL, 0xffffffffffffffffULL, 0, false, 64u},
        {0xffffffffffffffffULL, 0xffffffffffffffffULL, 15, false, 0x00200000u},
        {0xffff0000ffff0000ULL, 0xffffffffffffffffULL, 2, false, 128u},
    }};

    for (std::size_t i = 0; i < cases.size(); ++i) {
        const auto &c = cases[i];
        const uint32_t got = macPrimitive(c.left, c.right, c.bias, c.negative);
        panic_if(got != c.expected,
                 "%s MAC directed case %zu failed: expected=%#x got=%#x",
                 name(), i, c.expected, got);
    }

    inform("%s MAC primitive directed PASS: %zu cases", name(), cases.size());
}

void
MacTest::runMatDirectedTests()
{
    const uint64_t leftVec = 0xffffffffffffffffULL;
    std::array<MacArrayInput, 4> arrays = {{
        {0x1ULL, 0, false, true},
        {0x3ULL, 1, false, true},
        {0xfULL, 2, true, true},
        {0xffULL, 3, false, false},
    }};

    const uint32_t expected = static_cast<uint32_t>(1u + 4u + (0u - 16u));
    const uint32_t got = matMacSum(leftVec, arrays);
    panic_if(got != expected,
             "%s Mat MAC sum failed: expected=%#x got=%#x",
             name(), expected, got);

    arrays[3].enable = true;
    const uint32_t gotAllFour = matMacSum(leftVec, arrays);
    panic_if(gotAllFour != 53u,
             "%s four-array Mat MAC sum failed: expected=53 got=%u",
             name(), gotAllFour);

    inform("%s Mat four-PolyArray MAC sum PASS", name());
}

void
MacTest::runRandomTests()
{
    std::mt19937_64 rng(0x5049434d4143554cULL);
    for (uint64_t i = 0; i < randomCases; ++i) {
        const uint64_t left = rng();
        const uint64_t right = rng();
        const uint8_t bias = static_cast<uint8_t>(rng() & 0x0f);
        const bool negative = (rng() & 1ULL) != 0;

        const uint32_t expected = goldenMac(left, right, bias, negative);
        const uint32_t got = macPrimitive(left, right, bias, negative);
        panic_if(got != expected,
                 "%s MAC random case %llu failed: expected=%#x got=%#x",
                 name(), static_cast<unsigned long long>(i), expected, got);
    }

    inform("%s MAC randomized golden PASS: %llu cases",
           name(), static_cast<unsigned long long>(randomCases));
}

void
MacTest::startup()
{
    runPrimitiveDirectedTests();
    runMatDirectedTests();
    runRandomTests();
    inform("%s POLYMORPIC MAC FULL PASS", name());
    exitSimLoop(name() + " POLYMORPIC MAC FULL PASS");
}

} // namespace pic
} // namespace gem5
