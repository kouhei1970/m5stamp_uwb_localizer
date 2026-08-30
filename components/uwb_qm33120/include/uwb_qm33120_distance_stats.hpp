/**
 * @file uwb_qm33120_distance_stats.hpp
 * @brief Online mean/standard-deviation accumulator for distance samples
 * (Welford's algorithm), shared between firmware/anchor's main.cpp and
 * uwb::ResponderStats (docs/ARCHITECTURE_V2.md §2.3).
 *
 * 1:1 with the `DistanceStats` struct that used to be defined locally in
 * both firmware/anchor/main/main.cpp and firmware/twr/main/main.cpp
 * (identical copies, per firmware/anchor's own comment "firmware/twr/main/
 * main.cpp の DistanceStats と同一実装"). Moved here so
 * components/uwb_qm33120's new Responder (§2.2/§2.3) can reuse the exact
 * same running mean/std logic instead of a third copy, per
 * docs/ARCHITECTURE_V2.md's instruction to move it to a shared header.
 * firmware/anchor/main/main.cpp now aliases `DistanceStats` to
 * `uwb::DistanceStats` instead of defining its own (see that file). twr is
 * left untouched (its own local copy still compiles standalone).
 *
 * ESP-IDF/Qorvo-SDK-free (only <cstdint> and <cmath>), so it can be included
 * from uwb_qm33120_responder_fsm.hpp (host-testable, tests/host/) as well
 * as from ordinary firmware application code.
 */
#pragma once

#include <cmath>
#include <cstdint>

namespace uwb {

/**
 * @brief Online mean/standard-deviation of distance samples (mm), via
 * Welford's algorithm - does not keep every sample, so it is safe for a
 * long-running anchor/tag.
 */
struct DistanceStats {
    uint32_t count = 0;
    double mean    = 0.0;
    double m2      = 0.0;

    void add(float distanceMm)
    {
        count++;
        const double delta = static_cast<double>(distanceMm) - mean;
        mean += delta / static_cast<double>(count);
        const double delta2 = static_cast<double>(distanceMm) - mean;
        m2 += delta * delta2;
    }

    double stddev() const
    {
        return (count < 2) ? 0.0 : std::sqrt(m2 / static_cast<double>(count - 1));
    }
};

} // namespace uwb
