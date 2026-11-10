/*!
 * \file rfsoc_uhd_filter_design_test.cc
 * \brief Checks that the RFSoC signal source decimation filters meet their
 *  specification: passband ripple and stopband attenuation measured on the
 *  designed taps, for the stage plans of the example configurations.
 * \author Wei Cheng, 2026.
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2026  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "rfsoc_uhd_filter_design.h"
#include "rfsoc_uhd_utils.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <complex>
#include <vector>

namespace
{
double magnitude_db(const std::vector<float>& taps, double f_hz, double fs_hz)
{
    std::complex<double> h(0.0, 0.0);
    const double w = -2.0 * M_PI * f_hz / fs_hz;
    for (size_t n = 0; n < taps.size(); n++)
        {
            h += static_cast<double>(taps[n]) * std::polar(1.0, w * static_cast<double>(n));
        }
    return 20.0 * std::log10(std::max(std::abs(h), 1e-30));
}

struct Response
{
    double max_passband_dev_db;  // |gain in dB| over the passband
    double max_stopband_db;      // highest gain over the stopband
};

Response measure(const rfsoc_uhd::Decimation_Stage& st, const std::vector<float>& taps)
{
    constexpr int points = 2000;
    Response r{0.0, -1e9};
    for (int i = 0; i <= points; i++)
        {
            const double f = st.passband_edge_hz * i / points;
            r.max_passband_dev_db = std::max(r.max_passband_dev_db, std::abs(magnitude_db(taps, f, st.input_rate_sps)));
        }
    const double stop_end = st.input_rate_sps / 2.0;
    for (int i = 0; i <= points; i++)
        {
            const double f = st.stopband_edge_hz + (stop_end - st.stopband_edge_hz) * i / points;
            r.max_stopband_db = std::max(r.max_stopband_db, magnitude_db(taps, f, st.input_rate_sps));
        }
    return r;
}

void check_plan(double device, double out, double pass)
{
    constexpr double atten = rfsoc_uhd::DEFAULT_STOPBAND_ATTENUATION_DB;
    constexpr double ripple = rfsoc_uhd::DEFAULT_PASSBAND_RIPPLE_DB;
    const auto plan = rfsoc_uhd::plan_decimation(device, out, pass);
    ASSERT_FALSE(plan.empty());
    for (size_t s = 0; s < plan.size(); s++)
        {
            const auto& st = plan[s];
            const std::vector<float> taps = rfsoc_uhd::design_stage_taps(st, atten, ripple);
            EXPECT_EQ(taps.size(), rfsoc_uhd::estimate_taps(st, atten, ripple));
            const Response r = measure(st, taps);
            // everything that folds onto the passband is at least ~atten down
            EXPECT_LE(r.max_stopband_db, -(atten - 1.0))
                << "stage " << s << " /" << st.factor << " at " << st.input_rate_sps << " sps, " << taps.size() << " taps";
            EXPECT_LE(r.max_passband_dev_db, ripple + 0.05)
                << "stage " << s << " /" << st.factor << " at " << st.input_rate_sps << " sps, " << taps.size() << " taps";
        }
}
}  // namespace


TEST(RfsocUhdFilterDesignTest, GpsL1PlanMeetsSpecification)
{
    check_plan(122.88e6, 3.84e6, 1.5e6);
}


TEST(RfsocUhdFilterDesignTest, GpsL5PlanMeetsSpecification)
{
    check_plan(122.88e6, 20.48e6, 9.0e6);
}


TEST(RfsocUhdFilterDesignTest, OtherRatesMeetSpecification)
{
    check_plan(122.88e6, 4.096e6, 1.6e6);
    check_plan(125.0e6, 5.0e6, 2.0e6);
    check_plan(120.0e6, 4.0e6, 1.6e6);
}
