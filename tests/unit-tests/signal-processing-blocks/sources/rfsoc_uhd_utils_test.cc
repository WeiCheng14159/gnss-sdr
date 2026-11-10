/*!
 * \file rfsoc_uhd_utils_test.cc
 * \brief Unit tests for the RFSoC (uhd-rfsoc) signal source helpers:
 *  decimation planning, first-Nyquist-zone checks and UHD device arguments.
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

#include "rfsoc_uhd_utils.h"
#include <gtest/gtest.h>
#include <functional>
#include <numeric>
#include <stdexcept>
#include <vector>

namespace
{
unsigned int product_of(const std::vector<unsigned int>& v)
{
    return std::accumulate(v.cbegin(), v.cend(), 1U, std::multiplies<unsigned int>());
}
}  // namespace


// ---- decimation factor ------------------------------------------------------

TEST(RfsocUhdUtilsTest, DecimationFactorIntegerRatio)
{
    EXPECT_EQ(rfsoc_uhd::decimation_factor(122.88e6, 3.84e6), 32U);
    EXPECT_EQ(rfsoc_uhd::decimation_factor(122.88e6, 4.096e6), 30U);
    EXPECT_EQ(rfsoc_uhd::decimation_factor(122.88e6, 122.88e6), 1U);
    EXPECT_EQ(rfsoc_uhd::decimation_factor(125.0e6, 5.0e6), 25U);
}


TEST(RfsocUhdUtilsTest, DecimationFactorRejectsNonIntegerRatio)
{
    // 122.88 / 4 = 30.72: not an integer
    EXPECT_THROW(rfsoc_uhd::decimation_factor(122.88e6, 4.0e6), std::invalid_argument);
}


TEST(RfsocUhdUtilsTest, DecimationFactorRejectsHugeRatio)
{
    // would overflow an unsigned int if cast blindly
    EXPECT_THROW(rfsoc_uhd::decimation_factor(122.88e6, 0.01), std::invalid_argument);
    EXPECT_THROW(rfsoc_uhd::decimation_factor(122.88e6, 1.0), std::invalid_argument);
}


TEST(RfsocUhdUtilsTest, EstimateTapsFollowsSpecification)
{
    const rfsoc_uhd::Decimation_Stage wide{8, 122.88e6, 15.36e6, 1.5e6, 13.86e6};
    const rfsoc_uhd::Decimation_Stage sharp{4, 15.36e6, 3.84e6, 1.5e6, 1.92e6};
    const unsigned int n_wide = rfsoc_uhd::estimate_taps(wide, 60.0, 0.1);
    const unsigned int n_sharp = rfsoc_uhd::estimate_taps(sharp, 60.0, 0.1);
    EXPECT_GT(n_sharp, n_wide);
    EXPECT_GT(rfsoc_uhd::estimate_taps(sharp, 80.0, 0.1), n_sharp);
    EXPECT_EQ(n_wide % 2, 1U);  // odd: type I linear phase, integer group delay
    EXPECT_EQ(n_sharp % 2, 1U);
    EXPECT_GE(n_wide, 3U);
}


TEST(RfsocUhdUtilsTest, DecimationFactorRejectsUpsamplingAndBadRates)
{
    EXPECT_THROW(rfsoc_uhd::decimation_factor(3.84e6, 122.88e6), std::invalid_argument);
    EXPECT_THROW(rfsoc_uhd::decimation_factor(0.0, 1.0e6), std::invalid_argument);
    EXPECT_THROW(rfsoc_uhd::decimation_factor(122.88e6, 0.0), std::invalid_argument);
    EXPECT_THROW(rfsoc_uhd::decimation_factor(122.88e6, -1.0), std::invalid_argument);
}


// ---- splitting a decimation into stages --------------------------------------

TEST(RfsocUhdUtilsTest, SplitDecimationPowerOfTwo)
{
    const std::vector<unsigned int> s = rfsoc_uhd::split_decimation(32, 8);
    EXPECT_EQ(s, (std::vector<unsigned int>{8, 4}));
}


TEST(RfsocUhdUtilsTest, SplitDecimationMixedPrimes)
{
    const std::vector<unsigned int> s = rfsoc_uhd::split_decimation(30, 8);
    EXPECT_EQ(product_of(s), 30U);
    for (unsigned int f : s)
        {
            EXPECT_LE(f, 8U);
            EXPECT_GE(f, 2U);
        }
    // largest stage first: it runs at the highest rate with the widest transition
    for (size_t i = 1; i < s.size(); i++)
        {
            EXPECT_GE(s[i - 1], s[i]);
        }
}


TEST(RfsocUhdUtilsTest, SplitDecimationOneMeansNoStage)
{
    EXPECT_TRUE(rfsoc_uhd::split_decimation(1, 8).empty());
}


TEST(RfsocUhdUtilsTest, SplitDecimationPrimeAboveLimitIsOneStage)
{
    // 11 cannot be split: one stage of 11 (larger than the limit) is the only option
    EXPECT_EQ(rfsoc_uhd::split_decimation(11, 8), (std::vector<unsigned int>{11}));
    EXPECT_EQ(rfsoc_uhd::split_decimation(22, 8), (std::vector<unsigned int>{11, 2}));
}


TEST(RfsocUhdUtilsTest, SplitDecimationRejectsZero)
{
    EXPECT_THROW(rfsoc_uhd::split_decimation(0, 8), std::invalid_argument);
    EXPECT_THROW(rfsoc_uhd::split_decimation(32, 1), std::invalid_argument);
}


// ---- full decimation plan ----------------------------------------------------

TEST(RfsocUhdUtilsTest, PlanFromFactorsRatesChain)
{
    const auto plan = rfsoc_uhd::plan_from_factors(122.88e6, 1.5e6, {8, 4});
    ASSERT_EQ(plan.size(), 2U);
    EXPECT_EQ(plan[0].factor, 8U);
    EXPECT_DOUBLE_EQ(plan[0].input_rate_sps, 122.88e6);
    EXPECT_DOUBLE_EQ(plan[0].output_rate_sps, 15.36e6);
    EXPECT_DOUBLE_EQ(plan[0].stopband_edge_hz, 15.36e6 - 1.5e6);
    EXPECT_EQ(plan[1].factor, 4U);
    EXPECT_DOUBLE_EQ(plan[1].input_rate_sps, 15.36e6);
    EXPECT_DOUBLE_EQ(plan[1].output_rate_sps, 3.84e6);
    EXPECT_DOUBLE_EQ(plan[1].stopband_edge_hz, 3.84e6 / 2.0);
}


TEST(RfsocUhdUtilsTest, PlanDecimationRatesChain)
{
    const auto plan = rfsoc_uhd::plan_decimation(122.88e6, 3.84e6, 1.5e6, 8);
    ASSERT_GE(plan.size(), 2U);
    unsigned int product = 1;
    double rate = 122.88e6;
    for (const auto& st : plan)
        {
            EXPECT_LE(st.factor, 8U);
            EXPECT_DOUBLE_EQ(st.input_rate_sps, rate);
            product *= st.factor;
            rate = st.output_rate_sps;
        }
    EXPECT_EQ(product, 32U);
    EXPECT_DOUBLE_EQ(plan.back().output_rate_sps, 3.84e6);
}


TEST(RfsocUhdUtilsTest, PlanDecimationSplitsWhenCheaper)
{
    // L5 at 20.48 MS/s: one /6 stage needs a sharp filter at 122.88 MS/s,
    // /3 then /2 moves the sharp part to the lower rate
    const auto plan = rfsoc_uhd::plan_decimation(122.88e6, 20.48e6, 9.0e6, 8);
    ASSERT_EQ(plan.size(), 2U);
    EXPECT_EQ(plan[0].factor, 3U);
    EXPECT_EQ(plan[1].factor, 2U);
    const auto one_stage = rfsoc_uhd::plan_from_factors(122.88e6, 9.0e6, {6});
    EXPECT_LT(rfsoc_uhd::relative_cost(plan), rfsoc_uhd::relative_cost(one_stage));
}


TEST(RfsocUhdUtilsTest, PlanDecimationIsNoWorseThanGreedySplit)
{
    struct Case
    {
        double device, out, pass;
    };
    const std::vector<Case> cases = {
        {122.88e6, 3.84e6, 1.5e6},
        {122.88e6, 4.096e6, 1.6e6},
        {122.88e6, 20.48e6, 9.0e6},
        {125.0e6, 5.0e6, 2.0e6},
        {245.76e6, 2.048e6, 0.8e6},
        {120.0e6, 4.0e6, 1.6e6}};
    for (const auto& c : cases)
        {
            const unsigned int d = rfsoc_uhd::decimation_factor(c.device, c.out);
            const auto best = rfsoc_uhd::plan_decimation(c.device, c.out, c.pass, 8);
            const auto greedy = rfsoc_uhd::plan_from_factors(c.device, c.pass, rfsoc_uhd::split_decimation(d, 8));
            EXPECT_LE(rfsoc_uhd::relative_cost(best), rfsoc_uhd::relative_cost(greedy) * (1.0 + 1e-12))
                << "device " << c.device << " out " << c.out;
            EXPECT_DOUBLE_EQ(best.back().output_rate_sps, c.out);
        }
}


TEST(RfsocUhdUtilsTest, PlanDecimationRespectsStageLimit)
{
    const auto plan = rfsoc_uhd::plan_decimation(122.88e6, 3.84e6, 1.5e6, 2);
    EXPECT_EQ(plan.size(), 5U);
    for (const auto& st : plan)
        {
            EXPECT_EQ(st.factor, 2U);
        }
}


TEST(RfsocUhdUtilsTest, PlanDecimationProtectsThePassband)
{
    const double out = 3.84e6;
    const double pass = 1.5e6;
    const auto plan = rfsoc_uhd::plan_decimation(122.88e6, out, pass, 8);
    ASSERT_FALSE(plan.empty());
    for (const auto& st : plan)
        {
            EXPECT_DOUBLE_EQ(st.passband_edge_hz, pass);
            EXPECT_GT(st.stopband_edge_hz, st.passband_edge_hz);
            // Nothing that folds back onto the passband survives:
            // images land at k * out_rate, so the stopband must start by
            // out_rate - pass.
            EXPECT_LE(st.stopband_edge_hz, st.output_rate_sps - pass + 1e-6);
            EXPECT_LT(st.stopband_edge_hz, st.input_rate_sps / 2.0);
        }
    // the last stage keeps the whole output band free of aliases
    EXPECT_LE(plan.back().stopband_edge_hz, out / 2.0 + 1e-6);
}


TEST(RfsocUhdUtilsTest, PlanDecimationNoDecimationIsEmpty)
{
    EXPECT_TRUE(rfsoc_uhd::plan_decimation(122.88e6, 122.88e6, 50e6, 8).empty());
}


TEST(RfsocUhdUtilsTest, PlanDecimationRejectsPassbandBeyondOutputNyquist)
{
    EXPECT_THROW(rfsoc_uhd::plan_decimation(122.88e6, 3.84e6, 1.92e6, 8), std::invalid_argument);
    EXPECT_THROW(rfsoc_uhd::plan_decimation(122.88e6, 3.84e6, 0.0, 8), std::invalid_argument);
}


TEST(RfsocUhdUtilsTest, DefaultPassbandIsBelowOutputNyquist)
{
    const double out = 3.84e6;
    const double p = rfsoc_uhd::default_passband_hz(out);
    EXPECT_GT(p, 0.0);
    EXPECT_LT(p, out / 2.0);
}


// ---- first Nyquist zone ------------------------------------------------------

TEST(RfsocUhdUtilsTest, FirstNyquistZoneCoversAllGnssBandsOnRfsoc4x2)
{
    // lowest converter rate of the RFSoC 4x2 builds: 4800 MS/s -> 0 .. 2400 MHz
    const double fs = 4800.0e6;
    EXPECT_TRUE(rfsoc_uhd::in_first_nyquist_zone(1176.45e6, fs));  // L5 / E5a
    EXPECT_TRUE(rfsoc_uhd::in_first_nyquist_zone(1207.14e6, fs));  // E5b / B2I
    EXPECT_TRUE(rfsoc_uhd::in_first_nyquist_zone(1227.60e6, fs));  // L2
    EXPECT_TRUE(rfsoc_uhd::in_first_nyquist_zone(1278.75e6, fs));  // E6
    EXPECT_TRUE(rfsoc_uhd::in_first_nyquist_zone(1561.098e6, fs));  // B1I
    EXPECT_TRUE(rfsoc_uhd::in_first_nyquist_zone(1575.42e6, fs));  // L1 / E1
    EXPECT_TRUE(rfsoc_uhd::in_first_nyquist_zone(1602.0e6, fs));   // GLONASS L1
}


TEST(RfsocUhdUtilsTest, FirstNyquistZoneEdges)
{
    const double fs = 4915.2e6;
    EXPECT_TRUE(rfsoc_uhd::in_first_nyquist_zone(fs / 2.0, fs));
    EXPECT_FALSE(rfsoc_uhd::in_first_nyquist_zone(fs / 2.0 + 1.0, fs));
    EXPECT_FALSE(rfsoc_uhd::in_first_nyquist_zone(0.0, fs));
    EXPECT_FALSE(rfsoc_uhd::in_first_nyquist_zone(-1.0e6, fs));
    // the ZCU216 (fs 2500 MS/s) cannot reach L1 in the first zone
    EXPECT_FALSE(rfsoc_uhd::in_first_nyquist_zone(1575.42e6, 2500.0e6));
}


// ---- UHD device arguments ----------------------------------------------------

TEST(RfsocUhdUtilsTest, DeviceArgsDefaults)
{
    rfsoc_uhd::Device_Args a;
    a.addr = "192.168.1.50";
    a.channels = 1;
    EXPECT_EQ(rfsoc_uhd::build_device_args(a),
        "type=rfsoc,product=rfsoc4x2,addr=192.168.1.50,channels=1");
}


TEST(RfsocUhdUtilsTest, DeviceArgsDiscoveryWithoutAddrAndNarrowWireFormat)
{
    rfsoc_uhd::Device_Args a;
    a.channels = 2;
    a.otw_format = "sc8";
    EXPECT_EQ(rfsoc_uhd::build_device_args(a),
        "type=rfsoc,product=rfsoc4x2,channels=2,otw=sc8");
}


TEST(RfsocUhdUtilsTest, DeviceArgsExtraArgsAppended)
{
    rfsoc_uhd::Device_Args a;
    a.addr = "10.0.0.2";
    a.channels = 1;
    a.extra = "rx_workers=2";
    EXPECT_EQ(rfsoc_uhd::build_device_args(a),
        "type=rfsoc,product=rfsoc4x2,addr=10.0.0.2,channels=1,rx_workers=2");
}


TEST(RfsocUhdUtilsTest, DeviceArgsRejectsBadChannelCountAndWireFormat)
{
    rfsoc_uhd::Device_Args a;
    a.channels = 0;
    EXPECT_THROW(rfsoc_uhd::build_device_args(a), std::invalid_argument);
    a.channels = 3;  // the RFSoC 4x2 has two ADC inputs
    EXPECT_THROW(rfsoc_uhd::build_device_args(a), std::invalid_argument);
    a.channels = 1;
    a.otw_format = "fc64";
    EXPECT_THROW(rfsoc_uhd::build_device_args(a), std::invalid_argument);
}


TEST(RfsocUhdUtilsTest, DeviceArgsRejectsOverridingFixedKeys)
{
    rfsoc_uhd::Device_Args a;
    a.channels = 1;
    a.extra = "type=b200";
    EXPECT_THROW(rfsoc_uhd::build_device_args(a), std::invalid_argument);
    a.extra = "foo=1,channels=2";
    EXPECT_THROW(rfsoc_uhd::build_device_args(a), std::invalid_argument);
}
