/*!
 * \file rfsoc_uhd_signal_source_test.cc
 * \brief Configuration checks of RFSoC_UHD_Signal_Source. Every case here
 *  must be rejected before any UHD device is opened, so no hardware is used.
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

#include "concurrent_queue.h"
#include "in_memory_configuration.h"
#include "rfsoc_uhd_signal_source.h"
#include <gtest/gtest.h>
#include <memory>
#include <stdexcept>

namespace
{
// A configuration the source accepts. InMemoryConfiguration::set_property()
// never overwrites, so the cases below change a key with supersede_property().
std::shared_ptr<InMemoryConfiguration> valid_rfsoc_config()
{
    auto config = std::make_shared<InMemoryConfiguration>();
    // an address nothing answers on: the cases below must fail before discovery
    config->set_property("Test.device_address", "192.0.2.1");
    config->set_property("Test.item_type", "gr_complex");
    config->set_property("Test.RF_channels", "1");
    config->set_property("Test.device_sampling_frequency", "122880000");
    config->set_property("Test.sampling_frequency", "3840000");
    config->set_property("Test.freq", "1575420000");
    config->set_property("Test.gain", "0");
    return config;
}

void expect_rejected(const std::shared_ptr<InMemoryConfiguration>& config)
{
    auto queue = std::make_shared<Concurrent_Queue<pmt::pmt_t>>();
    EXPECT_THROW(RfsocUhdSignalSource(config.get(), "Test", 0, 1, queue.get()), std::invalid_argument);
}
}  // namespace


TEST(RfsocUhdSignalSourceTest, RejectsFrequencyOutsideFirstNyquistZone)
{
    auto config = valid_rfsoc_config();
    config->supersede_property("Test.freq", "3000000000");
    expect_rejected(config);
}


TEST(RfsocUhdSignalSourceTest, RejectsNonIntegerDecimation)
{
    auto config = valid_rfsoc_config();
    config->supersede_property("Test.sampling_frequency", "4000000");  // 122.88 / 4 = 30.72
    expect_rejected(config);
}


TEST(RfsocUhdSignalSourceTest, RejectsPassbandBeyondOutputNyquist)
{
    auto config = valid_rfsoc_config();
    config->supersede_property("Test.passband_hz", "2000000");  // > 3.84 MS/s / 2
    expect_rejected(config);
}


TEST(RfsocUhdSignalSourceTest, RejectsNonComplexFloatOutput)
{
    auto config = valid_rfsoc_config();
    config->supersede_property("Test.item_type", "cshort");
    expect_rejected(config);
}


TEST(RfsocUhdSignalSourceTest, RejectsMoreThanTwoRfChannels)
{
    auto config = valid_rfsoc_config();
    config->supersede_property("Test.RF_channels", "3");
    expect_rejected(config);
}


TEST(RfsocUhdSignalSourceTest, RejectsSecondChannelOutsideFirstNyquistZone)
{
    auto config = valid_rfsoc_config();
    config->supersede_property("Test.RF_channels", "2");
    config->supersede_property("Test.freq0", "1575420000");
    config->supersede_property("Test.freq1", "2600000000");
    expect_rejected(config);
}


TEST(RfsocUhdSignalSourceTest, RejectsMixedSampleLimitsWithoutDecimation)
{
    auto config = valid_rfsoc_config();
    config->supersede_property("Test.RF_channels", "2");
    config->supersede_property("Test.sampling_frequency", "122880000");
    config->supersede_property("Test.passband_hz", "50000000");
    config->supersede_property("Test.freq0", "1575420000");
    config->supersede_property("Test.freq1", "1176450000");
    config->supersede_property("Test.samples0", "1000");
    config->supersede_property("Test.samples1", "0");
    expect_rejected(config);
}


TEST(RfsocUhdSignalSourceTest, RejectsDeviceArgsOverridingOwnKeys)
{
    auto config = valid_rfsoc_config();
    config->supersede_property("Test.device_args", "channels=2");
    expect_rejected(config);
}


TEST(RfsocUhdSignalSourceTest, RejectsUnknownWireFormat)
{
    auto config = valid_rfsoc_config();
    config->supersede_property("Test.otw_format", "sc4");
    expect_rejected(config);
}
