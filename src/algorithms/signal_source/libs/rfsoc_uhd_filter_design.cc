/*!
 * \file rfsoc_uhd_filter_design.cc
 * \brief Decimation filter taps for the RFSoC signal source (uhd-rfsoc).
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
#include <gnuradio/filter/pm_remez.h>
#include <cmath>
#include <stdexcept>
#include <string>

namespace rfsoc_uhd
{
std::vector<float> design_stage_taps(const Decimation_Stage& stage,
    double stopband_attenuation_db, double passband_ripple_db)
{
    const double nyquist = stage.input_rate_sps / 2.0;
    if (!(stage.passband_edge_hz > 0.0) || !(stage.stopband_edge_hz > stage.passband_edge_hz) ||
        !(stage.stopband_edge_hz < nyquist))
        {
            throw std::invalid_argument("RFSoC decimation stage edges must satisfy 0 < pass < stop < input_rate / 2");
        }

    const unsigned int ntaps = estimate_taps(stage, stopband_attenuation_db, passband_ripple_db);

    // weight the stopband by delta_p / delta_s so both bands hit their spec
    const double r = std::pow(10.0, passband_ripple_db / 20.0);
    const double delta_p = (r - 1.0) / (r + 1.0);
    const double delta_s = std::pow(10.0, -stopband_attenuation_db / 20.0);

    const std::vector<double> bands = {0.0, stage.passband_edge_hz / nyquist, stage.stopband_edge_hz / nyquist, 1.0};
    const std::vector<double> ampl = {1.0, 1.0, 0.0, 0.0};
    const std::vector<double> weight = {1.0, delta_p / delta_s};

    std::vector<double> taps_d;
    try
        {
            taps_d = gr::filter::pm_remez(static_cast<int>(ntaps) - 1, bands, ampl, weight, "bandpass", 16);
        }
    catch (const std::exception& e)
        {
            throw std::runtime_error("RFSoC decimation filter design (" + std::to_string(ntaps) +
                                     " taps) failed: " + e.what());
        }
    return std::vector<float>(taps_d.cbegin(), taps_d.cend());
}
}  // namespace rfsoc_uhd
