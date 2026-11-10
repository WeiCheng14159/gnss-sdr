/*!
 * \file rfsoc_uhd_filter_design.h
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

#ifndef GNSS_SDR_RFSOC_UHD_FILTER_DESIGN_H
#define GNSS_SDR_RFSOC_UHD_FILTER_DESIGN_H

#include "rfsoc_uhd_utils.h"
#include <vector>

/** \addtogroup Signal_Source
 * \{ */
/** \addtogroup Signal_Source_libs
 * \{ */

namespace rfsoc_uhd
{
/*!
 * \brief Equiripple (Parks-McClellan) low-pass taps for one decimation
 * stage: unit gain over 0 .. passband_edge_hz within \p passband_ripple_db
 * peak-to-peak, at least \p stopband_attenuation_db down from
 * stopband_edge_hz to input_rate / 2, estimate_taps() long. Throws
 * std::runtime_error if the design does not converge.
 */
std::vector<float> design_stage_taps(const Decimation_Stage& stage,
    double stopband_attenuation_db = DEFAULT_STOPBAND_ATTENUATION_DB,
    double passband_ripple_db = DEFAULT_PASSBAND_RIPPLE_DB);
}  // namespace rfsoc_uhd


/** \} */
/** \} */
#endif  // GNSS_SDR_RFSOC_UHD_FILTER_DESIGN_H
