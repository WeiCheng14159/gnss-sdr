/*!
 * \file rfsoc_uhd_utils.h
 * \brief Helpers for the RFSoC signal source (AMD RFSoC boards served as UHD
 *  devices by uhd-rfsoc): decimation planning from the board's lowest sample
 *  rate down to a GNSS rate, first-Nyquist-zone checks and UHD device
 *  arguments. Pure C++, without UHD or GNU Radio, so it can be unit tested
 *  (the filter taps themselves are in rfsoc_uhd_filter_design.h).
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

#ifndef GNSS_SDR_RFSOC_UHD_UTILS_H
#define GNSS_SDR_RFSOC_UHD_UTILS_H

#include <string>
#include <vector>

/** \addtogroup Signal_Source
 * \{ */
/** \addtogroup Signal_Source_libs
 * \{ */

namespace rfsoc_uhd
{
/*!
 * \brief Real Digital RFSoC 4x2 limits as served by uhd-rfsoc.
 *
 * The ADCs run at fs = 4800, 4915.2 or 5000 MS/s and uhd-rfsoc tunes only
 * within the first Nyquist zone, 0 .. fs/2, so 0 .. 2400 MHz is reachable
 * with every build: all the GNSS bands (L5/E5, L2, E6, L1/E1/B1, G1).
 * The per-channel rate is fs / F, F in {2, 3, 4, 5, 6, 8, 10, 12, 16, 20,
 * 24, 40}: at least 120 MS/s, far above a GNSS rate, hence the decimation.
 */
constexpr double RFSOC4X2_MIN_CONVERTER_RATE_SPS = 4800.0e6;
constexpr double RFSOC4X2_MAX_CONVERTER_RATE_SPS = 5000.0e6;
constexpr double RFSOC4X2_DEFAULT_DEVICE_RATE_SPS = 122.88e6;  // fs 4915.2 MS/s, F = 40
constexpr unsigned int RFSOC4X2_MAX_RF_CHANNELS = 2;
constexpr unsigned int DEFAULT_MAX_STAGE_DECIMATION = 8;

// decimation filter specification: what folds onto the passband is at
// least this far down, and the passband is flat to this peak-to-peak ripple
constexpr double DEFAULT_STOPBAND_ATTENUATION_DB = 60.0;
constexpr double DEFAULT_PASSBAND_RIPPLE_DB = 0.1;

/*!
 * \brief One FIR decimation stage: low-pass with a passband edge and a
 * stopband edge (one-sided, Hz), then keep one sample out of \p factor.
 */
struct Decimation_Stage
{
    unsigned int factor;
    double input_rate_sps;
    double output_rate_sps;
    double passband_edge_hz;
    double stopband_edge_hz;
};

/*!
 * \brief Arguments for the uhd-rfsoc UHD device (type=rfsoc).
 */
struct Device_Args
{
    std::string addr;                 //!< board LAN (management) address; empty: discovery
    std::string product{"rfsoc4x2"};  //!< filters discovery by board; empty: any
    unsigned int channels{1};         //!< RF channels, 1 .. RFSOC4X2_MAX_RF_CHANNELS
    std::string otw_format{"sc16"};   //!< sc16, sc12 or sc8
    std::string extra;                //!< appended verbatim, e.g. "rx_workers=2"
};

/*!
 * \brief Integer ratio device_rate / output_rate. Throws std::invalid_argument
 * when a rate is not positive, the output rate exceeds the device rate, or the
 * ratio is not an integer (relative tolerance 1e-9).
 */
unsigned int decimation_factor(double device_rate_sps, double output_rate_sps);

/*!
 * \brief Splits \p total into stage factors of at most \p max_stage_factor
 * each (a prime larger than the limit is kept as one stage), largest first.
 * Returns an empty vector for \p total = 1. Throws std::invalid_argument for
 * \p total = 0 or \p max_stage_factor < 2.
 */
std::vector<unsigned int> split_decimation(unsigned int total, unsigned int max_stage_factor);

/*!
 * \brief The stages for a given split of the decimation, keeping 0 ..
 * \p passband_hz (one-sided) free of aliases. Intermediate stages only
 * protect the passband (stopband edge = stage output rate - passband); the
 * last stage also clears the band up to its output rate / 2. No validation
 * beyond what plan_decimation() does first.
 */
std::vector<Decimation_Stage> plan_from_factors(double device_rate_sps,
    double passband_hz,
    const std::vector<unsigned int>& factors);

/*!
 * \brief Taps an equiripple (Parks-McClellan) low-pass needs for \p stage:
 * Kaiser's estimate for the given stopband attenuation and peak-to-peak
 * passband ripple, rounded up to an odd count (at least 3).
 */
unsigned int estimate_taps(const Decimation_Stage& stage,
    double stopband_attenuation_db = DEFAULT_STOPBAND_ATTENUATION_DB,
    double passband_ripple_db = DEFAULT_PASSBAND_RIPPLE_DB);

/*!
 * \brief Computational cost of a plan in multiply-accumulates per second:
 * sum over the stages of output rate x estimate_taps().
 */
double relative_cost(const std::vector<Decimation_Stage>& plan,
    double stopband_attenuation_db = DEFAULT_STOPBAND_ATTENUATION_DB,
    double passband_ripple_db = DEFAULT_PASSBAND_RIPPLE_DB);

/*!
 * \brief Plans the decimation from \p device_rate_sps to \p output_rate_sps:
 * among the ordered splits of the factor into stages of at most
 * \p max_stage_factor (a larger prime stays one stage), the one with the
 * lowest relative_cost(), fewer stages on a tie. See plan_from_factors()
 * for the filter edges. Throws std::invalid_argument when the passband is
 * not inside (0, output_rate / 2) or the rates are inconsistent (see
 * decimation_factor).
 */
std::vector<Decimation_Stage> plan_decimation(double device_rate_sps,
    double output_rate_sps,
    double passband_hz,
    unsigned int max_stage_factor = DEFAULT_MAX_STAGE_DECIMATION,
    double stopband_attenuation_db = DEFAULT_STOPBAND_ATTENUATION_DB,
    double passband_ripple_db = DEFAULT_PASSBAND_RIPPLE_DB);

/*!
 * \brief Default one-sided passband for an output rate: 0.4 x the rate.
 */
double default_passband_hz(double output_rate_sps);

/*!
 * \brief True when 0 < \p freq_hz <= \p converter_rate_sps / 2.
 */
bool in_first_nyquist_zone(double freq_hz, double converter_rate_sps);

/*!
 * \brief The UHD device address string for uhd-rfsoc, e.g.
 * "type=rfsoc,product=rfsoc4x2,addr=192.168.1.50,channels=1". Throws
 * std::invalid_argument on a bad channel count or wire format, or when
 * \p args.extra sets a key this function owns (type, product, addr,
 * channels, otw).
 */
std::string build_device_args(const Device_Args& args);

}  // namespace rfsoc_uhd


/** \} */
/** \} */
#endif  // GNSS_SDR_RFSOC_UHD_UTILS_H
