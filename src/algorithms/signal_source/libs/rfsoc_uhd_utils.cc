/*!
 * \file rfsoc_uhd_utils.cc
 * \brief Helpers for the RFSoC signal source (uhd-rfsoc).
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
#include <algorithm>
#include <cmath>
#include <functional>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace rfsoc_uhd
{
namespace
{
constexpr double RATIO_REL_TOLERANCE = 1e-9;
constexpr double COST_REL_TOLERANCE = 1e-9;
constexpr double DEFAULT_PASSBAND_FRACTION = 0.4;
constexpr double TAPS_MARGIN = 1.1;
// bounds the search over stage splits (2500 MS/s down to 1 MS/s is 2500)
constexpr unsigned int MAX_TOTAL_DECIMATION = 10000;

bool is_prime(unsigned int n)
{
    if (n < 2)
        {
            return false;
        }
    for (unsigned int p = 2; p <= n / p; p++)
        {
            if (n % p == 0)
                {
                    return false;
                }
        }
    return true;
}

// Every ordered way of writing n as a product of stage factors, each at
// most max_stage, or a prime above it (which cannot be split further).
void ordered_splits_rec(unsigned int n, unsigned int max_stage,
    std::vector<unsigned int>& prefix, std::vector<std::vector<unsigned int>>& out)
{
    if (n == 1)
        {
            out.push_back(prefix);
            return;
        }
    for (unsigned int f = 2; f <= n; f++)
        {
            if (n % f != 0 || (f > max_stage && !is_prime(f)))
                {
                    continue;
                }
            prefix.push_back(f);
            ordered_splits_rec(n / f, max_stage, prefix, out);
            prefix.pop_back();
        }
}

std::vector<std::vector<unsigned int>> ordered_splits(unsigned int n, unsigned int max_stage)
{
    std::vector<std::vector<unsigned int>> out;
    std::vector<unsigned int> prefix;
    ordered_splits_rec(n, max_stage, prefix, out);
    return out;
}

std::vector<unsigned int> prime_factors_descending(unsigned int n)
{
    std::vector<unsigned int> primes;
    for (unsigned int p = 2; p <= n / p; p++)
        {
            while (n % p == 0)
                {
                    primes.push_back(p);
                    n /= p;
                }
        }
    if (n > 1)
        {
            primes.push_back(n);
        }
    std::sort(primes.begin(), primes.end(), std::greater<unsigned int>());
    return primes;
}

std::string trim(const std::string& s)
{
    const auto first = s.find_first_not_of(" \t");
    if (first == std::string::npos)
        {
            return std::string();
        }
    const auto last = s.find_last_not_of(" \t");
    return s.substr(first, last - first + 1);
}

void check_extra_args(const std::string& extra)
{
    static const std::vector<std::string> reserved = {"type", "product", "addr", "channels", "otw"};
    std::istringstream in(extra);
    std::string pair;
    while (std::getline(in, pair, ','))
        {
            const std::string key = trim(pair.substr(0, pair.find('=')));
            if (std::find(reserved.cbegin(), reserved.cend(), key) != reserved.cend())
                {
                    throw std::invalid_argument("RFSoC device_args must not set '" + key +
                                                "': it has its own configuration key");
                }
        }
}
}  // namespace


unsigned int decimation_factor(double device_rate_sps, double output_rate_sps)
{
    if (!(device_rate_sps > 0.0) || !(output_rate_sps > 0.0) ||
        !std::isfinite(device_rate_sps) || !std::isfinite(output_rate_sps))
        {
            throw std::invalid_argument("RFSoC sample rates must be positive");
        }
    if (output_rate_sps > device_rate_sps * (1.0 + RATIO_REL_TOLERANCE))
        {
            throw std::invalid_argument("RFSoC sampling_frequency exceeds device_sampling_frequency: only decimation is supported");
        }
    const double ratio = device_rate_sps / output_rate_sps;
    const double n = std::round(ratio);
    if (n > static_cast<double>(MAX_TOTAL_DECIMATION))
        {
            std::ostringstream msg;
            msg << "RFSoC device_sampling_frequency / sampling_frequency = " << ratio << " exceeds "
                << MAX_TOTAL_DECIMATION;
            throw std::invalid_argument(msg.str());
        }
    if (n < 1.0 || std::abs(ratio - n) > RATIO_REL_TOLERANCE * ratio)
        {
            std::ostringstream msg;
            msg.precision(12);
            msg << "RFSoC device_sampling_frequency / sampling_frequency = " << device_rate_sps << " / "
                << output_rate_sps << " = " << ratio << " is not an integer";
            throw std::invalid_argument(msg.str());
        }
    return static_cast<unsigned int>(n);
}


std::vector<unsigned int> split_decimation(unsigned int total, unsigned int max_stage_factor)
{
    if (total == 0)
        {
            throw std::invalid_argument("RFSoC decimation factor must be at least 1");
        }
    if (max_stage_factor < 2)
        {
            throw std::invalid_argument("RFSoC max_stage_decimation must be at least 2");
        }
    // first-fit of the primes, largest first, into stages of at most the limit
    std::vector<unsigned int> stages;
    for (unsigned int p : prime_factors_descending(total))
        {
            auto fits = std::find_if(stages.begin(), stages.end(),
                [p, max_stage_factor](unsigned int s) { return s * p <= max_stage_factor; });
            if (fits != stages.end())
                {
                    *fits *= p;
                }
            else
                {
                    stages.push_back(p);
                }
        }
    // the first stage runs at the highest rate with the widest transition band
    std::sort(stages.begin(), stages.end(), std::greater<unsigned int>());
    return stages;
}


std::vector<Decimation_Stage> plan_decimation(double device_rate_sps,
    double output_rate_sps,
    double passband_hz,
    unsigned int max_stage_factor,
    double stopband_attenuation_db,
    double passband_ripple_db)
{
    const unsigned int total = decimation_factor(device_rate_sps, output_rate_sps);
    if (!(passband_hz > 0.0) || !(passband_hz < output_rate_sps / 2.0))
        {
            std::ostringstream msg;
            msg << "RFSoC passband_hz=" << passband_hz << " must be inside (0, sampling_frequency / 2 = "
                << output_rate_sps / 2.0 << ")";
            throw std::invalid_argument(msg.str());
        }

    if (max_stage_factor < 2)
        {
            throw std::invalid_argument("RFSoC max_stage_decimation must be at least 2");
        }
    if (total == 1)
        {
            return {};
        }

    std::vector<Decimation_Stage> best;
    double best_cost = 0.0;
    for (const std::vector<unsigned int>& split : ordered_splits(total, max_stage_factor))
        {
            std::vector<Decimation_Stage> plan = plan_from_factors(device_rate_sps, passband_hz, split);
            const double cost = relative_cost(plan, stopband_attenuation_db, passband_ripple_db);
            const bool cheaper = best.empty() || cost < best_cost * (1.0 - COST_REL_TOLERANCE);
            const bool tie_fewer = !best.empty() && !cheaper && cost <= best_cost * (1.0 + COST_REL_TOLERANCE) &&
                                   plan.size() < best.size();
            if (cheaper || tie_fewer)
                {
                    best = std::move(plan);
                    best_cost = cost;
                }
        }
    return best;
}


std::vector<Decimation_Stage> plan_from_factors(double device_rate_sps,
    double passband_hz,
    const std::vector<unsigned int>& factors)
{
    std::vector<Decimation_Stage> plan;
    plan.reserve(factors.size());
    unsigned int cumulative = 1;
    double in_rate = device_rate_sps;
    for (size_t i = 0; i < factors.size(); i++)
        {
            cumulative *= factors[i];
            // from the device rate each time: no rounding drift along the chain
            const double out_rate = device_rate_sps / static_cast<double>(cumulative);
            const bool last = (i + 1 == factors.size());
            // images of the output land at multiples of out_rate: an
            // intermediate stage only has to keep them off the passband, the
            // last one keeps the whole output band clean
            const double stop = last ? out_rate / 2.0 : out_rate - passband_hz;
            plan.push_back(Decimation_Stage{factors[i], in_rate, out_rate, passband_hz, stop});
            in_rate = out_rate;
        }
    return plan;
}


unsigned int estimate_taps(const Decimation_Stage& stage,
    double stopband_attenuation_db,
    double passband_ripple_db)
{
    // Kaiser's estimate for an equiripple low-pass, with a margin: the
    // Parks-McClellan design at exactly the estimate can miss by ~1 dB
    const double r = std::pow(10.0, passband_ripple_db / 20.0);
    const double delta_p = (r - 1.0) / (r + 1.0);  // peak-to-peak ripple -> +-deviation
    const double delta_s = std::pow(10.0, -stopband_attenuation_db / 20.0);
    const double transition = (stage.stopband_edge_hz - stage.passband_edge_hz) / stage.input_rate_sps;
    const double n = (-10.0 * std::log10(delta_p * delta_s) - 13.0) / (14.6 * transition);
    auto taps = static_cast<unsigned int>(std::ceil(TAPS_MARGIN * n)) + 1U;
    if (taps % 2 == 0)
        {
            taps++;
        }
    return std::max(taps, 3U);
}


double relative_cost(const std::vector<Decimation_Stage>& plan,
    double stopband_attenuation_db,
    double passband_ripple_db)
{
    double cost = 0.0;
    for (const Decimation_Stage& st : plan)
        {
            // a polyphase decimator computes its taps once per output sample
            cost += st.output_rate_sps * static_cast<double>(estimate_taps(st, stopband_attenuation_db, passband_ripple_db));
        }
    return cost;
}


double default_passband_hz(double output_rate_sps)
{
    if (!(output_rate_sps > 0.0))
        {
            throw std::invalid_argument("RFSoC sampling_frequency must be positive");
        }
    return DEFAULT_PASSBAND_FRACTION * output_rate_sps;
}


bool in_first_nyquist_zone(double freq_hz, double converter_rate_sps)
{
    return converter_rate_sps > 0.0 && freq_hz > 0.0 && freq_hz <= converter_rate_sps / 2.0;
}


std::string build_device_args(const Device_Args& args)
{
    if (args.channels < 1 || args.channels > RFSOC4X2_MAX_RF_CHANNELS)
        {
            throw std::invalid_argument("RFSoC 4x2 RF_channels must be 1 or 2, got " + std::to_string(args.channels));
        }
    if (args.otw_format != "sc16" && args.otw_format != "sc12" && args.otw_format != "sc8")
        {
            throw std::invalid_argument("RFSoC otw_format must be sc16, sc12 or sc8, got " + args.otw_format);
        }
    check_extra_args(args.extra);

    std::string s = "type=rfsoc";
    if (!args.product.empty())
        {
            s += ",product=" + args.product;
        }
    if (!args.addr.empty())
        {
            s += ",addr=" + args.addr;
        }
    s += ",channels=" + std::to_string(args.channels);
    if (args.otw_format != "sc16")
        {
            // uhd-rfsoc offers the rates only a narrower format fits when told
            s += ",otw=" + args.otw_format;
        }
    if (!args.extra.empty())
        {
            s += "," + args.extra;
        }
    return s;
}
}  // namespace rfsoc_uhd
