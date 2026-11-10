/*!
 * \file rfsoc_uhd_signal_source.cc
 * \brief Signal source for AMD RFSoC boards (Real Digital RFSoC 4x2) served
 *  as UHD devices over 100 GbE by uhd-rfsoc (UHD device type=rfsoc).
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

#include "rfsoc_uhd_signal_source.h"
#include "GPS_L1_CA.h"
#include "configuration_interface.h"
#include "gnss_sdr_create_directory.h"
#include "gnss_sdr_filesystem.h"
#include "gnss_sdr_string_literals.h"
#include "gnss_sdr_valve.h"
#include "rfsoc_uhd_filter_design.h"
#include <uhd/types/device_addr.hpp>
#include <uhd/types/ranges.hpp>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <sstream>
#include <stdexcept>

#ifdef GR_GREATER_38
#include <gnuradio/filter/fir_filter_blk.h>
#include <gnuradio/filter/freq_xlating_fir_filter.h>
#else
#include <gnuradio/filter/fir_filter_ccf.h>
#include <gnuradio/filter/freq_xlating_fir_filter_ccf.h>
#endif

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

using namespace std::string_literals;

namespace
{
const char* const IMPLEMENTATION = "RFSoC_UHD_Signal_Source";

// a rate the board reports back may differ from the request by rounding only
constexpr double RATE_REL_TOLERANCE = 1e-6;

// uhd-rfsoc RFDC step attenuator (Gen3 DSA): -27 .. 0 dB
constexpr double DEFAULT_GAIN_DB = 0.0;

// The RFDC fine mixer has a 48-bit NCO (about 1e-5 Hz steps at fs 4915.2
// MS/s); a residual below this is left to the tracking loops rather than
// paying for a complex-tap first stage.
constexpr double MAX_IGNORED_TUNE_OFFSET_HZ = 1.0;

std::string channel_key(const std::string& role, const std::string& key, int channel, int n_channels)
{
    return n_channels == 1 ? role + "." + key : role + "." + key + std::to_string(channel);
}

void create_dump_directory(const std::string& filename)
{
    const std::string::size_type pos = filename.find_last_of(fs::path::preferred_separator);
    if (pos != std::string::npos && !gnss_sdr_create_directory(filename.substr(0, pos)))
        {
            std::cerr << "GNSS-SDR cannot create the " << filename.substr(0, pos) << " folder. Wrong permissions?\n";
        }
}
}  // namespace


RfsocUhdSignalSource::RfsocUhdSignalSource(const ConfigurationInterface* configuration,
    const std::string& role, unsigned int in_stream, unsigned int out_stream,
    Concurrent_Queue<pmt::pmt_t>* queue)
    : SignalSourceBase(configuration, role, IMPLEMENTATION),
      item_size_(sizeof(gr_complex)),
      in_stream_(in_stream),
      out_stream_(out_stream)
{
    const std::string empty;

    // ---- configuration ---------------------------------------------------
    rfsoc_uhd::Device_Args dev;
    dev.addr = configuration->property(role + ".device_address", empty);
    dev.product = configuration->property(role + ".product", std::string("rfsoc4x2"));
    dev.extra = configuration->property(role + ".device_args", empty);
    otw_format_ = configuration->property(role + ".otw_format", std::string("sc16"));
    dev.otw_format = otw_format_;
    clock_source_ = configuration->property(role + ".clock_source", std::string("internal"));
    RF_channels_ = configuration->property(role + ".RF_channels", 1);
    device_rate_sps_ = configuration->property(role + ".device_sampling_frequency", rfsoc_uhd::RFSOC4X2_DEFAULT_DEVICE_RATE_SPS);
    sample_rate_sps_ = configuration->property(role + ".sampling_frequency", 3.84e6);
    passband_hz_ = configuration->property(role + ".passband_hz", rfsoc_uhd::default_passband_hz(sample_rate_sps_));
    const unsigned int max_stage = configuration->property(role + ".max_stage_decimation", rfsoc_uhd::DEFAULT_MAX_STAGE_DECIMATION);
    stopband_attenuation_db_ = configuration->property(role + ".stopband_attenuation_db", rfsoc_uhd::DEFAULT_STOPBAND_ATTENUATION_DB);
    const std::string item_type = configuration->property(role + ".item_type", std::string("gr_complex"));

    if (item_type != "gr_complex")
        {
            throw std::invalid_argument(std::string(IMPLEMENTATION) + ": item_type=" + item_type +
                                        " is not supported, the decimated output is gr_complex");
        }
    if (RF_channels_ < 1 || RF_channels_ > static_cast<int>(rfsoc_uhd::RFSOC4X2_MAX_RF_CHANNELS))
        {
            throw std::invalid_argument(std::string(IMPLEMENTATION) + ": RF_channels=" + std::to_string(RF_channels_) +
                                        ", the RFSoC 4x2 has 1 or 2 RX channels");
        }
    dev.channels = static_cast<unsigned int>(RF_channels_);

    // throws std::invalid_argument on a non-integer ratio or a bad passband
    plan_ = rfsoc_uhd::plan_decimation(device_rate_sps_, sample_rate_sps_, passband_hz_, max_stage,
        stopband_attenuation_db_, rfsoc_uhd::DEFAULT_PASSBAND_RIPPLE_DB);

    for (int ch = 0; ch < RF_channels_; ch++)
        {
            read_channel_config(configuration, role, ch);
        }

    if (plan_.empty() && RF_channels_ > 1)
        {
            // Without decimators the UHD block is the shared multi-port output:
            // the flowgraph can only address it consistently if every channel
            // ends on the same kind of block.
            const bool any_valve = std::any_of(samples_.cbegin(), samples_.cend(), [](uint64_t s) { return s != 0; });
            const bool all_valve = std::all_of(samples_.cbegin(), samples_.cend(), [](uint64_t s) { return s != 0; });
            if (any_valve && !all_valve)
                {
                    throw std::invalid_argument(std::string(IMPLEMENTATION) +
                                                ": without decimation, set samplesN on every RF channel or on none");
                }
        }

    // ---- device ------------------------------------------------------------
    open_device(rfsoc_uhd::build_device_args(dev));

    decimators_.resize(RF_channels_);
    for (int ch = 0; ch < RF_channels_; ch++)
        {
            tune_channel(ch);
        }

    // ---- sample limit and dump (at the decimated rate) ------------------
    valve_.resize(RF_channels_);
    file_sink_.resize(RF_channels_);
    for (int ch = 0; ch < RF_channels_; ch++)
        {
            if (samples_.at(ch) != 0ULL)
                {
                    LOG(INFO) << "RF_channel " << ch << ": send STOP signal after " << samples_.at(ch) << " samples";
                    valve_.at(ch) = gnss_sdr_make_valve(item_size_, samples_.at(ch), queue);
                }
            if (dump_.at(ch))
                {
                    LOG(INFO) << "RF_channel " << ch << ": dumping the decimated output into " << dump_filename_.at(ch);
                    file_sink_.at(ch) = gr::blocks::file_sink::make(item_size_, dump_filename_.at(ch).c_str());
                }
        }

    if (in_stream_ > 0)
        {
            LOG(ERROR) << "A signal source does not have an input stream";
        }
    if (out_stream_ > 1)
        {
            LOG(ERROR) << "This implementation only supports one output stream";
        }
}


void RfsocUhdSignalSource::read_channel_config(const ConfigurationInterface* configuration,
    const std::string& role, int channel)
{
    const int n = RF_channels_;
    const std::string default_dump = n == 1 ? "./data/signal_source.dat"s : "./data/signal_source"s + std::to_string(channel) + ".dat"s;

    samples_.push_back(configuration->property(channel_key(role, "samples", channel, n), static_cast<uint64_t>(0)));
    const bool dump = configuration->property(channel_key(role, "dump", channel, n), false);
    dump_.push_back(dump);
    const std::string dump_filename = configuration->property(channel_key(role, "dump_filename", channel, n), default_dump);
    dump_filename_.push_back(dump_filename);
    if (dump)
        {
            create_dump_directory(dump_filename);
        }

    const double freq = configuration->property(channel_key(role, "freq", channel, n), GPS_L1_FREQ_HZ);
    // Checked against the device's own range once it is open; this catches a
    // frequency no RFSoC 4x2 build reaches before the (slow) device setup.
    if (!rfsoc_uhd::in_first_nyquist_zone(freq, rfsoc_uhd::RFSOC4X2_MAX_CONVERTER_RATE_SPS))
        {
            throw std::invalid_argument(std::string(IMPLEMENTATION) + ": RF channel " + std::to_string(channel) +
                                        " freq=" + std::to_string(freq) + " Hz is outside the first Nyquist zone (0 .. " +
                                        std::to_string(rfsoc_uhd::RFSOC4X2_MAX_CONVERTER_RATE_SPS / 2.0) + " Hz)");
        }
    freq_.push_back(freq);
    gain_.push_back(configuration->property(channel_key(role, "gain", channel, n), DEFAULT_GAIN_DB));
}


void RfsocUhdSignalSource::open_device(const std::string& device_args)
{
    std::cout << IMPLEMENTATION << ": opening UHD device \"" << device_args << "\"\n";
    LOG(INFO) << IMPLEMENTATION << ": opening UHD device \"" << device_args << "\"";

    uhd::stream_args_t stream_args("fc32", otw_format_);
    for (int ch = 0; ch < RF_channels_; ch++)
        {
            stream_args.channels.push_back(ch);
        }

    try
        {
            uhd_source_ = gr::uhd::usrp_source::make(uhd::device_addr_t(device_args), stream_args);
        }
    catch (const std::exception& e)
        {
            throw std::runtime_error(std::string(IMPLEMENTATION) + ": cannot open \"" + device_args + "\": " + e.what() +
                                     " (is the uhd-rfsoc daemon running on the board, the uhd_rfsoc module installed,"
                                     " and gnss-sdr running as root for DPDK?)");
        }

    if (!clock_source_.empty())
        {
            uhd_source_->set_clock_source(clock_source_);
        }

    // A rate outside the loaded bitstream reloads the board (10 .. 20 s).
    std::cout << IMPLEMENTATION << ": requesting " << device_rate_sps_ << " sps from the board...\n";
    uhd_source_->set_samp_rate(device_rate_sps_);
    const double actual = uhd_source_->get_samp_rate();
    std::cout << IMPLEMENTATION << ": device rate " << actual << " sps, decimated to " << sample_rate_sps_ << " sps\n";
    LOG(INFO) << IMPLEMENTATION << ": device rate " << actual << " sps";
    if (std::abs(actual - device_rate_sps_) > RATE_REL_TOLERANCE * device_rate_sps_)
        {
            // uhd-rfsoc meets a request from below: the decimation chain would
            // run at the wrong rate and every Doppler and code rate would be off
            std::ostringstream msg;
            msg << IMPLEMENTATION << ": the board runs at " << actual << " sps instead of the requested "
                << device_rate_sps_ << " sps (device_sampling_frequency). Offered rates: "
                << uhd_source_->get_samp_rates().to_pp_string();
            throw std::runtime_error(msg.str());
        }
}


void RfsocUhdSignalSource::tune_channel(int channel)
{
    const auto ch = static_cast<size_t>(channel);
    const double freq = freq_.at(ch);

    // uhd-rfsoc clamps a frequency outside 0 .. fs/2 of the loaded build
    // without an error, so check against the front end's own range
    // (get_freq_range() is the overall tune range, widened by the bandwidth)
    const uhd::freq_range_t range = uhd_source_->get_device()->get_fe_rx_freq_range(ch);
    if (freq < range.start() || freq > range.stop())
        {
            std::ostringstream msg;
            msg.precision(12);
            msg << IMPLEMENTATION << ": RF channel " << channel << " freq=" << freq
                << " Hz is outside the board's first Nyquist zone " << range.to_pp_string();
            throw std::invalid_argument(msg.str());
        }

    uhd_source_->set_center_freq(freq, ch);
    const double actual = uhd_source_->get_center_freq(ch);
    const double offset = freq - actual;  // residual tune error, removed by the first stage
    std::cout << IMPLEMENTATION << ": RF channel " << channel << " tuned to " << actual << " Hz (residual " << offset << " Hz)\n";
    LOG(INFO) << IMPLEMENTATION << ": RF channel " << channel << " tuned to " << actual << " Hz (residual " << offset << " Hz)";
    if (std::abs(offset) > passband_hz_)
        {
            std::ostringstream msg;
            msg.precision(12);
            msg << IMPLEMENTATION << ": RF channel " << channel << " tuned to " << actual
                << " Hz instead of " << freq << " Hz";
            throw std::runtime_error(msg.str());
        }

    const uhd::gain_range_t gain_range = uhd_source_->get_gain_range(ch);
    double gain = gain_.at(ch);
    if (gain < gain_range.start() || gain > gain_range.stop())
        {
            LOG(WARNING) << IMPLEMENTATION << ": RF channel " << channel << " gain " << gain
                         << " dB outside " << gain_range.to_pp_string() << ", clamped";
            std::cout << IMPLEMENTATION << ": RF channel " << channel << " gain " << gain
                      << " dB is outside " << gain_range.to_pp_string() << " (RFDC attenuation), clamped\n";
            gain = std::min(std::max(gain, gain_range.start()), gain_range.stop());
        }
    uhd_source_->set_gain(gain, ch);
    std::cout << IMPLEMENTATION << ": RF channel " << channel << " gain " << uhd_source_->get_gain(ch) << " dB\n";

    build_decimators(channel, offset);
}


void RfsocUhdSignalSource::build_decimators(int channel, double tune_offset_hz)
{
    auto& chain = decimators_.at(static_cast<size_t>(channel));
    if (plan_.empty())
        {
            if (std::abs(tune_offset_hz) > MAX_IGNORED_TUNE_OFFSET_HZ)
                {
                    LOG(WARNING) << IMPLEMENTATION << ": no decimation, residual tune error of "
                                 << tune_offset_hz << " Hz left in the signal";
                }
            return;
        }

    double macs = 0.0;
    for (size_t s = 0; s < plan_.size(); s++)
        {
            const rfsoc_uhd::Decimation_Stage& st = plan_[s];
            // equiripple: at least stopband_attenuation_db on all that folds onto the passband
            const std::vector<float> taps = rfsoc_uhd::design_stage_taps(st, stopband_attenuation_db_,
                rfsoc_uhd::DEFAULT_PASSBAND_RIPPLE_DB);
            macs += st.output_rate_sps * static_cast<double>(taps.size());
            LOG(INFO) << IMPLEMENTATION << ": RF channel " << channel << " stage " << s << ": /" << st.factor
                      << " " << st.input_rate_sps << " -> " << st.output_rate_sps << " sps, "
                      << taps.size() << " taps, pass " << st.passband_edge_hz << " Hz, stop " << st.stopband_edge_hz << " Hz";
            if (s == 0 && std::abs(tune_offset_hz) > MAX_IGNORED_TUNE_OFFSET_HZ)
                {
                    // also translates the residual tune error to 0 Hz
                    // (complex taps: about four times a real-tap stage)
                    chain.push_back(gr::filter::freq_xlating_fir_filter_ccf::make(
                        st.factor, taps, tune_offset_hz, st.input_rate_sps));
                }
            else
                {
                    chain.push_back(gr::filter::fir_filter_ccf::make(st.factor, taps));
                }
        }
    std::cout << IMPLEMENTATION << ": RF channel " << channel << " decimation in " << plan_.size()
              << " stage(s), " << stopband_attenuation_db_ << " dB alias rejection, about " << macs / 1e9
              << " G multiply-accumulates per second\n";
}


gr::basic_block_sptr RfsocUhdSignalSource::decimated_output(int channel) const
{
    const auto& chain = decimators_.at(static_cast<size_t>(channel));
    if (chain.empty())
        {
            return uhd_source_;
        }
    return chain.back();
}


void RfsocUhdSignalSource::connect(gr::top_block_sptr top_block)
{
    for (int ch = 0; ch < RF_channels_; ch++)
        {
            const auto& chain = decimators_.at(ch);
            // the UHD port of this channel: chain.empty() keeps it on uhd_source_
            const int out_port = chain.empty() ? ch : 0;
            if (!chain.empty())
                {
                    top_block->connect(uhd_source_, ch, chain.front(), 0);
                    for (size_t s = 1; s < chain.size(); s++)
                        {
                            top_block->connect(chain[s - 1], 0, chain[s], 0);
                        }
                    DLOG(INFO) << "connected RFSoC RF channel " << ch << " to its decimators";
                }
            gr::basic_block_sptr tail = decimated_output(ch);
            int tail_port = out_port;
            if (valve_.at(ch))
                {
                    top_block->connect(tail, tail_port, valve_.at(ch), 0);
                    tail = valve_.at(ch);
                    tail_port = 0;
                }
            if (file_sink_.at(ch))
                {
                    top_block->connect(tail, tail_port, file_sink_.at(ch), 0);
                }
        }
}


void RfsocUhdSignalSource::disconnect(gr::top_block_sptr top_block)
{
    uhd_source_->stop();
    for (int ch = 0; ch < RF_channels_; ch++)
        {
            const auto& chain = decimators_.at(ch);
            const int out_port = chain.empty() ? ch : 0;
            if (!chain.empty())
                {
                    top_block->disconnect(uhd_source_, ch, chain.front(), 0);
                    for (size_t s = 1; s < chain.size(); s++)
                        {
                            top_block->disconnect(chain[s - 1], 0, chain[s], 0);
                        }
                }
            gr::basic_block_sptr tail = decimated_output(ch);
            int tail_port = out_port;
            if (valve_.at(ch))
                {
                    top_block->disconnect(tail, tail_port, valve_.at(ch), 0);
                    tail = valve_.at(ch);
                    tail_port = 0;
                }
            if (file_sink_.at(ch))
                {
                    top_block->disconnect(tail, tail_port, file_sink_.at(ch), 0);
                }
        }
    LOG(INFO) << IMPLEMENTATION << " disconnected";
}


gr::basic_block_sptr RfsocUhdSignalSource::get_left_block()
{
    LOG(WARNING) << "Trying to get signal source left block.";
    return gr::uhd::usrp_source::sptr();
}


gr::basic_block_sptr RfsocUhdSignalSource::get_right_block()
{
    return get_right_block(0);
}


gr::basic_block_sptr RfsocUhdSignalSource::get_right_block(int RF_channel)
{
    if (RF_channel < 0 || RF_channel >= RF_channels_)
        {
            return nullptr;
        }
    if (valve_.at(RF_channel))
        {
            return valve_.at(RF_channel);
        }
    return decimated_output(RF_channel);
}
