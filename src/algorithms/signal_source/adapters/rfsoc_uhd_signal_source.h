/*!
 * \file rfsoc_uhd_signal_source.h
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

#ifndef GNSS_SDR_RFSOC_UHD_SIGNAL_SOURCE_H
#define GNSS_SDR_RFSOC_UHD_SIGNAL_SOURCE_H

#include "concurrent_queue.h"
#include "rfsoc_uhd_utils.h"
#include "signal_source_base.h"
#include <gnuradio/blocks/file_sink.h>
#include <gnuradio/uhd/usrp_source.h>
#include <pmt/pmt.h>
#include <cstdint>
#include <string>
#include <vector>

/** \addtogroup Signal_Source
 * \{ */
/** \addtogroup Signal_Source_adapters
 * \{ */

class ConfigurationInterface;

/*!
 * \brief Reads samples from an RFSoC board through uhd-rfsoc.
 *
 * The board's lowest per-channel rate (120 .. 125 MS/s on the RFSoC 4x2) is
 * far above what the GNSS processing needs, so each RF channel goes through
 * a multi-stage FIR decimator (the first stage also removes the residual
 * tuning error) down to \c sampling_frequency. Tuning is limited to the
 * first Nyquist zone (0 .. fs/2, at least 0 .. 2400 MHz on the RFSoC 4x2),
 * which holds every GNSS band.
 *
 * Main configuration keys (role = SignalSource):
 *  - device_address: the board's LAN address (empty: UHD discovery)
 *  - device_sampling_frequency: rate requested from the board [122.88e6]
 *  - sampling_frequency: rate delivered to the receiver [3.84e6]; the ratio
 *    to device_sampling_frequency must be an integer
 *  - RF_channels: 1 or 2
 *  - freq / freqN, gain / gainN (RFDC attenuation, -27 .. 0 dB) [0]
 *  - passband_hz: one-sided passband kept alias-free [0.4 x sampling_frequency]
 *  - stopband_attenuation_db: rejection of what folds onto the passband [60]
 *  - max_stage_decimation: largest factor of one FIR stage [8]
 *  - otw_format: sc16, sc12 or sc8; device_args: extra uhd-rfsoc device args
 *  - samples / samplesN, dump / dumpN, dump_filename / dump_filenameN
 *
 * Streaming uses DPDK: the receiver must run as root (e.g. through the
 * uhd-rfsoc wrapper: uhd-rfsoc gnss-sdr --config_file=...).
 */
class RfsocUhdSignalSource : public SignalSourceBase
{
public:
    RfsocUhdSignalSource(const ConfigurationInterface* configuration,
        const std::string& role, unsigned int in_stream,
        unsigned int out_stream, Concurrent_Queue<pmt::pmt_t>* queue);

    ~RfsocUhdSignalSource() = default;

    inline size_t item_size() override
    {
        return item_size_;
    }

    void connect(gr::top_block_sptr top_block) override;
    void disconnect(gr::top_block_sptr top_block) override;
    gr::basic_block_sptr get_left_block() override;
    gr::basic_block_sptr get_right_block() override;
    gr::basic_block_sptr get_right_block(int RF_channel) override;

private:
    void read_channel_config(const ConfigurationInterface* configuration,
        const std::string& role, int channel);
    void open_device(const std::string& device_args);
    void tune_channel(int channel);
    void build_decimators(int channel, double tune_offset_hz);
    gr::basic_block_sptr decimated_output(int channel) const;

    gr::uhd::usrp_source::sptr uhd_source_;

    // per RF channel: the decimation chain, first block first
    std::vector<std::vector<gr::basic_block_sptr>> decimators_;
    std::vector<gnss_shared_ptr<gr::block>> valve_;
    std::vector<gr::blocks::file_sink::sptr> file_sink_;

    std::vector<double> freq_;
    std::vector<double> gain_;
    std::vector<uint64_t> samples_;
    std::vector<std::string> dump_filename_;
    std::vector<bool> dump_;

    std::vector<rfsoc_uhd::Decimation_Stage> plan_;

    std::string otw_format_;
    std::string clock_source_;
    double device_rate_sps_;
    double sample_rate_sps_;
    double passband_hz_;
    double stopband_attenuation_db_;
    size_t item_size_;
    int RF_channels_;
    unsigned int in_stream_;
    unsigned int out_stream_;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_RFSOC_UHD_SIGNAL_SOURCE_H
