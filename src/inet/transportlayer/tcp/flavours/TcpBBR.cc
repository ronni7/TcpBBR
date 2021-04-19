#include "inet/transportlayer/tcp/Tcp.h"
#include "inet/transportlayer/tcp/flavours/TcpBBR.h"

namespace inet {
namespace tcp {

Register_Class(TcpBBR);

TcpBBR::TcpBBR() :
        TcpBaseAlg(), state((TcpBBRStateVariables *&) TcpAlgorithm::state) {
    BBRInit();
}

TcpStateVariables *TcpBBR::createStateVariables() {
    return new TcpBBRStateVariables();
}

void TcpBBR::receivedDataAck(uint32 firstSeqAcked) {
    EV_INFO << "TcpBBR::receivedDataAck executed" << "\n";
    TcpBaseAlg::receivedDataAck(firstSeqAcked);
    state->BBR_packet_rtt = simTime().inUnit(SIMTIME_MS) - state->BBR_packet_send_time;
    state->BBR_prior_inflight = state->BBR_packets_in_flight;
    state->BBR_packets_in_flight--;
    state->BBR_packets_delivered++;
    BBRUpdateModelAndState();
    BBRUpdateControlParameters();
}

void TcpBBR::dataSent(uint32 fromseq) {
    EV_INFO << "TcpBBR::dataSent executed" << "\n";
    TcpBaseAlg::dataSent(fromseq);
    //Upon sending each packet transmission:    packet.delivered = BBR.delivered
    state->BBR_packet_delivered = state->BBR_total_delivered;
    state->BBR_packet_send_time = simTime().inUnit(SIMTIME_MS);
    state->BBR_packets_in_flight++;
    if (state->BBR_packets_lost > 0) {
        state->BBR_packets_lost--;
    }
    BBRHandleRestartFromIdle();
}

void TcpBBR::BBRUpdateModelAndState() {
    BBRUpdateBtlBw();
    BBRCheckCyclePhase();
    BBRCheckFullPipe();
    BBRCheckDrain();
    BBRUpdateRTprop();
    BBRCheckProbeRTT();
}

void TcpBBR::BBRUpdateControlParameters() {
    BBRSetPacingRate();
    BBRSetSendQuantum();
    BBRSetCwnd();
}

void TcpBBR::BBRUpdateRound() {
    state->BBR_total_delivered += state->sentBytes;
    if (state->BBR_packet_delivered >= state->BBR_next_round_delivered) {
        state->BBR_next_round_delivered = state->BBR_total_delivered;
        state->BBR_round_count++;
        state->BBR_round_start = true;
    } else {
        state->BBR_round_start = false;
    }
}

void TcpBBR::BBRUpdateBtlBw() {
    BBRUpdateRound();
    uint32 delivery_rate = state->srtt.inUnit(SIMTIME_MS);
    if (delivery_rate >= state->BBR_BtlBw) {
        state->BBR_BtlBw = UpdateBtlBwFilterAndGetMax(delivery_rate, state->BBR_round_count);
    }
}

void TcpBBR::BBRCheckCyclePhase() {
    if (state->BBR_state == ProbeBW && BBRIsNextCyclePhase()) {
        BBRAdvanceCyclePhase();
    }
}

void TcpBBR::BBRAdvanceCyclePhase() {
    state->BBR_cycle_stamp = simTime().inUnit(SIMTIME_MS);
    state->BBR_cycle_index = (state->BBR_cycle_index + 1) % BBRGainCycleLen;
    switch (state->BBR_cycle_index) {
    case 0:
        state->BBR_pacing_gain = 5.0 / 4.0;
        break;
    case 1:
        state->BBR_pacing_gain = 3.0 / 4.0;
        break;
    default:
        state->BBR_pacing_gain = 1;
        break;
    }
}

bool TcpBBR::BBRIsNextCyclePhase() {
    bool is_full_length = (simTime().inUnit(SIMTIME_MS)
            - state->BBR_cycle_stamp) > state->BBR_RTprop;
    if (state->BBR_pacing_gain == 1) {
        return is_full_length;
    }
    //Here, "prior_inflight" is the amount of data that was in flight before processing this ACK.
    if (state->BBR_pacing_gain > 1) {
        return is_full_length
                && (state->BBR_packets_lost > 0
                        || state->BBR_prior_inflight
                                >= BBRInFlight(state->BBR_pacing_gain));
    } else { //  (BBR.pacing_gain < 1)
        return is_full_length || state->BBR_prior_inflight <= BBRInFlight(1.0);
    }
}

void TcpBBR::BBRCheckFullPipe() {
    if (state->BBR_filled_pipe || !state->BBR_round_start) {
        return;  // no need to check for a full pipe now
    }
    if (state->BBR_BtlBw >= state->BBR_full_bw * 1.25) { // BBR.BtlBw still growing?
        state->BBR_full_bw = state->BBR_BtlBw;    // record new baseline level
        state->BBR_full_bw_count = 0;
        return;
    }
    state->BBR_full_bw_count++; // another round w/o much growth
    if (state->BBR_full_bw_count >= 3) {
        state->BBR_filled_pipe = true;
    }
}

void TcpBBR::BBRCheckDrain() {
    if (state->BBR_state == Startup && state->BBR_filled_pipe)
        // BBREnterDrain() :
        state->BBR_state = Drain;
    state->BBR_pacing_gain = 1 / BBRHighGain;  // pace slowly
    state->BBR_cwnd_gain = BBRHighGain;    // maintain cwnd
    if (state->BBR_state == Drain && state->BBR_packets_in_flight <= BBRInFlight(1.0))
        BBREnterProbeBW();  // we estimate queue is drained
}

void TcpBBR::BBRUpdateRTprop() {
    state->BBR_rtprop_expired = simTime().inUnit(SIMTIME_MS)
            > state->BBR_rtprop_stamp + RTpropFilterLen;
    if (state->BBR_packet_rtt >= 0
            && (state->BBR_packet_rtt <= state->BBR_RTprop
                    || state->BBR_rtprop_expired)) {
        state->BBR_RTprop = state->BBR_packet_rtt;
        state->BBR_rtprop_stamp = simTime().inUnit(SIMTIME_MS);
    }
}

void TcpBBR::BBRCheckProbeRTT() {
    if (state->BBR_state != ProbeRTT && state->BBR_rtprop_expired
            && !state->BBR_idle_restart) {
        BBREnterProbeRTT();
        BBRSaveCwnd();
        state->BBR_probe_rtt_done_stamp = 0;
    }
    if (state->BBR_state == ProbeRTT) {
        BBRHandleProbeRTT();
    }
    state->BBR_idle_restart = false;
}

void TcpBBR::BBREnterProbeRTT() {
    state->BBR_state = ProbeRTT;
    state->BBR_pacing_gain = 1;
    state->BBR_cwnd_gain = 1;
}

void TcpBBR::BBRHandleProbeRTT() {
    /* Ignore low rate samples during ProbeRTT: */
    if (state->BBR_probe_rtt_done_stamp == 0
            && state->BBR_packets_in_flight <= BBRMinPipeCwndFactor * state->snd_mss) {
        state->BBR_probe_rtt_done_stamp = simTime().inUnit(SIMTIME_MS)
                + ProbeRTTDuration;
        state->BBR_probe_rtt_round_done = false;
        state->BBR_next_round_delivered = state->BBR_total_delivered;
    } else if (state->BBR_probe_rtt_done_stamp != 0) {
        if (state->BBR_round_start) {
            state->BBR_probe_rtt_round_done = true;
        }
        if (state->BBR_probe_rtt_round_done
                && simTime().inUnit(SIMTIME_MS)
                        > state->BBR_probe_rtt_done_stamp) {
            state->BBR_rtprop_stamp = simTime().inUnit(SIMTIME_MS);
            // BBRRestoreCwnd() :
            state->snd_cwnd = fmax(state->snd_cwnd, state->BBR_prior_cwnd);
            BBRExitProbeRTT();
        }
    }
}

void TcpBBR::BBRExitProbeRTT() {
    if (state->BBR_filled_pipe) {
        BBREnterProbeBW();
    } else {
        // BBREnterStartup() :
        state->BBR_state = Startup;
        state->BBR_pacing_gain = BBRHighGain;
        state->BBR_cwnd_gain = BBRHighGain;
    }
}

void TcpBBR::BBRSetPacingRate() {
    BBRSetPacingRateWithGain(state->BBR_pacing_gain);
}

void TcpBBR::BBRSetSendQuantum() {
    if (state->BBR_pacing_rate < 1200000) { // 1.2 Mbps
        state->BBR_send_quantum = state->snd_mss;
    } else if (state->BBR_pacing_rate < 24000000) { // 24 Mbps
        state->BBR_send_quantum = 2 * state->snd_mss;
    } else {
        state->BBR_send_quantum = fmin(state->BBR_pacing_rate, 64000); // fmin(BBR.pacing_rate * 1ms, 64KBytes)
    }
}

void TcpBBR::BBRSetCwnd() {
    BBRUpdateTargetCwnd();
    BBRModulateCwndForRecovery();
    if (!state->BBR_packet_conservation) {
        if (state->BBR_filled_pipe) {
            state->snd_cwnd = fmin(
                    state->snd_cwnd + state->BBR_packets_delivered,
                    state->BBR_target_cwnd);
        } else if (state->snd_cwnd < state->BBR_target_cwnd
                || state->BBR_total_delivered < BBRHighGain) {
            state->snd_cwnd = state->snd_cwnd + state->BBR_packets_delivered;
        }
        state->snd_cwnd = fmax(state->snd_cwnd, BBRMinPipeCwndFactor * state->snd_mss);
    }
    BBRModulateCwndForProbeRTT();
}

void TcpBBR::BBRModulateCwndForRecovery() {
    if (state->BBR_packets_lost > 0) {
        state->snd_cwnd = fmax(state->snd_cwnd - state->BBR_packets_lost, 1);
    }
    if (state->BBR_packet_conservation) {
        state->snd_cwnd = fmax(state->snd_cwnd,
                state->BBR_packets_in_flight + state->BBR_packet_delivered);
    }
}

void TcpBBR::BBRModulateCwndForProbeRTT() {
    if (state->BBR_state == ProbeRTT) {
        state->snd_cwnd = fmin(state->snd_cwnd, BBRMinPipeCwndFactor * state->snd_mss);
    }
}

void TcpBBR::BBRHandleRestartFromIdle() {
    if (state->BBR_packets_in_flight == 0) {
        state->BBR_idle_restart = true;
        if (state->BBR_state == ProbeBW) {
            BBRSetPacingRateWithGain(1);
        }
    }
}

void TcpBBR::BBRInit() {
}

uint32 TcpBBR::BBRInFlight(double gain) {
    if (state->BBR_RTprop == INFINITY) {
        return state->snd_cwnd; /* no valid RTT samples yet */
    }
    double quanta = 3 * state->BBR_send_quantum;
    double estimated_bdp = state->BBR_BtlBw * state->BBR_RTprop;
    return gain * estimated_bdp + quanta;
}

void TcpBBR::BBRUpdateTargetCwnd() {
    state->BBR_target_cwnd = TcpBBR::BBRInFlight(state->BBR_cwnd_gain);
}

void TcpBBR::BBREnterProbeBW() {
    state->BBR_state = ProbeBW;
    state->BBR_pacing_gain = 1;
    state->BBR_cwnd_gain = 2;
    state->BBR_cycle_index = BBRGainCycleLen - 1 - (rand() % 7);
    BBRAdvanceCyclePhase();
}

void TcpBBR::BBRSetPacingRateWithGain(double pacing_gain) {
    double rate = state->BBR_pacing_gain * state->BBR_BtlBw;
    if (state->BBR_filled_pipe || rate > state->BBR_pacing_rate) {
        state->BBR_pacing_rate = rate;
    }
}
uint32 TcpBBR::BBRSaveCwnd() {
    if (state->BBR_state != ProbeRTT && !state->lossRecovery)
        return state->snd_cwnd;
    else
        return fmax(state->BBR_prior_cwnd, state->snd_cwnd);
}

void TcpBBR::segmentRetransmitted(uint32 fromseq, uint32 toseq) {
    TcpBaseAlg::segmentRetransmitted(fromseq, toseq);
    // Upon retransmission all packets in flight are considered lost.
    state->BBR_packets_lost += state->BBR_packets_in_flight;
    state->BBR_packets_in_flight = 0;
    // Steps after RTO:
    state->BBR_prior_cwnd = BBRSaveCwnd();
    state->snd_cwnd = 1;
}

uint32 TcpBBR::UpdateBtlBwFilterAndGetMax(uint32 value, uint32 time) {
    uint32 amountOfTime = time - state->BBR_BtlBwFilterLastSampleTime;
    while (amountOfTime-- > 0) {
        state->BBR_BtlBwFilter[state->BBR_BtlBwFilterIndex] = value;
        state->BBR_BtlBwFilterIndex++;
        if (state->BBR_BtlBwFilterIndex >= BtlBwFilterLen) {
            state->BBR_BtlBwFilterIndex = 0;
        }
    }
    state->BBR_BtlBwFilterLastSampleTime = time;

    uint32 max = 0;
    for (uint32 i = 0; i < BtlBwFilterLen; i++) {
        if (state->BBR_BtlBwFilter[state->BBR_BtlBwFilterIndex] > max) {
            max = state->BBR_BtlBwFilter[state->BBR_BtlBwFilterIndex];
        }
    }
    return max;
}

}
}
