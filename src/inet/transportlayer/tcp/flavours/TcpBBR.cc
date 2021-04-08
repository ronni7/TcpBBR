#include "inet/transportlayer/tcp/Tcp.h"
#include "inet/transportlayer/tcp/flavours/TcpBBR.h"

namespace inet {
namespace tcp {

Register_Class(TcpBBR);

TcpBBR::TcpBBR()
    : TcpBaseAlg(), state((TcpBBRStateVariables *&)TcpAlgorithm::state)
{
    BBRInit();
}

TcpStateVariables *TcpBBR::createStateVariables() {
    return new TcpBBRStateVariables();
}

void TcpBBR::receivedDataAck(uint32 firstSeqAcked)
{
    EV_INFO << "TcpBBR::receivedDataAck executed" << "\n";
    TcpBaseAlg::receivedDataAck(firstSeqAcked);

    BBRUpdateModelAndState();
    BBRUpdateControlParameters();
}

void TcpBBR::dataSent(uint32 fromseq)
{
    EV_INFO << "TcpBBR::dataSent executed" << "\n";
    TcpBaseAlg::dataSent(fromseq);

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
    state->BBR->delivered += state->sentBytes; // packet->size
    if (packet->delivered /* ??? */ >= state->BBR->next_round_delivered) {
        state->BBR->next_round_delivered = state->BBR->delivered;
        state->BBR->round_count++;
        state->BBR->round_start = true;
    }
    else {
        state->BBR->round_start = false;
    }
}

void TcpBBR::BBRUpdateBtlBw() {
    BBRUpdateRound();
    if (rs.delivery_rate >= state->BBR->BtlBw || ! rs.is_app_limited)  {
        state->BBR->BtlBw = update_windowed_max_filter( /* ??? */
                      filter=state->BBR->BtlBwFilter,
                      value=state->BBR->delivery_rate,
                      time=state->BBR->round_count,
                      window_length=BtlBwFilterLen);
    }
}

void TcpBBR::BBRCheckCyclePhase() {
    if (state->BBR->state == ProbeBW && BBRIsNextCyclePhase()) {
        BBRAdvanceCyclePhase();
    }
}

void TcpBBR::BBRAdvanceCyclePhase() {
    state->BBR->cycle_stamp = simTime().inUnit(ms);
    state->BBR->cycle_index = (state->BBR->cycle_index + 1) % BBRGainCycleLen;
    switch (state->BBR->cycle_index) {
    case 0:
        state->BBR->pacing_gain = 5.0/4.0;
        break;
    case 1:
        state->BBR->pacing_gain = 3.0/4.0;
        break;
    default:
        state->BBR->pacing_gain = 1;
        break;
    }
}

bool TcpBBR::BBRIsNextCyclePhase() {
    bool is_full_length = (simTime().inUnit(ms) - state->BBR->cycle_stamp) > state->BBR->RTprop;
    if (state->BBR->pacing_gain == 1) {
      return is_full_length;
    }
    if (state->BBR->pacing_gain > 1) {
      return is_full_length && (packets_lost > 0 || prior_inflight >= BBRInflight(state->BBR->pacing_gain));
    }
    else { //  (BBR.pacing_gain < 1)
      return is_full_length || prior_inflight <= BBRInflight(1);
    }
}

void TcpBBR::BBRCheckFullPipe() {
    if (state->BBR->filled_pipe || !state->BBR->round_start || rs.is_app_limited) {
        return;  // no need to check for a full pipe now
    }
    if (state->BBR->BtlBw >= state->BBR->full_bw * 1.25) {  // BBR.BtlBw still growing?
        state->BBR->full_bw = state->BBR->BtlBw;    // record new baseline level
        state->BBR->full_bw_count = 0;
        return;
    }
    state->BBR->full_bw_count++; // another round w/o much growth
    if (state->BBR->full_bw_count >= 3) {
        state->BBR->filled_pipe = true;
    }
}

void TcpBBR::BBRCheckDrain() {
    if (state->BBR->state == Startup and state->BBR->filled_pipe)
        // BBREnterDrain() :
        state->BBR->state = Drain;
        state->BBR->pacing_gain = 1 / BBRHighGain;  // pace slowly
        state->BBR->cwnd_gain = BBRHighGain;    // maintain cwnd
    if (state->BBR->state == Drain and packets_in_flight <= BBRInFlight(1.0))
        BBREnterProbeBW();  // we estimate queue is drained
}

void TcpBBR::BBRUpdateRTprop() {
    state->BBR->rtprop_expired = simTime().inUnit(ms) > state->BBR->rtprop_stamp + RTpropFilterLen;
    if (packet.rtt >= 0 && (packet.rtt <= state->BBR->RTprop || state->BBR->rtprop_expired)) {
        state->BBR->RTprop = packet.rtt;
        state->BBR->rtprop_stamp = simTime().inUnit(ms);
    }
}

void TcpBBR::BBRCheckProbeRTT() {
    if (state->BBR->state != ProbeRTT && state->BBR->rtprop_expired && !state->BBR->idle_restart) {
        BBREnterProbeRTT();
        BBRSaveCwnd();
        state->BBR->probe_rtt_done_stamp = 0;
    }
    if (state->BBR->state == ProbeRTT) {
        BBRHandleProbeRTT();
    }
    state->BBR->idle_restart = false;
}

void TcpBBR::BBREnterProbeRTT() {
    state->BBR->state = ProbeRTT;
    state->BBR->pacing_gain = 1;
    state->BBR->cwnd_gain = 1;
}

void TcpBBR::BBRHandleProbeRTT() {
    /* Ignore low rate samples during ProbeRTT: */
    C.app_limited = (BW.delivered + packets_in_flight) ? C.app_limited : 1;
    if (state->BBR->probe_rtt_done_stamp == 0 && packets_in_flight <= BBRMinPipeCwnd) {
        state->BBR->probe_rtt_done_stamp = simTime().inUnit(ms) + ProbeRTTDuration;
        state->BBR->probe_rtt_round_done = false;
        state->BBR->next_round_delivered = state->BBR->delivered;
    }
    else if (state->BBR->probe_rtt_done_stamp != 0) {
        if (state->BBR->round_start) {
            state->BBR->probe_rtt_round_done = true;
        }
        if (state->BBR->probe_rtt_round_done && simTime().inUnit(ms) > state->BBR->probe_rtt_done_stamp){
            state->BBR->rtprop_stamp = simTime().inUnit(ms);
            // BBRRestoreCwnd() :
            state->snd_cwnd = std::max(state->snd_cwnd, state->BBR->prior_cwnd);
            BBRExitProbeRTT();
        }
    }
}

void TcpBBR::BBRExitProbeRTT() {
    if (state->BBR->filled_pipe) {
        BBREnterProbeBW();
    }
    else {
        // BBREnterStartup() :
        state->BBR->state = Startup;
        state->BBR->pacing_gain = BBRHighGain;
        state->BBR->cwnd_gain = BBRHighGain;
    }
}

void TcpBBR::BBRSetPacingRate() {
    BBRSetPacingRateWithGain(state->BBR->pacing_gain);
}

void TcpBBR::BBRSetSendQuantum() {
    if (state->BBR->pacing_rate < 1200000) { // 1.2 Mbps
        state->BBR->send_quantum = state->snd_mss;
    }
    else if (state->BBR->pacing_rate < 24000000) { // 24 Mbps
        state->BBR->send_quantum  = 2 * state->snd_mss;
    }
    else {
        state->BBR->send_quantum  = std::min(state->BBR->pacing_rate, 64000); // min(BBR.pacing_rate * 1ms, 64KBytes)
    }
}

void TcpBBR::BBRSetCwnd() {
    BBRUpdateTargetCwnd();
    BBRModulateCwndForRecovery();
    if (!state->BBR->packet_conservation) {
        if (state->BBR->filled_pipe) {
            state->snd_cwnd = min(state->snd_cwnd + packets_delivered, state->BBR->target_cwnd);
        }
        else if (state->snd_cwnd < state->BBR->target_cwnd || state->BBR->delivered < BBRHighGain) {
            state->snd_cwnd = state->snd_cwnd + packets_delivered;
        }
        state->snd_cwnd = std::max(state->snd_cwnd, BBRMinPipeCwnd);
    }
    BBRModulateCwndForProbeRTT();
}

void TcpBBR::BBRModulateCwndForRecovery() {
    if (packets_lost > 0) {
        state->snd_cwnd = std::max(state->snd_cwnd - packets_lost, 1);
    }
    if (state->BBR->packet_conservation) {
        state->snd_cwnd = std::max(state->snd_cwnd, packets_in_flight + packets_delivered);
    }
}

void TcpBBR::BBRModulateCwndForProbeRTT() {
    if (state->BBR->state == ProbeRTT) {
        state->snd_cwnd = std::min(state->snd_cwnd, BBRMinPipeCwnd);
    }
}

void TcpBBR::BBRHandleRestartFromIdle() {
    if (packets_in_flight == 0 && C.app_limited) {
        state->BBR->idle_restart = true;
        if (state->BBR->state == ProbeBW) {
            BBRSetPacingRateWithGain(1);
        }
    }
}

void TcpBBR::BBRInit() {
    state->BBR = new BBR();
}

bool TcpBBR::BBRIsNextCyclePhase() {
    bool is_full_length = (simTime().inUnit(ms) - state->BBR->cycle_stamp) > state->BBR->RTprop;
    if (state->BBR->pacing_gain == 1)
      return is_full_length;
    if (state->BBR->pacing_gain > 1)
      return is_full_length && (packets_lost > 0 || prior_inflight >= BBRInflight(state->BBR->pacing_gain));
    else  //  (BBR.pacing_gain < 1)
      return is_full_length || prior_inflight <= BBRInflight(1);
}

uint32 TcpBBR::BBRInFlight(double gain) {
    if (state->BBR->RTprop == std::numeric_limits<double>::infinity) {
        return state->snd_cwnd; /* no valid RTT samples yet */
    }
    double quanta = 3 * state->BBR->send_quantum;
    double estimated_bdp = state->BBR->BtlBw * state->BBR->RTprop;
    return gain * estimated_bdp + quanta;
}

void TcpBBR::BBRUpdateTargetCwnd() {
    state->snd_cwnd = TcpBBR::BBRInFlight(state->BBR->cwnd_gain);
}

void TcpBBR::BBREnterProbeBW() {
    state->BBR->state = ProbeBW;
    state->BBR->pacing_gain = 1;
    state->BBR->cwnd_gain = 2;
    state->BBR->cycle_index = BBRGainCycleLen - 1 - (rand() % 7);
    BBRAdvanceCyclePhase();
}

void TcpBBR::BBRSetPacingRateWithGain(double pacing_gain) {
    double rate = state->BBR->pacing_gain * state->BBR->BtlBw;
    if (state->BBR->filled_pipe || rate > state->BBR->pacing_rate) {
        state->BBR->pacing_rate = rate;
    }
}


}
}
