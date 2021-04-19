#ifndef __INET_TCPBBR_H
#define __INET_TCPBBR_H

#define BtlBwFilterLen 10

#include "inet/common/INETDefs.h"
#include "inet/transportlayer/tcp/flavours/TcpBaseAlg.h"

namespace inet {
namespace tcp {

enum State {
    Startup, Drain, ProbeBW, ProbeRTT
};
const double BBRHighGainValue = 2.89;

class INET_API TcpBBRStateVariables: public TcpBaseAlgStateVariables {
public:
    uint32 BBR_total_delivered = 0;
    uint32 BBR_packet_delivered = 0; // "packet.delivered" as specified in technical draft
    uint32 BBR_next_round_delivered = 0;
    uint32 BBR_round_count = 0;
    uint32 BBR_BtlBwFilter[BtlBwFilterLen];
    uint32 BBR_BtlBwFilterIndex = 0;
    uint32 BBR_BtlBwFilterLastSampleTime = 0;
    uint32 BBR_BtlBw = 0; //bottleneck bandwidth
    uint32 BBR_full_bw = 0;
    uint32 BBR_full_bw_count = 0;
    uint32 BBR_cycle_stamp = 0;
    uint32 BBR_cycle_index = 0;
    uint32 BBR_rtprop_stamp = 0;
    uint32 BBR_send_quantum = 0;
    uint32 BBR_probe_rtt_done_stamp = 0;
    uint32 BBR_prior_cwnd = 0;
    uint32 BBR_target_cwnd = 0;
    double BBR_pacing_gain = BBRHighGainValue;
    double BBR_RTprop = INFINITY;
    double BBR_cwnd_gain = BBRHighGainValue;
    double BBR_pacing_rate = 0;
    double BBR_packet_send_time = 0;
    double BBR_packet_rtt = 0;
    uint32 BBR_packets_in_flight = 0;
    uint32 BBR_packets_delivered = 0;
    uint32 BBR_prior_inflight = 0;
    uint32 BBR_packets_lost = 0;
    State BBR_state = Startup;
    bool BBR_round_start = false;
    bool BBR_filled_pipe = false;
    bool BBR_rtprop_expired = false;
    bool BBR_idle_restart = false;
    bool BBR_packet_conservation = false;
    bool BBR_probe_rtt_round_done= false;
};

class INET_API TcpBBR: public TcpBaseAlg {
protected:
    const uint32 BBRGainCycleLen = 8;
    const uint32 RTpropFilterLen = 10000; // 10 s
    const double BBRHighGain = BBRHighGainValue;
    const uint32 ProbeRTTDuration = 200; // 200 ms
    const uint32 BBRMinPipeCwndFactor = 4; // BBRMinPipeCwnd = 4 * SMSS

    TcpBBRStateVariables *& state;    // alias to TcpAlgorithm's 'state'

    virtual TcpStateVariables *createStateVariables() override;

    void BBRUpdateModelAndState();
    void BBRUpdateControlParameters();
    void BBRUpdateRound();
    void BBRUpdateBtlBw();
    void BBRCheckCyclePhase();
    void BBRAdvanceCyclePhase();
    bool BBRIsNextCyclePhase();
    void BBRCheckFullPipe();
    void BBRCheckDrain();
    void BBRUpdateRTprop();
    void BBRCheckProbeRTT();
    void BBREnterProbeRTT();
    void BBRHandleProbeRTT();
    void BBRExitProbeRTT();
    void BBRSetPacingRate();
    void BBRSetSendQuantum();
    void BBRSetCwnd();
    void BBRModulateCwndForRecovery();
    void BBRModulateCwndForProbeRTT();
    void BBRHandleRestartFromIdle();
    void BBRInit();
    uint32 BBRInFlight(double gain);
    void BBRUpdateTargetCwnd();
    void BBREnterProbeBW();
    void BBRSetPacingRateWithGain(double pacing_gain);
    uint32 BBRSaveCwnd();
    uint32 UpdateBtlBwFilterAndGetMax(uint32 value, uint32 time);
;
public:
    /** Ctor */
    TcpBBR();

    /** Redefine what should happen when data got acked, to add congestion window management */
    virtual void receivedDataAck(uint32 firstSeqAcked) override;

    /** Called after we send data */
    virtual void dataSent(uint32 fromseq) override;

    virtual void segmentRetransmitted(uint32 fromseq, uint32 toseq) override;
};

} // namespace tcp
} // namespace inet

#endif
