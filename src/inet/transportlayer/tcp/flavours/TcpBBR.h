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
class BBR {
public:
    uint32 total_delivered = 0;
    uint32 packet_delivered = 0; // "packet.delivered" as specified in technical draft
    uint32 next_round_delivered = 0;
    uint32 round_count = 0;
    uint32 BtlBwFilter[BtlBwFilterLen];
    uint32 BtlBwFilterIndex = 0;
    uint32 BtlBwFilterLastSampleTime = 0;
    uint32 BtlBw = 0; //bottleneck bandwidth
    uint32 full_bw = 0;
    uint32 full_bw_count = 0;
    uint32 cycle_stamp = 0;
    uint32 cycle_index = 0;
    uint32 rtprop_stamp = 0;
    uint32 send_quantum = 0;
    uint32 probe_rtt_done_stamp = 0;
    uint32 prior_cwnd = 0;
    uint32 target_cwnd = 0;
    double pacing_gain = BBRHighGainValue;
    double RTprop = INFINITY;
    double cwnd_gain = BBRHighGainValue;
    double pacing_rate = 0;
    double packet_send_time = 0;
    double packet_rtt = 0;
    uint32 packets_in_flight = 0;
    uint32 packets_delivered = 0;
    uint32 prior_inflight = 0;
    uint32 packets_lost = 0;
    State state = Startup;
    bool round_start = false;
    bool filled_pipe = false;
    bool rtprop_expired = false;
    bool idle_restart = false;
    bool packet_conservation = false;
    bool probe_rtt_round_done= false;
};

class INET_API TcpBBRStateVariables: public TcpBaseAlgStateVariables {
public:
    BBR* BBR;
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
