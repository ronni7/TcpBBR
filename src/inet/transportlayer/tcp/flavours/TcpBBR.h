#ifndef __INET_TCPBBR_H
#define __INET_TCPBBR_H

#include "inet/common/INETDefs.h"
#include "inet/transportlayer/tcp/flavours/TcpBaseAlg.h"

namespace inet {
namespace tcp {

class INET_API TcpBBRStateVariables : public TcpBaseAlgStateVariables
{
};

class INET_API TcpBBR : public TcpBaseAlg
{
protected:
    TcpBBRStateVariables *& state;    // alias to TcpAlgorithm's 'state'

    virtual TcpStateVariables *createStateVariables() override;

public:
    /** Ctor */
    TcpBBR();

    /** Redefine what should happen when data got acked, to add congestion window management */
    virtual void receivedDataAck(uint32 firstSeqAcked) override;

    /** Redefine what should happen when dupAck was received, to add congestion window management */
    virtual void receivedDuplicateAck() override;

    /** Called after we send data */
    virtual void dataSent(uint32 fromseq) override;
};

} // namespace tcp
} // namespace inet

#endif
