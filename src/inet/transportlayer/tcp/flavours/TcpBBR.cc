#include "inet/transportlayer/tcp/Tcp.h"
#include "inet/transportlayer/tcp/flavours/TcpBBR.h"

namespace inet {
namespace tcp {

Register_Class(TcpBBR);

TcpBBR::TcpBBR()
    : TcpBaseAlg(), state((TcpBBRStateVariables *&)TcpAlgorithm::state)
{
}

TcpStateVariables *TcpBBR::createStateVariables() {
    return new TcpBBRStateVariables();
}

void TcpBBR::receivedDataAck(uint32 firstSeqAcked)
{
    EV_INFO << "TcpBBR::receivedDataAck executed" << "\n";
    TcpBaseAlg::receivedDataAck(firstSeqAcked);
}

void TcpBBR::receivedDuplicateAck()
{
    EV_INFO << "TcpBBR::receivedDuplicateAck executed" << "\n";
    TcpBaseAlg::receivedDuplicateAck();
}

void TcpBBR::dataSent(uint32 fromseq)
{
    EV_INFO << "TcpBBR::dataSent executed" << "\n";
    TcpBaseAlg::dataSent(fromseq);
}

}
}
