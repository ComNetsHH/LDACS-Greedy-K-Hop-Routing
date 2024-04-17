//
// Copyright (C) 2013 Opensim Ltd
// Author: Levente Meszaros
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
//

#ifndef __INET_GREEDY_H
#define __INET_GREEDY_H

#include "inet/common/INETDefs.h"
#include "inet/common/geometry/common/Coord.h"
#include "inet/common/packet/Packet.h"
#include "inet/mobility/contract/IMobility.h"
#include "inet/networklayer/contract/IL3AddressType.h"
#include "inet/networklayer/contract/INetfilter.h"
#include "inet/networklayer/contract/IRoutingTable.h"
#include "inet/routing/base/RoutingProtocolBase.h"
#include "Greedy_m.h"
#include "inet/transportlayer/udp/UdpHeader_m.h"
#include "PositionTableModified.h"
#include "inet/physicallayer/contract/packetlevel/IRadio.h"



using namespace inet;

/**
 * This class implements the Greedy Perimeter Stateless Routing for Wireless Networks.
 * The implementation supports both GG and RNG planarization algorithms.
 *
 * For more information on the routing algorithm, see the GPSR paper
 * http://www.eecs.harvard.edu/~htk/publication/2000-mobi-karp-kung.pdf
 */
// TODO: optimize internal data structures for performance to use less lookups and be more prepared for routing a packet
// TODO: implement position piggybacking that is all packets should carry the position of the sender, all packets act as a beacon and reset beacon timer
// TODO: implement promiscuous mode, all receivers should process all packets with respect to neighbor positions
// KLUDGE: implement position registry protocol instead of using a global variable
class Greedy : public RoutingProtocolBase, public cListener, public NetfilterBase::HookBase
{
  private:
    // Greedy Forwarding parameters
    const char *interfaces = nullptr;
    simtime_t beaconInterval;
    simtime_t maxJitter;
    simtime_t neighborValidityInterval;
    bool displayBubbles;
    double commRange; // Communication range
    double thresholdDistance; 
    inet::physicallayer::IRadio *radio;
    double forwardingRadiusReduction;
    double GSx = 0.0;
    double GSy = 0.0;
    double GSz = 0.0;
    //Name of the Trace file where list of ground stations are given
    const char* groundstationsTraceFile ;
    virtual void parseGroundstationTraceFile2Vector(const char* file_name);
    virtual int findClosestGroundStation();
    //Declaring Vector to store the ground station coordinates and ethernet
    std::vector<std::vector<double>> ground_stations_coordinates_array;
    std::vector<std::string> ethernet_vector;

    bool beaconForwardedFromGpsr;
    bool useTwoHopGreedy;
    bool useThreeHopGreedy;
    bool useFFT;
    bool useEnforceEFFT;
    bool useRandom;
    bool useEFFT;

    // context
    cModule *host = nullptr;
    IMobility *mobility = nullptr;
    IL3AddressType *addressType = nullptr;
    IInterfaceTable *interfaceTable = nullptr;
    const char *outputInterface = nullptr;
    const char *a2gOutputInterface = nullptr;
    IRoutingTable *routingTable = nullptr;    // TODO: delete when necessary functions are moved to interface table
    INetfilter *networkProtocol = nullptr;
    static PositionTableModified globalPositionTable;    // KLUDGE: implement position registry protocol
    // packet size
    int positionByteLength = -1;
    int addressByteLength = -1;
    int sequenceByteLength = -1;
    int hopsByteLength = -1;

    // internal
    cMessage *beaconTimer = nullptr;
    cMessage *purgeNeighborsTimer = nullptr;
    PositionTableModified neighborPositionTable;

  public:
    Greedy();
    virtual ~Greedy();
    
  protected:
    // module interface
    virtual int numInitStages() const override { return NUM_INIT_STAGES; }
    void initialize(int stage) override;
    virtual void handleMessageWhenDown(cMessage *message) override;
    void handleMessageWhenUp(cMessage *message) override;

  private:
    // handling messages
    void processSelfMessage(cMessage *message);
    void processMessage(cMessage *message);

    // handling beacon timers
    void scheduleBeaconTimer();
    void processBeaconTimer();

    // handling purge neighbors timers
    void schedulePurgeNeighborsTimer();
    void processPurgeNeighborsTimer();

    // handling UDP packets
    void sendUdpPacket(Packet *packet);
    void processUdpPacket(Packet *packet);

    // handling beacons
    size_t numberOfSelectedNeighbors; // Use size_t in C++
    // size_t numberOfSelectedNeighbors; // Use size_t in C++
    void kHopNeighborInfoToBeaconFFT(const Ptr<GreedyBeacon>& beacon, size_t m);
    void kHopNeighborInfoToBeaconEFFT(const Ptr<GreedyBeacon>& beacon, size_t m);
    void kHopNeighborInfoToBeaconEnforceEFFT(const Ptr<GreedyBeacon>& beacon, size_t m);
    void kHopNeighborInfoToBeaconRandom(const Ptr<GreedyBeacon>& beacon, size_t m);
    void kHopNeighborInfoToBeacon(const Ptr<GreedyBeacon>& beacon);
    L3AddressVector selectRandomNeighbors(L3AddressVector& neighborAddresses, size_t m);

    const Ptr<GreedyBeacon> createBeacon();
    void sendBeacon(const Ptr<GreedyBeacon>& beacon);
    void processBeacon(Packet *packet);
    
    // handling packets
    GreedyOption *createGreedyOption(L3Address destination);
    int computeOptionLength(GreedyOption *greedyOption);
    void setGreedyOptionOnNetworkDatagram(Packet *packet, const Ptr<const NetworkHeaderBase>& networkHeader, GreedyOption *greedyOption);

    // returns nullptr if not found
    GreedyOption *findGreedyOptionInNetworkDatagramForUpdate(const Ptr<NetworkHeaderBase>& networkHeader);
    const GreedyOption *findGreedyOptionInNetworkDatagram(const Ptr<const NetworkHeaderBase>& networkHeader) const;

    // throws an error when not found
    GreedyOption *getGreedyOptionFromNetworkDatagramForUpdate(const Ptr<NetworkHeaderBase>& networkHeader);
    const GreedyOption *getGreedyOptionFromNetworkDatagram(const Ptr<const NetworkHeaderBase>& networkHeader) const;

    // configuration
    void configureInterfaces();

    // position
    Coord lookupPositionInGlobalRegistry(const L3Address& address) const;
    void storePositionInGlobalRegistry(const L3Address& address, const Coord& position) const;
    void storeSelfPositionInGlobalRegistry() const;
    Coord getNeighborPosition(const L3Address& address) const;
    m groundStationRange;

    // address
    std::string getHostName() const;
    L3Address getSelfAddress() const;
    L3Address getSenderNeighborAddress(const Ptr<const NetworkHeaderBase>& networkHeader) const;

    // neighbor
    // Get next expiration time for 1-hop neighbors
    simtime_t getNextOneHopNeighborExpiration();
    // Get next expiration time for 2-hop neighbors
    simtime_t getNextTwoHopNeighborExpiration();
    // Get next expiration time for 3-hop neighbors
    simtime_t getNextThreeHopNeighborExpiration();
    // Get next expiration time for 4-hop neighbors
    simtime_t getNextFourHopNeighborExpiration();
    // Purge old 1-hop neighbors and 2-hop neighbors and 3-hop neighbors
    void purgeNeighbors();

    // next hop
    L3AddressVector farthestFirstTraversal(L3AddressVector& NeighborAddresses, size_t m);
    L3AddressVector enhancedFarthestFirstTraversal(L3AddressVector& NeighborAddresses, int hops, size_t m);
    L3AddressVector farthestFirstTraversalWithPreSelection(const L3AddressVector& NeighborAddresses, const CoordVector& preSelectedPositions, int hops, size_t m);
    L3AddressVector farthestFirstTraversalWithoutThreshold(const L3AddressVector& NeighborAddresses, const CoordVector& preSelectedPositions, size_t m);
    L3Address findNextHop(const L3Address& destination, GreedyOption *greedyOption);
    L3Address findGreedyRoutingNextHop(const L3Address& destination, GreedyOption *greedyOption);

    // routing
    Result routeDatagram(Packet *datagram, GreedyOption *greedyOption);

    // netfilter
    virtual Result datagramPreRoutingHook(Packet *datagram) override;
    virtual Result datagramForwardHook(Packet *datagram) override { return ACCEPT; }
    virtual Result datagramPostRoutingHook(Packet *datagram) override { return ACCEPT; }
    virtual Result datagramLocalInHook(Packet *datagram) override;
    virtual Result datagramLocalOutHook(Packet *datagram) override;

    // lifecycle
    virtual void handleStartOperation(LifecycleOperation *operation) override;
    virtual void handleStopOperation(LifecycleOperation *operation) override;
    virtual void handleCrashOperation(LifecycleOperation *operation) override;

    // notification
    virtual void receiveSignal(cComponent *source, simsignal_t signalID, cObject *obj, cObject *details) override;

    simsignal_t hopCountSignal;
    simsignal_t routingFailedSignal;
    simsignal_t greedyForwardingFailedSignal;

    // Signal identifiers
    simsignal_t beaconSentBytesSignal;
    simsignal_t beaconSentCountSignal;

};


#endif // ifndef __INET_GREEDY_H

