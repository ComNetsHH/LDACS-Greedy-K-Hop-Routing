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

#include <algorithm>
#include <random>
#include <set>


#include "inet/common/INETUtils.h"
#include "inet/common/IProtocolRegistrationListener.h"
#include "inet/common/ModuleAccess.h"
#include "inet/common/ProtocolTag_m.h"
#include "inet/common/lifecycle/ModuleOperations.h"
#include "inet/linklayer/common/InterfaceTag_m.h"
#include "inet/networklayer/common/HopLimitTag_m.h"
#include "inet/networklayer/common/IpProtocolId_m.h"
#include "inet/networklayer/common/L3AddressTag_m.h"
#include "inet/networklayer/common/L3Tools.h"
#include "inet/networklayer/common/NextHopAddressTag_m.h"
#include "inet/networklayer/contract/IInterfaceTable.h"
#include "Greedy.h"
#include <string>
#include<fstream>



#ifdef WITH_IPv4
#include "inet/networklayer/ipv4/Ipv4Header_m.h"
#endif

#ifdef WITH_IPv6
#include "inet/networklayer/ipv6/Ipv6ExtensionHeaders_m.h"
#include "inet/networklayer/ipv6/Ipv6InterfaceData.h"
#endif

#ifdef WITH_NEXTHOP
#include "inet/networklayer/nexthop/NextHopForwardingHeader_m.h"
#endif

using namespace inet;

Define_Module(Greedy);

static inline double determinant(double a1, double a2, double b1, double b2)
{
    return a1 * b2 - a2 * b1;
}

Greedy::Greedy()
{
}

Greedy::~Greedy()
{
    cancelAndDelete(beaconTimer);
    cancelAndDelete(purgeNeighborsTimer);
}

//
// module interface
//

void Greedy::initialize(int stage)
{
    if (stage == INITSTAGE_ROUTING_PROTOCOLS)
        addressType = getSelfAddress().getAddressType();

    RoutingProtocolBase::initialize(stage);

    if (stage == INITSTAGE_LOCAL) {
        interfaces = par("interfaces");
        beaconInterval = par("beaconInterval");
        maxJitter = par("maxJitter");
        neighborValidityInterval = par("neighborValidityInterval");
        displayBubbles = par("displayBubbles");
        // context
        host = getContainingNode(this);
        interfaceTable = getModuleFromPar<IInterfaceTable>(par("interfaceTableModule"), this);
        outputInterface = par("outputInterface");
        mobility = check_and_cast<IMobility *>(host->getSubmodule("mobility"));
        routingTable = getModuleFromPar<IRoutingTable>(par("routingTableModule"), this);
        networkProtocol = getModuleFromPar<INetfilter>(par("networkProtocolModule"), this);
        // internal
        beaconTimer = new cMessage("BeaconTimer");
        purgeNeighborsTimer = new cMessage("PurgeNeighborsTimer");
        // packet size
        positionByteLength = par("positionByteLength");
        addressByteLength = par("addressByteLength");
        sequenceByteLength = par("sequenceByteLength");
        hopsByteLength = par("hopsByteLength");
        // Safely cast to size_t, assuming the value is non-negative
        // numberOfSelectedNeighbors = static_cast<size_t>(par("numberOfSelectedNeighbors").intValue());
        numberOfSelectedNeighbors = par("numberOfSelectedNeighbors");
        // KLUDGE: implement position registry protocol
        globalPositionTable.clearOneHopMap();
        hopCountSignal = registerSignal("hopCount");
        routingFailedSignal = registerSignal("routingFailed");
        greedyForwardingFailedSignal = registerSignal("greedyForwardingFailed");
        groundStationRange = m(par("groundStationRange"));
        GSx = par("GSx");
        GSy = par("GSy");
        GSz = par("GSz");
        const char *file_name = par("groundstationsTraceFile");
        parseGroundstationTraceFile2Vector(file_name);
        a2gOutputInterface = par("a2gOutputInterface");
        //new code added here
        int destination_index = findClosestGroundStation();
        beaconForwardedFromGpsr = par("beaconForwardedFromGpsr");
        useTwoHopGreedy = par("useTwoHopGreedy");
        useThreeHopGreedy = par("useThreeHopGreedy");
        useFFT = par("useFFT");
        useRandom = par("useRandom");
        useEFFT = par("useEFFT");
        useEnforceEFFT = par("useEnforceEFFT");
        forwardingRadiusReduction = par("forwardingRadiusReduction").doubleValue();
        beaconSentBytesSignal = registerSignal("beaconSentBytes");
        beaconSentCountSignal = registerSignal("beaconSentCount");
    }
    else if (stage == INITSTAGE_ROUTING_PROTOCOLS) {
        registerService(Protocol::manet, nullptr, gate("ipIn"));
        registerProtocol(Protocol::manet, gate("ipOut"), nullptr);
        host->subscribe(linkBrokenSignal, this);
        networkProtocol->registerHook(0, this);
        WATCH(neighborPositionTable);
        // Obtain the communication range from the radio module
        cModule *radioModule = getModuleFromPar<cModule>(par("radioModule"), this);
        radio = check_and_cast<inet::physicallayer::IRadio *>(radioModule);
        commRange = radio->getTransmitter()->getMaxCommunicationRange().get();
        // thresholdDistance = 1 / 2 * commRange;
        thresholdDistance = 1 / std::sqrt(2) * commRange; //half the length of a square's side inscribed in a unit circle
        // thresholdDistance = std::sqrt(3) / 2 * commRange; // half the length of a cube side inscribed in a unit sphere.
        // thresholdDistance = 3 / (std::sqrt(2) * 2) * commRange;
    }
}

void Greedy::handleMessageWhenUp(cMessage *message)
{
    if (message->isSelfMessage())
        processSelfMessage(message);
    else
        processMessage(message);
}

//
// handling messages
//

void Greedy::processSelfMessage(cMessage *message)
{
    if (message == beaconTimer)
        processBeaconTimer();
    else if (message == purgeNeighborsTimer)
        processPurgeNeighborsTimer();
    else
        throw cRuntimeError("Unknown self message");
}

void Greedy::processMessage(cMessage *message)
{
    if (auto pk = dynamic_cast<Packet *>(message))
        processUdpPacket(pk);
    else
        throw cRuntimeError("Unknown message");
}

//The following function reads a trace file to store the coordinates of the ground stations
void Greedy::parseGroundstationTraceFile2Vector(const char* file_name)
{
    // std::vector<double> groundstationCoordinate;
    std::ifstream in(file_name, std::ios::in);
    // Check if the file is opened (we modified the error message here to just  return in order to enable scripting the application)
    if (in.fail()){
         throw std::invalid_argument( "file does not exist" );
    }
    std::string lineStr;
    std::string ethernetInterface;
    double x;
    double y;
    double z;
    std::istringstream iss;
    // Read the file line by line until the end.
    while (std::getline(in, lineStr))
    {
        std::istringstream iss(lineStr);
        iss >> x >> y >> z >> ethernetInterface;
        if (!iss) {
            EV_ERROR << "Failed to parse line: " << lineStr << std::endl;
            continue; // Skip this line and move to the next
        }
        //std::cout << "x=" << x << ",y=" << y << ",z=" << z << ",interface=" << ethernetInterface << std::endl;
        // insert the x, y, z coordinates of one groundstation into groundstationCoordinate vector
        // groundstationCoordinate.insert(groundstationCoordinate.end(), { x,y,z });
        // ground_stations_coordinates_array.insert(ground_stations_coordinates_array.end(), { groundstationCoordinate });
        ground_stations_coordinates_array.push_back({x,y,z });
        // clear the vector that contains a row of the grounstation trace file
        // groundstationCoordinate.clear();
        // insert the groundstationCoordinate vector into the vector of vectors groundstationCoordinates
        ethernet_vector.insert(ethernet_vector.end(), { ethernetInterface });
    }
    int num_of_GS = ground_stations_coordinates_array.size();
    for (int i = 0; i < num_of_GS; i++){
        EV_INFO << "Ground station " << i << " location: " << ground_stations_coordinates_array.at(i).at(0) << ", " << ground_stations_coordinates_array.at(i).at(1) << ", " << ground_stations_coordinates_array.at(i).at(2) << endl;
    }
    // auto array_test = ground_stations_coordinates_array;
    // auto array_test1 = ground_stations_coordinates_array[0];
    // auto array_test2= ground_stations_coordinates_array[1];
    // Close The File
    in.close();
}

////The following function calculates distance to all ground stations and finds the closest one
int Greedy::findClosestGroundStation()
{
    int closest_ground_station=0;
    //getting the position of the current aircraft
    Coord aircraft_position = check_and_cast<IMobility *>(getContainingNode(this)->getSubmodule("mobility"))->getCurrentPosition();
    EV << " aircraft_position: " << aircraft_position << " \n";
    
    std::vector<double> distance_vector;
    //applying the formula sqrt((x_2-x_1)^2+(y_2-y_1)^2+(z_2-z1)^2) to get the distance between aircraft and all ground stations
    double min_distance = sqrt(pow((aircraft_position.x-ground_stations_coordinates_array.at(0).at(0)),2) + pow((aircraft_position.y-ground_stations_coordinates_array.at(0).at(1)),2)+ pow((aircraft_position.z-ground_stations_coordinates_array.at(0).at(2)),2));
    int num_of_GS = ground_stations_coordinates_array.size();
    for(int i = 0; i < num_of_GS; i++){
        double distance = sqrt(pow((aircraft_position.x-ground_stations_coordinates_array.at(i).at(0)),2) + pow((aircraft_position.y-ground_stations_coordinates_array.at(i).at(1)),2)+ pow((aircraft_position.z-ground_stations_coordinates_array.at(i).at(2)),2));
        distance_vector.push_back(distance);

        //find the closest ground station
        if (distance < min_distance){
            min_distance = distance;
            closest_ground_station=i;
        }
    }
    EV << " closest_ground_station is No.: " << closest_ground_station << " \n";
    EV << " Corresponding ground station location (" 
   << ground_stations_coordinates_array.at(closest_ground_station).at(0) << "," 
   << ground_stations_coordinates_array.at(closest_ground_station).at(1) << "," 
   << ground_stations_coordinates_array.at(closest_ground_station).at(2) << ") \n";
    EV << " Corresponding Ethernet is: " << ethernet_vector[closest_ground_station] << " \n";
    //new code upto here
    return closest_ground_station;
}


//
// beacon timers
//

void Greedy::scheduleBeaconTimer()
{
    EV_DEBUG << "Scheduling beacon timer" << endl;
    // scheduleAt(simTime() + beaconInterval + uniform(-1, 1) * maxJitter, beaconTimer);
    scheduleAt(simTime() + beaconInterval, beaconTimer);
}

void Greedy::processBeaconTimer()
{
    EV_DEBUG << "Processing beacon timer" << endl;
    const L3Address selfAddress = getSelfAddress();
    if (!selfAddress.isUnspecified()) {
        if(beaconForwardedFromGpsr) {
            sendBeacon(createBeacon());
        }
        // sendBeacon(createBeacon());
        storeSelfPositionInGlobalRegistry();
    }
    scheduleBeaconTimer();
    schedulePurgeNeighborsTimer();
}

//
// handling purge neighbors timers
//

void Greedy::schedulePurgeNeighborsTimer() 
{
    EV_DEBUG << "Scheduling purge neighbors timer" << endl;
    simtime_t nextOneHopExpiration = getNextOneHopNeighborExpiration();
    simtime_t nextTwoHopExpiration = getNextTwoHopNeighborExpiration();
    simtime_t nextThreeHopExpiration = getNextThreeHopNeighborExpiration();
    simtime_t nextFourHopExpiration = getNextFourHopNeighborExpiration();
    
    simtime_t nextExpiration = std::min(
        std::min(nextOneHopExpiration, nextTwoHopExpiration),
        std::min(nextThreeHopExpiration, nextFourHopExpiration)
    );


    if (nextExpiration == SimTime::getMaxTime()) {
        if (purgeNeighborsTimer->isScheduled())
            cancelEvent(purgeNeighborsTimer);
    }
    else {
        if (!purgeNeighborsTimer->isScheduled())
            scheduleAt(nextExpiration, purgeNeighborsTimer);
        else {
            if (purgeNeighborsTimer->getArrivalTime() != nextExpiration) {
                cancelEvent(purgeNeighborsTimer);
                scheduleAt(nextExpiration, purgeNeighborsTimer);
            }
        }
    }
}

void Greedy::processPurgeNeighborsTimer()
{
    EV_DEBUG << "Processing purge neighbors timer" << endl;
    purgeNeighbors();
    schedulePurgeNeighborsTimer();
}

//
// handling UDP packets
//

void Greedy::sendUdpPacket(Packet *packet)
{
    send(packet, "ipOut");
}

void Greedy::processUdpPacket(Packet *packet)
{
    packet->popAtFront<UdpHeader>();
    processBeacon(packet);
    schedulePurgeNeighborsTimer();
}

//
// handling beacons
//

// Start by selecting random node and the rest m-1 nodes with the farthest first traversal with the using of threshold
L3AddressVector Greedy::farthestFirstTraversal(L3AddressVector& NeighborAddresses, size_t m) {
    L3Address selfAddress = getSelfAddress();
    Coord selfPosition = mobility->getCurrentPosition();
    L3AddressVector selectedAddresses;
    CoordVector selectedPositions;
    std::set<L3Address> selectedAddressesSet; // Efficient lookup for already selected addresses

    if (NeighborAddresses.empty()) {
        // Optionally, handle the situation or log a message
        return L3AddressVector(); // Return an empty vector
    }

    // If the number of neighbor addresses is less than or equal to m, return them directly.
    if (NeighborAddresses.size() <= m) {
        return NeighborAddresses; // Directly return all neighbor addresses
    }

    size_t numAddressesToSelect = m;  // Since we checked size <= m, we can use m directly.

    std::random_device rd;
    std::mt19937 g(rd());

    // Generate a random index
    size_t randomIndex = std::uniform_int_distribution<size_t>(0, NeighborAddresses.size() - 1)(g);
    L3Address firstNeighborAddress = NeighborAddresses[randomIndex];
    Coord firstNeighborPosition = neighborPositionTable.getNeighborPosition(firstNeighborAddress);

    // Add the randomly selected address
    selectedAddresses.push_back(firstNeighborAddress);
    selectedPositions.push_back(firstNeighborPosition);
    selectedAddressesSet.insert(firstNeighborAddress); // Also add to set for quick lookup

    // Select up to m-1 additional farthest addresses
    while (selectedAddresses.size() < numAddressesToSelect) {
        double farthestDistance = 0.0;
        L3Address farthestAddress;

        // Check each neighbor address to find the farthest one
        for (const auto& address : NeighborAddresses) {
            // Skip if already selected or is self
            if (selectedAddressesSet.count(address) > 0 || address == selfAddress) {
                continue;
            }

            Coord neighborPosition = neighborPositionTable.getNeighborPosition(address);
            double minDistance = std::numeric_limits<double>::max();

            // Find minimum distance to already selected addresses
            for (const Coord& selectedPosition : selectedPositions) {
                double distance = neighborPosition.distance(selectedPosition);
                minDistance = std::min(minDistance, distance);
            }

            if (minDistance > farthestDistance) {
                farthestDistance = minDistance;
                farthestAddress = address;
            }       
        }

        // Add the farthest address found to the selected list
        if (!farthestAddress.isUnspecified()) {
            selectedAddresses.push_back(farthestAddress);
            selectedPositions.push_back(neighborPositionTable.getNeighborPosition(farthestAddress));
            selectedAddressesSet.insert(farthestAddress); // Update the set
        } else {
            // Stop if no further farthest addresses are found
            break;  
        }
    }
    
    return selectedAddresses;
}

// Start by selecting random node and the rest m-1 nodes with the farthest first traversal with the using of threshold
L3AddressVector Greedy::enhancedFarthestFirstTraversal(L3AddressVector& NeighborAddresses, int hops, size_t m) {
    L3Address selfAddress = getSelfAddress();
    Coord selfPosition = mobility->getCurrentPosition();
    L3AddressVector selectedAddresses;
    CoordVector selectedPositions;
    std::set<L3Address> selectedAddressesSet; // Efficient lookup for already selected addresses

    if (NeighborAddresses.empty()) {
        // Optionally, handle the situation or log a message
        return L3AddressVector(); // Return an empty vector
    }

    size_t numAddressesToSelect = std::min(m, NeighborAddresses.size());

    std::random_device rd;
    std::mt19937 g(rd());

    // Generate a random index
    size_t randomIndex = std::uniform_int_distribution<size_t>(0, NeighborAddresses.size() - 1)(g);
    L3Address firstNeighborAddress = NeighborAddresses[randomIndex];
    Coord firstNeighborPosition = neighborPositionTable.getNeighborPosition(firstNeighborAddress);
    
    // Remove the selected address from the original vector
    NeighborAddresses.erase(NeighborAddresses.begin() + randomIndex);
    // numAddressesToSelect--;

    // Add the randomly selected address
    selectedAddresses.push_back(firstNeighborAddress);
    selectedPositions.push_back(firstNeighborPosition);
    selectedAddressesSet.insert(firstNeighborAddress); // Also add to set for quick lookup

    // Select up to m-1 additional farthest addresses
    while (selectedAddresses.size() < numAddressesToSelect) {
        double farthestDistance = 0.0;
        L3Address farthestAddress;

        // Check each neighbor address to find the farthest one
        for (const auto& address : NeighborAddresses) {
            // Skip if already selected or is self
            if (selectedAddressesSet.count(address) > 0 || address == selfAddress) {
                continue;
            }
            Coord neighborPosition = neighborPositionTable.getNeighborPosition(address);
            double minDistance = std::numeric_limits<double>::max();
            double distanceToSelf = neighborPosition.distance(selfPosition);

            // Find minimum distance to already selected addresses
            for (const Coord& selectedPosition : selectedPositions) {
                double distance = neighborPosition.distance(selectedPosition);
                minDistance = std::min(minDistance, distance);
            }

            if (minDistance > farthestDistance && minDistance > hops * thresholdDistance && distanceToSelf > hops * (1 / std::sqrt(2)) * thresholdDistance) {
                farthestDistance = minDistance;
                farthestAddress = address;
            }

        }
        // Add the farthest address found to the selected list
        if (!farthestAddress.isUnspecified()) {
            selectedAddresses.push_back(farthestAddress);
            selectedPositions.push_back(neighborPositionTable.getNeighborPosition(farthestAddress));
            selectedAddressesSet.insert(farthestAddress); // Update the set

        } else {
            break;  // Stop if no further farthest addresses are found
        }
    }

    return selectedAddresses;
}

L3AddressVector Greedy::farthestFirstTraversalWithPreSelection(const L3AddressVector& NeighborAddresses, const CoordVector& preSelectedPositions, int hops, size_t m) {
    L3Address selfAddress = getSelfAddress();
    Coord selfPosition = mobility->getCurrentPosition();
    L3AddressVector selectedAddresses;
    CoordVector selectedPositions;
    std::set<L3Address> selectedAddressesSet; // Efficient lookup for already selected addresses

    if (NeighborAddresses.empty()) {
        // Optionally, handle the situation or log a message
        return L3AddressVector(); // Return an empty vector
    }

    size_t numAddressesToSelect = std::min(m, NeighborAddresses.size());

    // Select up to m-1 additional farthest addresses
    while (selectedAddresses.size() < numAddressesToSelect) {
        double farthestDistance = 0.0;
        L3Address farthestAddress;

        // Check each neighbor address to find the farthest one
        for (const auto& address : NeighborAddresses) {
            // Skip if already selected or is self
            if (selectedAddressesSet.count(address) > 0 || address == selfAddress) {
                continue;
            }
            Coord neighborPosition = neighborPositionTable.getNeighborPosition(address);
            double minDistance = std::numeric_limits<double>::max();
            double distanceToSelf = neighborPosition.distance(selfPosition);

            // Find minimum distance to already selected addresses
            for (const Coord& selectedPosition : selectedPositions) {
                double distance = neighborPosition.distance(selectedPosition);
                minDistance = std::min(minDistance, distance);
            }
            // Additional step: Check distance to pre-selected positions
            for (const Coord& preSelectedPosition : preSelectedPositions) {
                double distanceToPreSelected = neighborPosition.distance(preSelectedPosition);
                minDistance = std::min(minDistance, distanceToPreSelected);
            }

            if (minDistance > farthestDistance && minDistance > thresholdDistance) {
                farthestDistance = minDistance;
                farthestAddress = address;
            }
        }
        // Add the farthest address found to the selected list
        if (!farthestAddress.isUnspecified()) {
            selectedAddresses.push_back(farthestAddress);
            selectedPositions.push_back(neighborPositionTable.getNeighborPosition(farthestAddress));
            selectedAddressesSet.insert(farthestAddress); // Update the set

        } else {
            break;  // Stop if no further farthest addresses are found
        }
    }

    return selectedAddresses;
}

L3AddressVector Greedy::farthestFirstTraversalWithoutThreshold(const L3AddressVector& NeighborAddresses, const CoordVector& preSelectedPositions, size_t m) {
    L3Address selfAddress = getSelfAddress();
    Coord selfPosition = mobility->getCurrentPosition();
    L3AddressVector selectedAddresses;
    CoordVector selectedPositions;
    std::set<L3Address> selectedAddressesSet; // Efficient lookup for already selected addresses

    if (NeighborAddresses.empty()) {
        // Optionally, handle the situation or log a message
        return L3AddressVector(); // Return an empty vector
    }

    size_t numAddressesToSelect = std::min(m, NeighborAddresses.size());

    // Select up to m-1 additional farthest addresses
    while (selectedAddresses.size() < numAddressesToSelect) {
        double farthestDistance = 0.0;
        L3Address farthestAddress;

        // Check each neighbor address to find the farthest one
        for (const auto& address : NeighborAddresses) {
            // Skip if already selected or is self
            if (selectedAddressesSet.count(address) > 0 || address == selfAddress) {
                continue;
            }
            Coord neighborPosition = neighborPositionTable.getNeighborPosition(address);
            double minDistance = std::numeric_limits<double>::max();
            double distanceToSelf = neighborPosition.distance(selfPosition);

            // Find minimum distance to already selected addresses
            for (const Coord& selectedPosition : selectedPositions) {
                double distance = neighborPosition.distance(selectedPosition);
                minDistance = std::min(minDistance, distance);
            }
            // Additional step: Check distance to pre-selected positions
            for (const Coord& preSelectedPosition : preSelectedPositions) {
                double distanceToPreSelected = neighborPosition.distance(preSelectedPosition);
                minDistance = std::min(minDistance, distanceToPreSelected);
            }

            if (minDistance > farthestDistance && selectedAddressesSet.count(selfAddress) == 0) {
                farthestDistance = minDistance;
                farthestAddress = address;
            }
        }
        // Add the farthest address found to the selected list
        if (!farthestAddress.isUnspecified()) {
            selectedAddresses.push_back(farthestAddress);
            selectedPositions.push_back(neighborPositionTable.getNeighborPosition(farthestAddress));
            selectedAddressesSet.insert(farthestAddress); // Update the set

            // Remove selfAddress after adding the second node
            if (selectedAddresses.size() == 2) {
                auto it = std::find(selectedAddresses.begin(), selectedAddresses.end(), selfAddress);
                if (it != selectedAddresses.end()) {
                    // Remove from vector
                    selectedPositions.erase(selectedPositions.begin() + (it - selectedAddresses.begin()));
                    selectedAddresses.erase(it);
                }
                // Remove from set
                selectedAddressesSet.erase(selfAddress);
            }
        } else {
            break;  // Stop if no further farthest addresses are found
        }
    }

    return selectedAddresses;
}

void Greedy::kHopNeighborInfoToBeaconFFT(const Ptr<GreedyBeacon>& beacon, size_t m) {
    L3AddressVector oneHopNeighborAddresses = neighborPositionTable.getOneHopNeighborAddresses();
    CoordVector oneHopNeighborPositions;

    CoordVector selectedOneHopNeighborPositions;
    EV_INFO << "Do we go inside FFT greedy k-hop function" << endl;

    // Now call the function
    L3AddressVector selectedOneHopNeighborAddresses = farthestFirstTraversal(oneHopNeighborAddresses, m);

    for (const L3Address& address : selectedOneHopNeighborAddresses) {
        Coord position = neighborPositionTable.getOneHopNeighborPosition(address);
        selectedOneHopNeighborPositions.push_back(position);
    }
    // Assign the vectors to the beacon
    beacon->setOneHopNeighborAddresses(selectedOneHopNeighborAddresses);
    beacon->setOneHopNeighborPositions(selectedOneHopNeighborPositions);
}

void Greedy::kHopNeighborInfoToBeaconEFFT(const Ptr<GreedyBeacon>& beacon, size_t m) {
    // Get 1-hop and 2-hop neighbor addresses and positions
    L3AddressVector oneHopNeighborAddresses = neighborPositionTable.getOneHopNeighborAddresses();
    L3AddressVector twoHopNeighborAddresses = neighborPositionTable.getTwoHopNeighborAddresses();
    L3AddressVector selectedOneHopNeighborAddresses;
    L3AddressVector selectedTwoHopNeighborAddresses;
    CoordVector selectedOneHopNeighborPositions;
    CoordVector selectedTwoHopNeighborPositions;
    
    EV_INFO << "Do we go inside EFFT greedy k-hop function" << endl;

    // Select up to m neighbors from 1-hop neighbors
    selectedOneHopNeighborAddresses = enhancedFarthestFirstTraversal(oneHopNeighborAddresses, 1, (m > 4 ? 4 : m));

    for (const L3Address& address : selectedOneHopNeighborAddresses) {
        Coord position = neighborPositionTable.getOneHopNeighborPosition(address);
        selectedOneHopNeighborPositions.push_back(position);
    }

    // If fewer than m neighbors were selected, select additional neighbors from 2-hop neighbors
    size_t selectedOneHopCount = selectedOneHopNeighborAddresses.size();
    size_t remaining_one = m - selectedOneHopCount;
    if (remaining_one > 0) {
        EV_INFO << "Still we can select " << remaining_one << " neighbors from 2-hop neighborhood." << endl;
        // selectedTwoHopNeighborAddresses = farthestFirstTraversalWithoutThreshold(twoHopNeighborAddresses, selectedOneHopNeighborPositions, remaining);
        selectedTwoHopNeighborAddresses = farthestFirstTraversalWithPreSelection(twoHopNeighborAddresses, selectedOneHopNeighborPositions, 1, remaining_one);
        // Append selected 2-hop neighbors' addresses and positions
        for (const L3Address& address : selectedTwoHopNeighborAddresses) {
            Coord position = neighborPositionTable.getTwoHopNeighborPosition(address);
            selectedTwoHopNeighborPositions.push_back(position);
        }
    }

    // If fewer than m neighbors were selected, select additional neighbors from 3-hop neighbors
    size_t selectedTwoHopCount = selectedTwoHopNeighborAddresses.size();
    size_t remaining_two = m - selectedOneHopCount - selectedTwoHopCount;
    if (remaining_two > 0) {
        EV_INFO << "Select remaining " << remaining_two << " neighbors from 1-hop neighborhood." << endl;
        // Combining the two vectors
        CoordVector combinedSelectedPositions;
        // Insert the elements of the first vector
        combinedSelectedPositions.insert(combinedSelectedPositions.end(), 
                                selectedOneHopNeighborPositions.begin(), 
                                selectedOneHopNeighborPositions.end());
        // Insert the elements of the second vector
        combinedSelectedPositions.insert(combinedSelectedPositions.end(), 
                                selectedTwoHopNeighborPositions.begin(), 
                                selectedTwoHopNeighborPositions.end());
        L3AddressVector selectedOneHopNeighborAddresses_1 = farthestFirstTraversalWithPreSelection(oneHopNeighborAddresses, combinedSelectedPositions, 1, remaining_two);

        // Append selected 1-hop neighbors' addresses and positions
        for (const L3Address& address : selectedOneHopNeighborAddresses_1) {
            Coord position = neighborPositionTable.getOneHopNeighborPosition(address);
            selectedOneHopNeighborPositions.push_back(position);
        }

        selectedOneHopNeighborAddresses.insert(selectedOneHopNeighborAddresses.end(), 
                                selectedOneHopNeighborAddresses_1.begin(), 
                                selectedOneHopNeighborAddresses_1.end()); 
    }
    
    // If 1-hop neighbors were added, assign them too
    if (!selectedOneHopNeighborAddresses.empty()) {
        // Assign the vectors to the beacon
        beacon->setOneHopNeighborAddresses(selectedOneHopNeighborAddresses);
        beacon->setOneHopNeighborPositions(selectedOneHopNeighborPositions);
    }

    // If 2-hop neighbors were added, assign them too
    if (!selectedTwoHopNeighborAddresses.empty()) {
        // Assign the vectors to the beacon
        beacon->setTwoHopNeighborAddresses(selectedTwoHopNeighborAddresses);
        beacon->setTwoHopNeighborPositions(selectedTwoHopNeighborPositions);
        EV_INFO << "Great, we selected additionally " << selectedTwoHopCount << " neighbors from 2-hop neighborhood." << endl;
    }

}

void Greedy::kHopNeighborInfoToBeaconEnforceEFFT(const Ptr<GreedyBeacon>& beacon, size_t m) {
    // Get 1-hop and 2-hop neighbor addresses and positions
    L3AddressVector oneHopNeighborAddresses = neighborPositionTable.getOneHopNeighborAddresses();
    L3AddressVector twoHopNeighborAddresses = neighborPositionTable.getTwoHopNeighborAddresses();
    L3AddressVector threeHopNeighborAddresses = neighborPositionTable.getThreeHopNeighborAddresses();
    L3AddressVector selectedOneHopNeighborAddresses;
    L3AddressVector selectedTwoHopNeighborAddresses;
    L3AddressVector selectedThreeHopNeighborAddresses;
    CoordVector selectedOneHopNeighborPositions;
    CoordVector selectedTwoHopNeighborPositions;
    CoordVector selectedThreeHopNeighborPositions;

    EV_INFO << "Do we go inside FFT greedy k-hop function" << endl;
    EV_INFO << "m= " << m << endl;
    // Now call the function
    // selectedOneHopNeighborAddresses = farthestFirstTraversal(oneHopNeighborAddresses, 1, m/2);
    selectedOneHopNeighborAddresses = enhancedFarthestFirstTraversal(oneHopNeighborAddresses, 1, 4);
    for (const L3Address& address : selectedOneHopNeighborAddresses) {
        Coord position = neighborPositionTable.getOneHopNeighborPosition(address);
        selectedOneHopNeighborPositions.push_back(position);
    }

    selectedTwoHopNeighborAddresses = farthestFirstTraversalWithPreSelection(twoHopNeighborAddresses, selectedOneHopNeighborPositions, 1, m-4);
    for (const L3Address& address : selectedTwoHopNeighborAddresses) {
        Coord position = neighborPositionTable.getTwoHopNeighborPosition(address);
        selectedTwoHopNeighborPositions.push_back(position);
    }
    
    // If fewer than m neighbors were selected, select additional neighbors from 2-hop neighbors
    size_t selectedOneHopCount = selectedOneHopNeighborAddresses.size();
    size_t selectedTwoHopCount = selectedTwoHopNeighborAddresses.size();
    size_t remaining = m - selectedOneHopCount -selectedTwoHopCount;

    EV_INFO << "Selected 1-hop nodes: " << selectedOneHopCount <<" nodes, Selected 2-hop nodes: " << selectedTwoHopCount << " nodes, Still we can select " << remaining << " nodes from 3-hop neighborhood." << endl;

    if (remaining > 0) {
        // Combining the two vectors
        CoordVector combinedSelectedPositions;
        // Insert the elements of the first vector
        combinedSelectedPositions.insert(combinedSelectedPositions.end(), 
                                selectedOneHopNeighborPositions.begin(), 
                                selectedOneHopNeighborPositions.end());
        // Insert the elements of the second vector
        combinedSelectedPositions.insert(combinedSelectedPositions.end(), 
                                selectedTwoHopNeighborPositions.begin(), 
                                selectedTwoHopNeighborPositions.end());
        selectedThreeHopNeighborAddresses = farthestFirstTraversalWithPreSelection(threeHopNeighborAddresses, combinedSelectedPositions, 1, remaining);
        // selectedThreeHopNeighborAddresses = farthestFirstTraversal(threeHopNeighborAddresses, 1, remaining);

        // Append selected 3-hop neighbors' addresses and positions
        for (const L3Address& address : selectedThreeHopNeighborAddresses) {
            Coord position = neighborPositionTable.getThreeHopNeighborPosition(address);
            selectedThreeHopNeighborPositions.push_back(position);
        }
    }

    size_t selectedThreeHopCount = selectedThreeHopNeighborAddresses.size();
    size_t remaining_one = m - selectedOneHopCount - selectedTwoHopCount - selectedThreeHopCount;
    
    selectedOneHopCount = selectedOneHopNeighborAddresses.size();
    selectedTwoHopCount = selectedTwoHopNeighborAddresses.size();
    selectedThreeHopCount = selectedThreeHopNeighborAddresses.size();
    size_t remaining_two = m - selectedOneHopCount - selectedTwoHopCount - selectedThreeHopCount;
    L3AddressVector selectedTwoHopNeighborAddresses_1;
    if (remaining_two > 0) { 
        CoordVector combinedSelectedPositions;
        // Insert the elements of the first vector
        combinedSelectedPositions.insert(combinedSelectedPositions.end(), 
                                selectedOneHopNeighborPositions.begin(), 
                                selectedOneHopNeighborPositions.end());
        // Insert the elements of the second vector
        combinedSelectedPositions.insert(combinedSelectedPositions.end(), 
                                selectedTwoHopNeighborPositions.begin(), 
                                selectedTwoHopNeighborPositions.end());
        // Insert the elements of the third vector
        combinedSelectedPositions.insert(combinedSelectedPositions.end(), 
                                selectedThreeHopNeighborPositions.begin(), 
                                selectedThreeHopNeighborPositions.end());
            selectedTwoHopNeighborAddresses_1 = farthestFirstTraversalWithoutThreshold(twoHopNeighborAddresses, combinedSelectedPositions, remaining_two);
        // Append selected 1-hop neighbors' addresses and positions
        for (const L3Address& address : selectedTwoHopNeighborAddresses_1) {
            Coord position = neighborPositionTable.getTwoHopNeighborPosition(address);
            selectedTwoHopNeighborPositions.push_back(position);
        }
    }
    selectedTwoHopNeighborAddresses.insert(selectedTwoHopNeighborAddresses.end(), 
                                selectedTwoHopNeighborAddresses_1.begin(), 
                                selectedTwoHopNeighborAddresses_1.end());
    
    // If fewer than m neighbors were selected, select additional neighbors from 2-hop neighbors
    selectedOneHopCount = selectedOneHopNeighborAddresses.size();
    selectedTwoHopCount = selectedTwoHopNeighborAddresses.size();
    selectedThreeHopCount = selectedThreeHopNeighborAddresses.size();
    EV_INFO << "Finished Selection: 1-hop nodes: " << selectedOneHopCount << ", 2-hop nodes: " << selectedTwoHopCount << ", 3-hop nodes: " << selectedThreeHopCount << endl;
    // Assign the vectors to the beacon
    beacon->setOneHopNeighborAddresses(selectedOneHopNeighborAddresses);
    beacon->setOneHopNeighborPositions(selectedOneHopNeighborPositions);

    // Assign the vectors to the beacon
    beacon->setTwoHopNeighborAddresses(selectedTwoHopNeighborAddresses);
    beacon->setTwoHopNeighborPositions(selectedTwoHopNeighborPositions);

    beacon->setThreeHopNeighborAddresses(selectedThreeHopNeighborAddresses);
    beacon->setThreeHopNeighborPositions(selectedThreeHopNeighborPositions);

}

L3AddressVector Greedy::selectRandomNeighbors(L3AddressVector& neighborAddresses, size_t m) {
    L3AddressVector selectedNeighborAddresses;
    if (neighborAddresses.empty() || m == 0) {
        return selectedNeighborAddresses; // Return empty if conditions not met
    }

    std::random_device rd;
    std::mt19937 g(rd());

    while (selectedNeighborAddresses.size() < m && !neighborAddresses.empty()) {
        // Generate a random index
        size_t randomIndex = std::uniform_int_distribution<size_t>(0, neighborAddresses.size() - 1)(g);
        // Add the randomly selected address
        selectedNeighborAddresses.push_back(neighborAddresses[randomIndex]);
        // Remove the selected address from the original vector
        neighborAddresses.erase(neighborAddresses.begin() + randomIndex);
    }

    return selectedNeighborAddresses;
}

void Greedy::kHopNeighborInfoToBeaconRandom(const Ptr<GreedyBeacon>& beacon, size_t m) {
    L3AddressVector oneHopNeighborAddresses = neighborPositionTable.getOneHopNeighborAddresses();
    L3AddressVector twoHopNeighborAddresses = neighborPositionTable.getTwoHopNeighborAddresses();
    CoordVector oneHopNeighborPositions;
    CoordVector twoHopNeighborPositions;

    CoordVector selectedOneHopNeighborPositions;
    EV_INFO << "Do we go inside random greedy k-hop function" << endl;

    L3AddressVector selectedOneHopNeighborAddresses = selectRandomNeighbors(oneHopNeighborAddresses, m);

    for (const L3Address& address : selectedOneHopNeighborAddresses) {
        Coord position = neighborPositionTable.getOneHopNeighborPosition(address);
        selectedOneHopNeighborPositions.push_back(position);
    }
    // Assign the vectors to the beacon
    beacon->setOneHopNeighborAddresses(selectedOneHopNeighborAddresses);
    beacon->setOneHopNeighborPositions(selectedOneHopNeighborPositions);

    // Add 3-hop neighbors using Random
    if (useThreeHopGreedy) {
        CoordVector selectedTwoHopNeighborPositions;
        L3AddressVector selectedTwoHopNeighborAddresses = selectRandomNeighbors(twoHopNeighborAddresses, m);
        
        for (const L3Address& address : selectedTwoHopNeighborAddresses) {
            Coord position = neighborPositionTable.getTwoHopNeighborPosition(address);
            selectedTwoHopNeighborPositions.push_back(position);
        }
        // Assign the vectors to the beacon
        beacon->setTwoHopNeighborAddresses(selectedTwoHopNeighborAddresses);
        beacon->setTwoHopNeighborPositions(selectedTwoHopNeighborPositions);

    }    
}

void Greedy::kHopNeighborInfoToBeacon(const Ptr<GreedyBeacon>& beacon) {
    L3AddressVector oneHopNeighborAddresses = neighborPositionTable.getOneHopNeighborAddresses();
    L3AddressVector twoHopNeighborAddresses = neighborPositionTable.getTwoHopNeighborAddresses();
    CoordVector oneHopNeighborPositions;
    CoordVector twoHopNeighborPositions;

    EV_INFO << "Do we go inside normal greedy k-hop function" << endl;

    for (const L3Address& address : oneHopNeighborAddresses) {
        Coord position = neighborPositionTable.getOneHopNeighborPosition(address);
        oneHopNeighborPositions.push_back(position);
    }
    // Assign the vectors to the beacon
    beacon->setOneHopNeighborAddresses(oneHopNeighborAddresses);
    beacon->setOneHopNeighborPositions(oneHopNeighborPositions);

    // Add 3-hop neighbors 
    if (useThreeHopGreedy) {
        for (const L3Address& address : twoHopNeighborAddresses) {
            Coord position = neighborPositionTable.getTwoHopNeighborPosition(address);
            twoHopNeighborPositions.push_back(position);
        }
        // Assign the vectors to the beacon
        beacon->setTwoHopNeighborAddresses(twoHopNeighborAddresses);
        beacon->setTwoHopNeighborPositions(twoHopNeighborPositions);

    }    
}

const Ptr<GreedyBeacon> Greedy::createBeacon()
{
    const auto& beacon = makeShared<GreedyBeacon>();
    beacon->setAddress(getSelfAddress());
    beacon->setPosition(mobility->getCurrentPosition());
    if (useEFFT) {
        // Add neighbors to the beacon using Enhanced FFT selection method
        if (useEnforceEFFT) {
            kHopNeighborInfoToBeaconEnforceEFFT(beacon, numberOfSelectedNeighbors);
        }
        
        else {
            kHopNeighborInfoToBeaconEFFT(beacon, numberOfSelectedNeighbors);
        }
        // kHopNeighborInfoToBeaconEFFT(beacon, numberOfSelectedNeighbors);
    }
    else if (useTwoHopGreedy || useThreeHopGreedy) {
        // Apply FFT algorithm
        if (useFFT) {
            // Add the k-hop neighbors to the beacon using FFT selection method
            kHopNeighborInfoToBeaconFFT(beacon, numberOfSelectedNeighbors);
        }

        // Apply random selection
        else if (useRandom) {
            // Add a subset of the k-hop neighbors to the beacon using Random selection method
            kHopNeighborInfoToBeaconRandom(beacon, numberOfSelectedNeighbors);  
        }
        // Regular two-hop greedy approach
        else {
            // Add a subset of the k-hop neighbors to the beacon
            kHopNeighborInfoToBeacon(beacon);
        }
    }
    
    // Calculate and set the chunk length
    // int addressSize = getSelfAddress().getAddressType()->getAddressByteLength();
    int numOneHopNeighbors = beacon->getOneHopNeighborAddresses().size();
    int numTwoHopNeighbors = beacon->getTwoHopNeighborAddresses().size();
    int numThreeHopNeighbors = beacon->getThreeHopNeighborAddresses().size();
    int chunkLength = addressByteLength + positionByteLength + sequenceByteLength
                      + numOneHopNeighbors * (addressByteLength + positionByteLength + sequenceByteLength + hopsByteLength)
                      + numTwoHopNeighbors * (addressByteLength + positionByteLength + sequenceByteLength + hopsByteLength)
                      + numThreeHopNeighbors * (addressByteLength + positionByteLength + sequenceByteLength + hopsByteLength);
    beacon->setChunkLength(B(chunkLength));
    return beacon;
}

void Greedy::sendBeacon(const Ptr<GreedyBeacon>& beacon)
{
    EV_INFO << "Sending beacon: address = " << beacon->getAddress() << ", position = " << beacon->getPosition() << endl;
    int beaconLengthBytes = B(beacon->getChunkLength()).get(); // Gets the length of the beacon in bytes
    Packet *udpPacket = new Packet("GreedyBeacon");
    udpPacket->insertAtBack(beacon);
    auto udpHeader = makeShared<UdpHeader>();
    udpHeader->setSourcePort(GREEDY_UDP_PORT);
    udpHeader->setDestinationPort(GREEDY_UDP_PORT);
    udpHeader->setCrcMode(CRC_DISABLED);
    udpPacket->insertAtFront(udpHeader);
    auto addresses = udpPacket->addTag<L3AddressReq>();
    addresses->setSrcAddress(getSelfAddress());
    addresses->setDestAddress(addressType->getLinkLocalManetRoutersMulticastAddress());
    udpPacket->addTag<HopLimitReq>()->setHopLimit(255);
    udpPacket->addTag<PacketProtocolTag>()->setProtocol(&Protocol::manet);
    udpPacket->addTag<DispatchProtocolReq>()->setProtocol(addressType->getNetworkProtocol());
    sendUdpPacket(udpPacket);
    emit(beaconSentBytesSignal, beaconLengthBytes);
    emit(beaconSentCountSignal, 1);
}

void Greedy::processBeacon(Packet *packet)
{
    const auto& beacon = packet->peekAtFront<GreedyBeacon>();
    EV_INFO << "Processing beacon: address = " << beacon->getAddress() << ", position = " << beacon->getPosition() << endl;
    // neighborPositionTable.setOneHopNeighborPosition(beacon->getAddress(), beacon->getPosition(), beacon->getAddress());
    neighborPositionTable.setPositionBasedOnHops(beacon->getAddress(), beacon->getPosition(), beacon->getAddress(), 1);

    // Get the viaAddress (address of the beacon sender)
    L3Address viaAddress = beacon->getAddress();

    // Step 1: Retrieve addresses from the position table with viaAddress
    L3AddressVector twoHopNeighborsFromTableViaSender = neighborPositionTable.getTwoHopNeighborsWithViaAddress(viaAddress);
    L3AddressVector threeHopNeighborsFromTableViaSender = neighborPositionTable.getThreeHopNeighborsWithViaAddress(viaAddress);
    L3AddressVector fourHopNeighborsFromTableViaSender = neighborPositionTable.getFourHopNeighborsWithViaAddress(viaAddress);

    // Step 1: Remove these addresses from the position table if they are not in the current beacon
    // Removing from two-hop neighbors
    for (const auto& addr : twoHopNeighborsFromTableViaSender) {
        if (std::find(beacon->getOneHopNeighborAddresses().begin(), beacon->getOneHopNeighborAddresses().end(), addr) == beacon->getOneHopNeighborAddresses().end()) {
            neighborPositionTable.removeTwoHopNeighborPosition(addr);
        }
    }

    // Removing from three-hop neighbors
    for (const auto& addr : threeHopNeighborsFromTableViaSender) {
        if (std::find(beacon->getTwoHopNeighborAddresses().begin(), beacon->getTwoHopNeighborAddresses().end(), addr) == beacon->getTwoHopNeighborAddresses().end()) {
            neighborPositionTable.removeThreeHopNeighborPosition(addr);
        }
    }

    // Removing from four-hop neighbors
    for (const auto& addr : fourHopNeighborsFromTableViaSender) {
        if (std::find(beacon->getThreeHopNeighborAddresses().begin(), beacon->getThreeHopNeighborAddresses().end(), addr) == beacon->getThreeHopNeighborAddresses().end()) {
            neighborPositionTable.removeFourHopNeighborPosition(addr);
        }
    }

    // Step 3: Update position table with new neighbor information from the beacon
    // one hop neighbors in the beacon are two hop neighbors for this node
    const auto& twoHopNeighborAddresses = beacon->getOneHopNeighborAddresses();
    const auto& twoHopNeighborPositions = beacon->getOneHopNeighborPositions();
    for (size_t i = 0; i < twoHopNeighborAddresses.size(); ++i) {
        // Do not add if the address is the node's own address
        if (twoHopNeighborAddresses[i] != getSelfAddress()) {
            // Set position for each two-hop neighbor in the table
            // neighborPositionTable.setTwoHopNeighborPosition(twoHopNeighborAddresses[i], twoHopNeighborPositions[i], beacon->getAddress());
            neighborPositionTable.setPositionBasedOnHops(twoHopNeighborAddresses[i], twoHopNeighborPositions[i], beacon->getAddress(), 2);

            // Retrieve and print the position to verify it's set correctly
            Coord retrievedPosition = neighborPositionTable.getTwoHopNeighborPosition(twoHopNeighborAddresses[i]);
            // Check if the position for this neighbor is available
            if (neighborPositionTable.hasTwoHopNeighborPosition(twoHopNeighborAddresses[i])) {
                EV_INFO << "Two-hop Neighbor " 
                        << ": address = " << twoHopNeighborAddresses[i] 
                        << ", position = " << retrievedPosition << endl;
            }
                
            
            
        }
    }

    // two hop neighbors in the beacon are three hop neighbors for this node
    const auto& threeHopNeighborAddresses = beacon->getTwoHopNeighborAddresses();
    const auto& threeHopNeighborPositions = beacon->getTwoHopNeighborPositions();
    for (size_t i = 0; i < threeHopNeighborAddresses.size(); ++i) {
        // Do not add if the address is the node's own address
        if (threeHopNeighborAddresses[i] != getSelfAddress()) {
            // Set position for each three-hop neighbor in the table
            // neighborPositionTable.setThreeHopNeighborPosition(threeHopNeighborAddresses[i], threeHopNeighborPositions[i], beacon->getAddress());
            neighborPositionTable.setPositionBasedOnHops(threeHopNeighborAddresses[i], threeHopNeighborPositions[i], beacon->getAddress(), 3);

            // Retrieve and print the position to verify it's set correctly
            Coord retrievedPosition = neighborPositionTable.getThreeHopNeighborPosition(threeHopNeighborAddresses[i]);
            // Check if the position for this neighbor is available
            if (neighborPositionTable.hasThreeHopNeighborPosition(threeHopNeighborAddresses[i])) {
                EV_INFO << "Three-hop Neighbor " 
                        << ": address = " << threeHopNeighborAddresses[i] 
                        << ", position = " << retrievedPosition << endl;
            }
                
            
            
        }
    }

    // three hop neighbors in the beacon are four hop neighbors for this node
    const auto& fourHopNeighborAddresses = beacon->getThreeHopNeighborAddresses();
    const auto& fourHopNeighborPositions = beacon->getThreeHopNeighborPositions();
    for (size_t i = 0; i < fourHopNeighborAddresses.size(); ++i) {
        // Do not add if the address is the node's own address
        if (fourHopNeighborAddresses[i] != getSelfAddress()) {
            // Set position for each four-hop neighbor in the table
            // neighborPositionTable.setFourHopNeighborPosition(fourHopNeighborAddresses[i], fourHopNeighborPositions[i], beacon->getAddress());
            neighborPositionTable.setPositionBasedOnHops(fourHopNeighborAddresses[i], fourHopNeighborPositions[i], beacon->getAddress(), 4);

            // Retrieve and print the position to verify it's set correctly
            Coord retrievedPosition = neighborPositionTable.getFourHopNeighborPosition(fourHopNeighborAddresses[i]);
            // Check if the position for this neighbor is available
            if (neighborPositionTable.hasFourHopNeighborPosition(fourHopNeighborAddresses[i])) {
                EV_INFO << "Four-hop Neighbor " 
                        << ": address = " << fourHopNeighborAddresses[i] 
                        << ", position = " << retrievedPosition << endl;
            }
        }
    }
    
    delete packet;
}

//
// handling packets
//

GreedyOption *Greedy::createGreedyOption(L3Address destination)
{
    GreedyOption *greedyOption = new GreedyOption();
    greedyOption->setRoutingMode(GPSR_GREEDY_ROUTING);
    int destination_index = findClosestGroundStation();
    // const Coord GroundStationLocation = Coord(GSx, GSy, GSz);
    const Coord GroundStationLocation = Coord(ground_stations_coordinates_array[destination_index][0], ground_stations_coordinates_array[destination_index][1], ground_stations_coordinates_array[destination_index][2]);
    EV_INFO << "Ground station (Destination) position = " << GroundStationLocation << endl;
    greedyOption->setDestinationPosition(GroundStationLocation);
    greedyOption->setLength(computeOptionLength(greedyOption));
//    greedyOption->setHopCount(0);
    return greedyOption;
}

int Greedy::computeOptionLength(GreedyOption *option)
{
    int positionsBytes = positionByteLength;
    int addressesBytes = addressByteLength;
    // type and length
    int tlBytes = 1 + 1;

    // return tlBytes + routingModeBytes + positionsBytes + addressesBytes;
    return tlBytes + positionsBytes + addressesBytes;
}

//
// configuration
//

void Greedy::configureInterfaces()
{
    // join multicast groups
    cPatternMatcher interfaceMatcher(interfaces, false, true, false);
    for (int i = 0; i < interfaceTable->getNumInterfaces(); i++) {
        InterfaceEntry *interfaceEntry = interfaceTable->getInterface(i);
        if (interfaceEntry->isMulticast() && interfaceMatcher.matches(interfaceEntry->getInterfaceName()))
            interfaceEntry->joinMulticastGroup(addressType->getLinkLocalManetRoutersMulticastAddress());
    }
}

//
// position
//

// KLUDGE: implement position registry protocol
PositionTableModified Greedy::globalPositionTable;

Coord Greedy::lookupPositionInGlobalRegistry(const L3Address& address) const
{
    // KLUDGE: implement position registry protocol
    return globalPositionTable.getOneHopNeighborPosition(address);
}

void Greedy::storePositionInGlobalRegistry(const L3Address& address, const Coord& position) const
{
    // KLUDGE: implement position registry protocol
    globalPositionTable.setOneHopNeighborPosition(address, position, address);
}

void Greedy::storeSelfPositionInGlobalRegistry() const
{
    auto selfAddress = getSelfAddress();
    if (!selfAddress.isUnspecified())
        storePositionInGlobalRegistry(selfAddress, mobility->getCurrentPosition());
}

Coord Greedy::getNeighborPosition(const L3Address& address) const
{
    return neighborPositionTable.getOneHopNeighborPosition(address);
}

//
// address
//

std::string Greedy::getHostName() const
{
    return host->getFullName();
}

L3Address Greedy::getSelfAddress() const
{
    //TODO choose self address based on a new 'interfaces' parameter
    L3Address ret = routingTable->getRouterIdAsGeneric();
#ifdef WITH_IPv6
    if (ret.getType() == L3Address::IPv6) {
        for (int i = 0; i < interfaceTable->getNumInterfaces(); i++) {
            InterfaceEntry *ie = interfaceTable->getInterface(i);
            if ((!ie->isLoopback())) {
                if (auto ipv6Data = ie->findProtocolData<Ipv6InterfaceData>()) {
                    ret = ipv6Data->getPreferredAddress();
                    break;
                }
            }
        }
    }
#endif
    return ret;
}

L3Address Greedy::getSenderNeighborAddress(const Ptr<const NetworkHeaderBase>& networkHeader) const
{
    const GreedyOption *greedyOption = getGreedyOptionFromNetworkDatagram(networkHeader);
    return greedyOption->getSenderAddress();
}

//
// neighbor
//

// Get next expiration time for 1-hop neighbors
simtime_t Greedy::getNextOneHopNeighborExpiration() 
{
    simtime_t oldestPosition = neighborPositionTable.getOldestOneHopPosition();
    if (oldestPosition == SimTime::getMaxTime())
        return oldestPosition;
    else
        return oldestPosition + neighborValidityInterval;
}

// Get next expiration time for 2-hop neighbors
simtime_t Greedy::getNextTwoHopNeighborExpiration() 
{
    simtime_t oldestPosition = neighborPositionTable.getOldestTwoHopPosition();
    if (oldestPosition == SimTime::getMaxTime())
        return oldestPosition;
    else
        return oldestPosition + neighborValidityInterval;
}

// Get next expiration time for 3-hop neighbors
simtime_t Greedy::getNextThreeHopNeighborExpiration() 
{
    simtime_t oldestPosition = neighborPositionTable.getOldestThreeHopPosition();
    if (oldestPosition == SimTime::getMaxTime())
        return oldestPosition;
    else
        return oldestPosition + neighborValidityInterval;
}

// Get next expiration time for 4-hop neighbors
simtime_t Greedy::getNextFourHopNeighborExpiration() 
{
    simtime_t oldestPosition = neighborPositionTable.getOldestFourHopPosition();
    if (oldestPosition == SimTime::getMaxTime())
        return oldestPosition;
    else
        return oldestPosition + neighborValidityInterval;
}

// Purge old 1-hop neighbors and 2-hop neighbors
void Greedy::purgeNeighbors() {
    // Purge old 1-hop neighbors
    neighborPositionTable.removeOldOneHopNeighborPositions(simTime() - neighborValidityInterval);

    // Purge old 2-hop neighbors
    neighborPositionTable.removeOldTwoHopNeighborPositions(simTime() - neighborValidityInterval);

    // Purge old 3-hop neighbors
    neighborPositionTable.removeOldThreeHopNeighborPositions(simTime() - neighborValidityInterval);

    // Purge old 4-hop neighbors
    neighborPositionTable.removeOldFourHopNeighborPositions(simTime() - neighborValidityInterval);
}

//
// next hop
//

L3Address Greedy::findNextHop(const L3Address& destination, GreedyOption *greedyOption)
{
    m distanceToGroundStation = m(mobility->getCurrentPosition().distance(greedyOption->getDestinationPosition()));
    EV_INFO << "The distance to the ground station = " << km(distanceToGroundStation) << endl;
    if (distanceToGroundStation <= groundStationRange) {
        EV_INFO << "The ground station is within communication range" << endl;
        return destination; // The next hop is the destination (the ground station)
    }
    switch (greedyOption->getRoutingMode()) {
        case GPSR_GREEDY_ROUTING: return findGreedyRoutingNextHop(destination, greedyOption);
        default: throw cRuntimeError("Unknown routing mode");
    }
}

L3Address Greedy::findGreedyRoutingNextHop(const L3Address& destination, GreedyOption *greedyOption)
{
    EV_DEBUG << "Finding next hop using greedy routing: destination = " << destination << endl;
    L3Address selfAddress = getSelfAddress();
    Coord selfPosition = mobility->getCurrentPosition();
    Coord destinationPosition = greedyOption->getDestinationPosition();
    double bestDistance = destinationPosition.distance(selfPosition); // distance of the closest neighbor to the destination
    L3Address bestNeighbor;
    L3AddressVector oneHopNeighborAddresses = neighborPositionTable.getOneHopNeighborAddresses();
    EV_INFO << "Print addresses in the beacon: 1-hop: ";
    for (const auto& addr : neighborPositionTable.getOneHopNeighborAddresses()) {
        EV_INFO << addr << " ";
    }
    EV_INFO << ", 2-hop: ";
    for (const auto& addr : neighborPositionTable.getTwoHopNeighborAddresses()) {
        EV_INFO << addr << " ";
    }
    EV_INFO << ", 3-hop: ";
    for (const auto& addr : neighborPositionTable.getThreeHopNeighborAddresses()) {
        EV_INFO << addr << " ";
    }
    EV_INFO << ", 4-hop: ";
    for (const auto& addr : neighborPositionTable.getFourHopNeighborAddresses()) {
        EV_INFO << addr << " ";
    }
    EV_INFO << endl;

    for (auto& neighborAddress: oneHopNeighborAddresses) {
        Coord neighborPosition = neighborPositionTable.getOneHopNeighborPosition(neighborAddress);
        double neighborDistance = destinationPosition.distance(neighborPosition); // distance of the current neighbor to the destination
        double oneHopNeighborDistance = selfPosition.distance(neighborPosition);
        if (neighborDistance < bestDistance && oneHopNeighborDistance <  commRange - forwardingRadiusReduction) {
            EV_INFO << "We found someone in 1-hop: " << neighborAddress << endl;
            bestDistance = neighborDistance;
            bestNeighbor = neighborAddress;
        }
    }

    L3AddressVector twoHopNeighborAddresses = neighborPositionTable.getTwoHopNeighborAddresses();
    for (auto& neighborAddress : twoHopNeighborAddresses) {
        Coord neighborPosition = neighborPositionTable.getTwoHopNeighborPosition(neighborAddress);
        double neighborDistance = destinationPosition.distance(neighborPosition); // distance of the current neighbor to the destination
        if (neighborDistance < bestDistance) {
            EV_INFO << "We found someone in 2-hop: " << neighborAddress << endl;
            auto oneHopNeighbor = neighborPositionTable.getTwoHopNeighborViaAddress(neighborAddress);
            Coord oneHopNeighborPosition = neighborPositionTable.getOneHopNeighborPosition(oneHopNeighbor);
            double oneHopNeighborDistance = selfPosition.distance(oneHopNeighborPosition);
            if (oneHopNeighborDistance < commRange - forwardingRadiusReduction) {
                bestDistance = neighborDistance;
                bestNeighbor = oneHopNeighbor;

            }
        }
    }
        
    L3AddressVector threeHopNeighborAddresses = neighborPositionTable.getThreeHopNeighborAddresses();
    for (auto& neighborAddress : threeHopNeighborAddresses) {
        Coord neighborPosition = neighborPositionTable.getThreeHopNeighborPosition(neighborAddress);
        double neighborDistance = destinationPosition.distance(neighborPosition); // distance of the current neighbor to the destination
        if (neighborDistance < bestDistance) {
            EV_INFO << "We found someone in 3-hop: " << neighborAddress << endl;
            auto oneHopNeighbor = neighborPositionTable.getTwoHopNeighborViaAddress(neighborAddress);
            Coord oneHopNeighborPosition = neighborPositionTable.getOneHopNeighborPosition(oneHopNeighbor);
            double oneHopNeighborDistance = selfPosition.distance(oneHopNeighborPosition);
            if (oneHopNeighborDistance < commRange - forwardingRadiusReduction) {
                bestDistance = neighborDistance;
                bestNeighbor = oneHopNeighbor;

            }
        }
    }

    L3AddressVector fourHopNeighborAddresses = neighborPositionTable.getFourHopNeighborAddresses();
    for (auto& neighborAddress : fourHopNeighborAddresses) {
        Coord neighborPosition = neighborPositionTable.getFourHopNeighborPosition(neighborAddress);
        double neighborDistance = destinationPosition.distance(neighborPosition); // distance of the current neighbor to the destination
        if (neighborDistance < bestDistance) {
            EV_INFO << "We found someone in 4-hop: " << neighborAddress << endl;
            auto oneHopNeighbor = neighborPositionTable.getTwoHopNeighborViaAddress(neighborAddress);
            Coord oneHopNeighborPosition = neighborPositionTable.getOneHopNeighborPosition(oneHopNeighbor);
            double oneHopNeighborDistance = selfPosition.distance(oneHopNeighborPosition);
            if (oneHopNeighborDistance < commRange - forwardingRadiusReduction) {
                bestDistance = neighborDistance;
                bestNeighbor = oneHopNeighbor;

            }
        }
    }
       
    if (bestNeighbor.isUnspecified()) {
        for (auto& neighborAddress: oneHopNeighborAddresses) {
            Coord neighborPosition = neighborPositionTable.getOneHopNeighborPosition(neighborAddress);
            double neighborDistance = destinationPosition.distance(neighborPosition); // distance of the current neighbor to the destination
            if (neighborDistance < bestDistance) {
                EV_INFO << "We found someone in 1-hop: " << neighborAddress << endl;
                bestDistance = neighborDistance;
                bestNeighbor = neighborAddress;
            }
        }

        // Switch to 2-hop greedy forwarding 
        L3AddressVector twoHopNeighborAddresses = neighborPositionTable.getTwoHopNeighborAddresses();
        for (auto& neighborAddress : twoHopNeighborAddresses) {
            Coord neighborPosition = neighborPositionTable.getTwoHopNeighborPosition(neighborAddress);
            double neighborDistance = destinationPosition.distance(neighborPosition); // distance of the current neighbor to the destination
            if (neighborDistance < bestDistance) {
                EV_INFO << "We found someone in 2-hop: " << neighborAddress << endl;
                bestDistance = neighborDistance;
                bestNeighbor = neighborPositionTable.getTwoHopNeighborViaAddress(neighborAddress);
            }
        }

        // Try 3-hop greedy forwarding 
        L3AddressVector threeHopNeighborAddresses = neighborPositionTable.getThreeHopNeighborAddresses();
        for (auto& neighborAddress : threeHopNeighborAddresses) {
            Coord neighborPosition = neighborPositionTable.getThreeHopNeighborPosition(neighborAddress);
            double neighborDistance = destinationPosition.distance(neighborPosition); // distance of the current neighbor to the destination
            if (neighborDistance < bestDistance) {
                EV_INFO << "We found someone in 3-hop: " << neighborAddress << endl;
                bestDistance = neighborDistance;
                bestNeighbor = neighborPositionTable.getThreeHopNeighborViaAddress(neighborAddress);
            }
        }

        // Try 4-hop greedy forwarding 
        L3AddressVector fourHopNeighborAddresses = neighborPositionTable.getFourHopNeighborAddresses();
        for (auto& neighborAddress : fourHopNeighborAddresses) {
            Coord neighborPosition = neighborPositionTable.getFourHopNeighborPosition(neighborAddress);
            double neighborDistance = destinationPosition.distance(neighborPosition); // distance of the current neighbor to the destination
            if (neighborDistance < bestDistance) {
                EV_INFO << "We found someone in 4-hop: " << neighborAddress << endl;
                bestDistance = neighborDistance;
                bestNeighbor = neighborPositionTable.getFourHopNeighborViaAddress(neighborAddress);
            }
        }

        if (bestNeighbor.isUnspecified()) {
            emit(greedyForwardingFailedSignal, simTime());
        }
    }

    return bestNeighbor;
}

//
// routing
//

INetfilter::IHook::Result Greedy::routeDatagram(Packet *datagram, GreedyOption *greedyOption)
{
    const auto& ipv4Header = datagram->peekAtFront<Ipv4Header>();
    const auto& networkHeader = getNetworkProtocolHeader(datagram);
    const L3Address& source = networkHeader->getSourceAddress();
    const L3Address& destination = networkHeader->getDestinationAddress();

    EV_INFO << "Finding next hop: source = " << source << ", destination = " << destination << endl;
    auto nextHop = findNextHop(destination, greedyOption);
    datagram->addTagIfAbsent<NextHopAddressReq>()->setNextHopAddress(nextHop);
    if (nextHop.isUnspecified()) {
        EV_WARN << "No next hop found, dropping packet: source = " << source << ", destination = " << destination << endl;
        emit(routingFailedSignal, simTime());
        if (displayBubbles && hasGUI())
            getContainingNode(host)->bubble("No next hop found, dropping packet");
        return DROP;
    }
    else {
        EV_INFO << "Next hop found: source = " << source << ", destination = " << destination << ", nextHop: " << nextHop << endl;
        // EV_INFO << "getSelfAddress() = " << getSelfAddress() << endl;
        // EV_INFO << "greedyOption->getDestinationIndex() = " << greedyOption->getDestinationIndex() << endl;
        greedyOption->setSenderAddress(getSelfAddress());
        // auto destination_index = greedyOption->getDestinationIndex();
        auto interfaceEntry = CHK(interfaceTable->findInterfaceByName(outputInterface));

        m distanceToGroundStation = m(mobility->getCurrentPosition().distance(greedyOption->getDestinationPosition()));
        if (distanceToGroundStation <= groundStationRange) {
            emit(hopCountSignal, 32 - (ipv4Header->getTimeToLive()) + 1);
            EV_INFO << "Hop count for application packet = " << 32 - (ipv4Header->getTimeToLive()) + 1 << endl;
            int destination_index = findClosestGroundStation();
            auto a2gInterfaceEntry = CHK(interfaceTable->findInterfaceByName(ethernet_vector[destination_index].c_str()));
            EV_INFO << "OutputInterface = " << ethernet_vector[destination_index].c_str() << " Interface ID = " << a2gInterfaceEntry << endl;
            datagram->addTagIfAbsent<InterfaceReq>()->setInterfaceId(a2gInterfaceEntry->getInterfaceId());
            return ACCEPT;
        }
        datagram->addTagIfAbsent<InterfaceReq>()->setInterfaceId(interfaceEntry->getInterfaceId());
        return ACCEPT;
    }
}

void Greedy::setGreedyOptionOnNetworkDatagram(Packet *packet, const Ptr<const NetworkHeaderBase>& networkHeader, GreedyOption *greedyOption)
{
    packet->trimFront();
#ifdef WITH_IPv4
    if (dynamicPtrCast<const Ipv4Header>(networkHeader)) {
        auto ipv4Header = removeNetworkProtocolHeader<Ipv4Header>(packet);
        greedyOption->setType(IPOPTION_TLV_GPSR);
        B oldHlen = ipv4Header->calculateHeaderByteLength();
        ASSERT(ipv4Header->getHeaderLength() == oldHlen);
        ipv4Header->addOption(greedyOption);
        B newHlen = ipv4Header->calculateHeaderByteLength();
        ipv4Header->setHeaderLength(newHlen);
        ipv4Header->addChunkLength(newHlen - oldHlen);
        ipv4Header->setTotalLengthField(ipv4Header->getTotalLengthField() + newHlen - oldHlen);
        insertNetworkProtocolHeader(packet, Protocol::ipv4, ipv4Header);
    }
    else
#endif
#ifdef WITH_IPv6
    if (dynamicPtrCast<const Ipv6Header>(networkHeader)) {
        auto ipv6Header = removeNetworkProtocolHeader<Ipv6Header>(packet);
        greedyOption->setType(IPv6TLVOPTION_TLV_GPSR);
        B oldHlen = ipv6Header->calculateHeaderByteLength();
        Ipv6HopByHopOptionsHeader *hdr = check_and_cast_nullable<Ipv6HopByHopOptionsHeader *>(ipv6Header->findExtensionHeaderByTypeForUpdate(IP_PROT_IPv6EXT_HOP));
        if (hdr == nullptr) {
            hdr = new Ipv6HopByHopOptionsHeader();
            hdr->setByteLength(B(8));
            ipv6Header->addExtensionHeader(hdr);
        }
        hdr->getTlvOptionsForUpdate().insertTlvOption(greedyOption);
        hdr->setByteLength(B(utils::roundUp(2 + B(hdr->getTlvOptions().getLength()).get(), 8)));
        B newHlen = ipv6Header->calculateHeaderByteLength();
        ipv6Header->addChunkLength(newHlen - oldHlen);
        insertNetworkProtocolHeader(packet, Protocol::ipv6, ipv6Header);
    }
    else
#endif
#ifdef WITH_NEXTHOP
    if (dynamicPtrCast<const NextHopForwardingHeader>(networkHeader)) {
        auto nextHopHeader = removeNetworkProtocolHeader<NextHopForwardingHeader>(packet);
        greedyOption->setType(NEXTHOP_TLVOPTION_TLV_GPSR);
        int oldHlen = nextHopHeader->getTlvOptions().getLength();
        nextHopHeader->getTlvOptionsForUpdate().insertTlvOption(greedyOption);
        int newHlen = nextHopHeader->getTlvOptions().getLength();
        nextHopHeader->addChunkLength(B(newHlen - oldHlen));
        insertNetworkProtocolHeader(packet, Protocol::nextHopForwarding, nextHopHeader);
    }
    else
#endif
    {
    }
}

const GreedyOption *Greedy::findGreedyOptionInNetworkDatagram(const Ptr<const NetworkHeaderBase>& networkHeader) const
{
    const GreedyOption *greedyOption = nullptr;

#ifdef WITH_IPv4
    if (auto ipv4Header = dynamicPtrCast<const Ipv4Header>(networkHeader)) {
        greedyOption = check_and_cast_nullable<const GreedyOption *>(ipv4Header->findOptionByType(IPOPTION_TLV_GPSR));
    }
    else
#endif
#ifdef WITH_IPv6
    if (auto ipv6Header = dynamicPtrCast<const Ipv6Header>(networkHeader)) {
        const Ipv6HopByHopOptionsHeader *hdr = check_and_cast_nullable<const Ipv6HopByHopOptionsHeader *>(ipv6Header->findExtensionHeaderByType(IP_PROT_IPv6EXT_HOP));
        if (hdr != nullptr) {
            int i = (hdr->getTlvOptions().findByType(IPv6TLVOPTION_TLV_GPSR));
            if (i >= 0)
                greedyOption = check_and_cast<const GreedyOption *>(hdr->getTlvOptions().getTlvOption(i));
        }
    }
    else
#endif
#ifdef WITH_NEXTHOP
    if (auto nextHopHeader = dynamicPtrCast<const NextHopForwardingHeader>(networkHeader)) {
        int i = (nextHopHeader->getTlvOptions().findByType(NEXTHOP_TLVOPTION_TLV_GPSR));
        if (i >= 0)
            greedyOption = check_and_cast<const GreedyOption *>(nextHopHeader->getTlvOptions().getTlvOption(i));
    }
    else
#endif
    {
    }
    return greedyOption;
}

GreedyOption *Greedy::findGreedyOptionInNetworkDatagramForUpdate(const Ptr<NetworkHeaderBase>& networkHeader)
{
    GreedyOption *greedyOption = nullptr;

#ifdef WITH_IPv4
    if (auto ipv4Header = dynamicPtrCast<Ipv4Header>(networkHeader)) {
        greedyOption = check_and_cast_nullable<GreedyOption *>(ipv4Header->findMutableOptionByType(IPOPTION_TLV_GPSR));
    }
    else
#endif
#ifdef WITH_IPv6
    if (auto ipv6Header = dynamicPtrCast<Ipv6Header>(networkHeader)) {
        Ipv6HopByHopOptionsHeader *hdr = check_and_cast_nullable<Ipv6HopByHopOptionsHeader *>(ipv6Header->findExtensionHeaderByTypeForUpdate(IP_PROT_IPv6EXT_HOP));
        if (hdr != nullptr) {
            int i = (hdr->getTlvOptions().findByType(IPv6TLVOPTION_TLV_GPSR));
            if (i >= 0)
                greedyOption = check_and_cast<GreedyOption *>(hdr->getTlvOptionsForUpdate().getTlvOptionForUpdate(i));
        }
    }
    else
#endif
#ifdef WITH_NEXTHOP
    if (auto nextHopHeader = dynamicPtrCast<NextHopForwardingHeader>(networkHeader)) {
        int i = (nextHopHeader->getTlvOptions().findByType(NEXTHOP_TLVOPTION_TLV_GPSR));
        if (i >= 0)
            greedyOption = check_and_cast<GreedyOption *>(nextHopHeader->getTlvOptionsForUpdate().getTlvOptionForUpdate(i));
    }
    else
#endif
    {
    }
    return greedyOption;
}

const GreedyOption *Greedy::getGreedyOptionFromNetworkDatagram(const Ptr<const NetworkHeaderBase>& networkHeader) const
{
    const GreedyOption *greedyOption = findGreedyOptionInNetworkDatagram(networkHeader);
    if (greedyOption == nullptr)
        throw cRuntimeError("Gpsr option not found in datagram!");
    return greedyOption;
}

GreedyOption *Greedy::getGreedyOptionFromNetworkDatagramForUpdate(const Ptr<NetworkHeaderBase>& networkHeader)
{
    GreedyOption *greedyOption = findGreedyOptionInNetworkDatagramForUpdate(networkHeader);
    if (greedyOption == nullptr)
        throw cRuntimeError("Gpsr option not found in datagram!");
    return greedyOption;
}

//
// netfilter
//

INetfilter::IHook::Result Greedy::datagramPreRoutingHook(Packet *datagram)
{
    Enter_Method("datagramPreRoutingHook");
    const auto& networkHeader = getNetworkProtocolHeader(datagram);
    const L3Address& destination = networkHeader->getDestinationAddress();
    if (destination.isMulticast() || destination.isBroadcast() || routingTable->isLocalAddress(destination))
        return ACCEPT;
    else {
        // KLUDGE: this allows overwriting the GPSR option inside
        auto greedyOption = const_cast<GreedyOption *>(getGreedyOptionFromNetworkDatagram(networkHeader));
        return routeDatagram(datagram, greedyOption);
    }
}

INetfilter::IHook::Result Greedy::datagramLocalOutHook(Packet *packet)
{
    Enter_Method("datagramLocalOutHook");

    const auto& networkHeader = getNetworkProtocolHeader(packet);
    const L3Address& destination = networkHeader->getDestinationAddress();
    if (destination.isMulticast() || destination.isBroadcast() || routingTable->isLocalAddress(destination))
        return ACCEPT;
    else {
        GreedyOption *greedyOption = createGreedyOption(networkHeader->getDestinationAddress());
        setGreedyOptionOnNetworkDatagram(packet, networkHeader, greedyOption);
        return routeDatagram(packet, greedyOption);
    }
}

//
// This allows emitting the hop count signal for application packets only 
//

INetfilter::IHook::Result Greedy::datagramLocalInHook(Packet *packet)
{
    Enter_Method("datagramLocalInHook");
    const auto& ipv4Header = packet->peekAtFront<Ipv4Header>();
    const auto& networkHeader = getNetworkProtocolHeader(packet);
    const L3Address& destination = networkHeader->getDestinationAddress();

    const GreedyOption *greedyOption = findGreedyOptionInNetworkDatagram(networkHeader);
    return ACCEPT;
}

//
// lifecycle
//

void Greedy::handleStartOperation(LifecycleOperation *operation)
{
    configureInterfaces();
    // scheduleBeaconTimer();
    scheduleAt(simTime() + beaconInterval * uniform(0, 1), beaconTimer);
    return storeSelfPositionInGlobalRegistry();
}

void Greedy::handleStopOperation(LifecycleOperation *operation)
{
    // TODO: send a beacon to remove ourself from peers neighbor position table
    neighborPositionTable.clearOneHopMap();
    neighborPositionTable.clearTwoHopMap();
    neighborPositionTable.clearThreeHopMap();
    neighborPositionTable.clearFourHopMap();
    cancelEvent(beaconTimer);
    cancelEvent(purgeNeighborsTimer);
}

void Greedy::handleCrashOperation(LifecycleOperation *operation)
{
    neighborPositionTable.clearOneHopMap();
    neighborPositionTable.clearTwoHopMap();
    neighborPositionTable.clearThreeHopMap();
    neighborPositionTable.clearFourHopMap();
    cancelEvent(beaconTimer);
    cancelEvent(purgeNeighborsTimer);
}

//
// notification
//

void Greedy::receiveSignal(cComponent *source, simsignal_t signalID, cObject *obj, cObject *details)
{
    Enter_Method("receiveChangeNotification");
    if (signalID == linkBrokenSignal) {
        EV_WARN << "Received link break" << endl;
        // TODO: remove the neighbor
    }
}

void Greedy::handleMessageWhenDown(cMessage *message) {
    return;
}




