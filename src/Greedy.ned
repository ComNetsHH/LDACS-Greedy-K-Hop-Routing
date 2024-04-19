// The LDACS Greedy K-Hop Routing implements the greedy routing algorithm with a subset of k-hop neighborhood.
// Copyright (C) 2024  Musab Ahmed, Konrad Fuger, Koojana Kuladinithi, Andreas Timm-Giel, Institute of Communication Networks, Hamburg University of Technology, Hamburg, Germany
//
// This file is derived from work originally developed by Levente Meszaros.
// Original Source: https://github.com/inet-framework/inet/tree/master/src/inet/routing/gpsr
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

package greedy_forwarding;

import inet.routing.contract.IManetRouting;

//
// This module provides Greedy Perimeter Stateless Routing for Wireless Networks.
// The implementation supports both GG and RNG planarization algorithms.
//
// For more information on the routing algorithm, see the GPSR paper
// http://www.eecs.harvard.edu/~~htk/publication/2000-mobi-karp-kung.pdf
//
simple Greedy like IManetRouting
{
    parameters:
        // properties
        @display("i=block/routing");

        // Multi-Link parameters
        bool useMultiLink = default(false);
        bool useIntelligentMultiLinkMode = default(false);
        double multiLinkCutoffDistance @unit(m) = default(0m);

        double pSatcom = default(0);

        // context parameters
        string interfaceTableModule;   // The path to the InterfaceTable module
        string routingTableModule = default("^.ipv4.routingTable");
        string networkProtocolModule = default("^.ipv4.ip");
        string outputInterface = default("wlan0");
        string a2gOutputInterface = default("eth0");
        bool beaconForwardedFromGpsr = default(false);
        bool useTwoHopGreedy = default(false);
        bool useThreeHopGreedy = default(false);
        int numberOfSelectedNeighbors = default(4); 
        bool useFFT = default(false);
        bool useRandom = default(false);
        bool useEFFT = default(false);
        bool useEnforceEFFT = default(false);
        string radioModule = default("^.wlan[*].radio");   // The path to the Radio module 
        double forwardingRadiusReduction @unit(m) = default(2500m); 

        // Greedy Forwarding parameters
        string planarizationMode @enum("", "GG", "RNG") = default("GG");
        // if this is set to other than wlan0 it causes error Fix assert due to the links e.g., ethernet
        string interfaces = default("wlan0");
        double beaconInterval @unit(s) = default(10s);
        double maxJitter @unit(s) = default(0.5 * beaconInterval);
        double neighborValidityInterval @unit(s) = default(2 * beaconInterval);
        int positionByteLength @unit(B) = default(6B); // using Compact Position Reporting (CPR) algorithm
        int addressByteLength @unit(B) = default(4B); // using MAC ID
        int sequenceByteLength @unit(B) = default(1B); // for sequence numbers used
        int hopsByteLength @unit(B) = default(1B); // for number of hops
        double groundStationRange @unit(m) = default(370400m);   // the range where A2G cimmunication is possible
		double GSx @unit(m) = default(4734222.285m);
        double GSy @unit(m) = default(1381949.583m);
        double GSz @unit(m) = default(662813.2938m);
        string groundstationsTraceFile =default("GS_coordinates.txt"); 
        double weightingFactor = default(0.5); 
        int congestionLevel = default(1); 
        int congestionLevelByteLength @unit(B) = default(1 * 1B); 
        bool enableCrossLayerRouting = default(false);
        // visualization parameters
        bool displayBubbles = default(false);	// display bubble messages about changes in routing state for packets

        @signal[hopCount](type=long);
        @statistic[hopCount](source=hopCount; record=vector);
        @signal[routingFailed](type=simtime_t);
        @statistic[routingFailed](source=routingFailed; record=vector,histogram,count);
        @signal[greedyForwardingFailed](type=simtime_t);
        @statistic[greedyForwardingFailed](source=greedyForwardingFailed; record=vector,histogram,count);
        @signal[beaconSentBytes](type="long");
        @signal[beaconSentCount](type="long");
        @statistic[beaconSentBytes](title="Total Beacon Bytes Sent"; source=beaconSentBytes; record=sum);
        @statistic[beaconSentCount](title="Total Beacons Sent"; source=beaconSentCount; record=count);
        
    gates:
        input ipIn;
        output ipOut;
}

