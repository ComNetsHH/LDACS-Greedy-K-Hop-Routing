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

#ifndef __INET_POSITIONTABLEMODIFIED_H
#define __INET_POSITIONTABLEMODIFIED_H

#include <map>
#include <vector>

#include "inet/common/INETDefs.h"
#include "inet/common/geometry/common/Coord.h"
#include "inet/networklayer/common/L3Address.h"
#include "Greedy_m.h"

namespace inet {

/**
 * This class provides a mapping between node addresses and their positions.
 */
class PositionTableModified
{
  private:
    struct AddressToPositionMapValue {
      simtime_t timestamp;
      Coord position;
      L3Address viaAddress;

      AddressToPositionMapValue() 
        : timestamp(-1),  // Negative value for invalid timestamp
          position(),     // Default Coord value, representing an invalid position
          viaAddress(Ipv4Address("0.0.0.0")) {}  // Default IPv4 address

      AddressToPositionMapValue(simtime_t t, Coord p, L3Address v)
          : timestamp(t), position(p), viaAddress(v) {}
    };
    typedef std::map<L3Address, AddressToPositionMapValue> AddressToPositionMap;

    // Define separate maps for 1-hop and 2-hop neighbors
    AddressToPositionMap addressToPositionMapOneHop;
    AddressToPositionMap addressToPositionMapTwoHop;
    AddressToPositionMap addressToPositionMapThreeHop;
    AddressToPositionMap addressToPositionMapFourHop;


  public:
    PositionTableModified() {}

    // Function to get 1-hop neighbor addresses
    L3AddressVector getOneHopNeighborAddresses() const;
    // Function to get 2-hop neighbor addresses
    L3AddressVector getTwoHopNeighborAddresses() const;
    // Function to get 3-hop neighbor addresses
    L3AddressVector getThreeHopNeighborAddresses() const;
    // Function to get 4-hop neighbor addresses
    L3AddressVector getFourHopNeighborAddresses() const;

    // Check if an address is present in the 1-hop neighbors map
    bool hasOneHopNeighborPosition(const L3Address& address) const;
    // Check if an address is present in the 2-hop neighbors map
    bool hasTwoHopNeighborPosition(const L3Address& address) const;
    // Check if an address is present in the 3-hop neighbors map
    bool hasThreeHopNeighborPosition(const L3Address& address) const;
    // Check if an address is present in the 4-hop neighbors map
    bool hasFourHopNeighborPosition(const L3Address& address) const;
    // Get position for a 1-hop neighbor
    Coord getOneHopNeighborPosition(const L3Address& address) const;
    // Get position for a 2-hop neighbor
    Coord getTwoHopNeighborPosition(const L3Address& address) const;
    // Get position for a 3-hop neighbor
    Coord getThreeHopNeighborPosition(const L3Address& address) const;
    // Get position for a 4-hop neighbor
    Coord getFourHopNeighborPosition(const L3Address& address) const;
    // New function declaration
    Coord getNeighborPosition(const L3Address& address) const;
    // Get viaAddress for a 1-hop neighbor
    L3Address getOneHopNeighborViaAddress(const L3Address& address) const;
    // Get viaAddress for a 2-hop neighbor
    L3Address getTwoHopNeighborViaAddress(const L3Address& address) const;
    // Get viaAddress for a 3-hop neighbor
    L3Address getThreeHopNeighborViaAddress(const L3Address& address) const;
    // Get viaAddress for a 4-hop neighbor
    L3Address getFourHopNeighborViaAddress(const L3Address& address) const;
    // Get 2-hop neighbors known from the viaAddress 
    L3AddressVector getTwoHopNeighborsWithViaAddress(const L3Address& viaAddress) const;
    // Get 3-hop neighbors known from the viaAddress 
    L3AddressVector getThreeHopNeighborsWithViaAddress(const L3Address& viaAddress) const;
    // Get 4-hop neighbors known from the viaAddress 
    L3AddressVector getFourHopNeighborsWithViaAddress(const L3Address& viaAddress) const;

    // Set position for a 1-hop neighbor
    void setOneHopNeighborPosition(const L3Address& address, const Coord& coord, const L3Address& viaAddress);
    // Set position for a 2-hop neighbor
    void setTwoHopNeighborPosition(const L3Address& address, const Coord& coord, const L3Address& viaAddress);
    // Set position for a 3-hop neighbor
    void setThreeHopNeighborPosition(const L3Address& address, const Coord& coord, const L3Address& viaAddress);
    // Set position for a 4-hop neighbor
    void setFourHopNeighborPosition(const L3Address& address, const Coord& coord, const L3Address& viaAddress);
    // Set position for a k-hop neighbor
    void setPositionBasedOnHops(const L3Address& address, const Coord& coord, const L3Address& viaAddress, int hopCount);
    // Remove position for a 1-hop neighbor
    void removeOneHopNeighborPosition(const L3Address& address);
    // Remove position for a 2-hop neighbor
    void removeTwoHopNeighborPosition(const L3Address& address);
    // Remove position for a 3-hop neighbor
    void removeThreeHopNeighborPosition(const L3Address& address);
    // Remove position for a 4-hop neighbor
    void removeFourHopNeighborPosition(const L3Address& address);
    // Remove old positions for 1-hop neighbors
    void removeOldOneHopNeighborPositions(simtime_t timestamp);
    // Remove old positions for 2-hop neighbors
    void removeOldTwoHopNeighborPositions(simtime_t timestamp);
    // Remove old positions for 3-hop neighbors
    void removeOldThreeHopNeighborPositions(simtime_t timestamp);
    // Remove old positions for 4-hop neighbors
    void removeOldFourHopNeighborPositions(simtime_t timestamp);

    // Clear 1-hop neighbor map
    void clearOneHopMap();
    // Clear 2-hop neighbor map
    void clearTwoHopMap();
    // Clear 3-hop neighbor map
    void clearThreeHopMap();
    // Clear 4-hop neighbor map
    void clearFourHopMap();

    // Get the oldest position from the 1-hop map
    simtime_t getOldestOneHopPosition() const;
    // Get the oldest position from the 2-hop map
    simtime_t getOldestTwoHopPosition() const;
    // Get the oldest position from the 3-hop map
    simtime_t getOldestThreeHopPosition() const;
    // Get the oldest position from the 4-hop map
    simtime_t getOldestFourHopPosition() const;


    friend std::ostream& operator << (std::ostream& o, const PositionTableModified& t);
};

} // namespace inet

#endif // ifndef __INET_POSITIONTABLEMODIFIED_H

