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

#include "PositionTableModified.h"

namespace inet {

// Function to get 1-hop neighbor addresses
L3AddressVector PositionTableModified::getOneHopNeighborAddresses() const
{
    L3AddressVector oneHopNeighborsaddresses;
    for (const auto & elem : addressToPositionMapOneHop)
        oneHopNeighborsaddresses.push_back(elem.first);
    return oneHopNeighborsaddresses;
}

// Function to get 2-hop neighbor addresses
L3AddressVector PositionTableModified::getTwoHopNeighborAddresses() const {
    L3AddressVector twoHopNeighborsaddresses;
    for (const auto & elem : addressToPositionMapTwoHop)
        twoHopNeighborsaddresses.push_back(elem.first);
    return twoHopNeighborsaddresses;
}

// Function to get 3-hop neighbor addresses
L3AddressVector PositionTableModified::getThreeHopNeighborAddresses() const {
    L3AddressVector threeHopNeighborsaddresses;
    for (const auto & elem : addressToPositionMapThreeHop)
        threeHopNeighborsaddresses.push_back(elem.first);
    return threeHopNeighborsaddresses;
}

// Function to get 4-hop neighbor addresses
L3AddressVector PositionTableModified::getFourHopNeighborAddresses() const {
    L3AddressVector fourHopNeighborsaddresses;
    for (const auto & elem : addressToPositionMapFourHop)
        fourHopNeighborsaddresses.push_back(elem.first);
    return fourHopNeighborsaddresses;
}

// Check if an address is present in the 1-hop neighbors map
bool PositionTableModified::hasOneHopNeighborPosition(const L3Address& address) const
{
    AddressToPositionMap::const_iterator it = addressToPositionMapOneHop.find(address);
    return it != addressToPositionMapOneHop.end();
}

// Check if an address is present in the 2-hop neighbors map
bool PositionTableModified::hasTwoHopNeighborPosition(const L3Address& address) const {
    AddressToPositionMap::const_iterator it = addressToPositionMapTwoHop.find(address);
    return it != addressToPositionMapTwoHop.end();
}

// Check if an address is present in the 3-hop neighbors map
bool PositionTableModified::hasThreeHopNeighborPosition(const L3Address& address) const {
    AddressToPositionMap::const_iterator it = addressToPositionMapThreeHop.find(address);
    return it != addressToPositionMapThreeHop.end();
}

// Check if an address is present in the 4-hop neighbors map
bool PositionTableModified::hasFourHopNeighborPosition(const L3Address& address) const {
    AddressToPositionMap::const_iterator it = addressToPositionMapFourHop.find(address);
    return it != addressToPositionMapFourHop.end();
}

// Get position for a 1-hop neighbor
Coord PositionTableModified::getOneHopNeighborPosition(const L3Address& address) const 
{
    AddressToPositionMap::const_iterator it = addressToPositionMapOneHop.find(address);
    if (it == addressToPositionMapOneHop.end())
        return Coord(NaN, NaN, NaN);
    else
        return it->second.position;
}

// Get position for a 2-hop neighbor
Coord PositionTableModified::getTwoHopNeighborPosition(const L3Address& address) const 
{
    AddressToPositionMap::const_iterator it = addressToPositionMapTwoHop.find(address);
    if (it == addressToPositionMapTwoHop.end())
        return Coord(NaN, NaN, NaN);
    else
        return it->second.position;
}

// Get position for a 3-hop neighbor
Coord PositionTableModified::getThreeHopNeighborPosition(const L3Address& address) const 
{
    AddressToPositionMap::const_iterator it = addressToPositionMapThreeHop.find(address);
    if (it == addressToPositionMapThreeHop.end())
        return Coord(NaN, NaN, NaN);
    else
        return it->second.position;
}

// Get position for a 4-hop neighbor
Coord PositionTableModified::getFourHopNeighborPosition(const L3Address& address) const 
{
    AddressToPositionMap::const_iterator it = addressToPositionMapFourHop.find(address);
    if (it == addressToPositionMapFourHop.end())
        return Coord(NaN, NaN, NaN);
    else
        return it->second.position;
}

// Get position by searching 1-hop, 2-hop and 3-hop neighbor
Coord PositionTableModified::getNeighborPosition(const L3Address& address) const 
{
    // Check if the address is a one-hop neighbor and get the position
    if (hasOneHopNeighborPosition(address)) {
        return getOneHopNeighborPosition(address);
    }
    // Check if the address is a two-hop neighbor and get the position
    else if (hasTwoHopNeighborPosition(address)) {
        return getTwoHopNeighborPosition(address);
    }
    // Check if the address is a three-hop neighbor and get the position
    else if (hasThreeHopNeighborPosition(address)) {
        return getThreeHopNeighborPosition(address);
    }
    // Check if the address is a four-hop neighbor and get the position
    else if (hasFourHopNeighborPosition(address)) {
        return getFourHopNeighborPosition(address);
    }
    // If the address is not found in any, return invalid Coord
    return Coord(NaN, NaN, NaN);
}

// Get viaAddress for a 1-hop neighbor
L3Address PositionTableModified::getOneHopNeighborViaAddress(const L3Address& address) const 
{
    auto it = addressToPositionMapOneHop.find(address);
    if (it == addressToPositionMapOneHop.end())
        return L3Address();  // Return a default L3Address to indicate 'not found'
    else
        return it->second.viaAddress;
}

// Get viaAddress for a 2-hop neighbor
L3Address PositionTableModified::getTwoHopNeighborViaAddress(const L3Address& address) const 
{
    auto it = addressToPositionMapTwoHop.find(address);
    if (it == addressToPositionMapTwoHop.end())
        return L3Address();  // Return a default L3Address to indicate 'not found'
    else
        return it->second.viaAddress;
}

// Get viaAddress for a 3-hop neighbor
L3Address PositionTableModified::getThreeHopNeighborViaAddress(const L3Address& address) const 
{
    auto it = addressToPositionMapThreeHop.find(address);
    if (it == addressToPositionMapThreeHop.end())
        return L3Address();  // Return a default L3Address to indicate 'not found'
    else
        return it->second.viaAddress;
}

// Get viaAddress for a 4-hop neighbor
L3Address PositionTableModified::getFourHopNeighborViaAddress(const L3Address& address) const 
{
    auto it = addressToPositionMapFourHop.find(address);
    if (it == addressToPositionMapFourHop.end())
        return L3Address();  // Return a default L3Address to indicate 'not found'
    else
        return it->second.viaAddress;
}

// Get 2-hop neighbors with specific viaAddress
L3AddressVector PositionTableModified::getTwoHopNeighborsWithViaAddress(const L3Address& viaAddress) const {
    L3AddressVector addresses;
    for (const auto& elem : addressToPositionMapTwoHop) {
        if (elem.second.viaAddress == viaAddress) {
            addresses.push_back(elem.first);
        }
    }
    return addresses;
}

// Get 3-hop neighbors with specific viaAddress
L3AddressVector PositionTableModified::getThreeHopNeighborsWithViaAddress(const L3Address& viaAddress) const {
    L3AddressVector addresses;
    for (const auto& elem : addressToPositionMapThreeHop) {
        if (elem.second.viaAddress == viaAddress) {
            addresses.push_back(elem.first);
        }
    }
    return addresses;
}

// Get 4-hop neighbors with specific viaAddress
L3AddressVector PositionTableModified::getFourHopNeighborsWithViaAddress(const L3Address& viaAddress) const {
    L3AddressVector addresses;
    for (const auto& elem : addressToPositionMapFourHop) {
        if (elem.second.viaAddress == viaAddress) {
            addresses.push_back(elem.first);
        }
    }
    return addresses;
}

// Set position for a 1-hop neighbor
void PositionTableModified::setOneHopNeighborPosition(const L3Address& address, const Coord& coord, const L3Address& viaAddress) 
{
    ASSERT(!address.isUnspecified());
    addressToPositionMapOneHop[address] = AddressToPositionMapValue(simTime(), coord, viaAddress);
}

// Set position for a 2-hop neighbor
void PositionTableModified::setTwoHopNeighborPosition(const L3Address& address, const Coord& coord, const L3Address& viaAddress) 
{
    ASSERT(!address.isUnspecified());
    if (addressToPositionMapOneHop.find(address) == addressToPositionMapOneHop.end()) {
        addressToPositionMapTwoHop[address] = AddressToPositionMapValue(simTime(), coord, viaAddress);
    }
}

// Set position for a 3-hop neighbor
void PositionTableModified::setThreeHopNeighborPosition(const L3Address& address, const Coord& coord, const L3Address& viaAddress) 
{
    ASSERT(!address.isUnspecified());
    if (addressToPositionMapOneHop.find(address) == addressToPositionMapOneHop.end() && addressToPositionMapTwoHop.find(address) == addressToPositionMapTwoHop.end()) {
        addressToPositionMapThreeHop[address] = AddressToPositionMapValue(simTime(), coord, viaAddress);
    }
}

// Set position for a 4-hop neighbor
void PositionTableModified::setFourHopNeighborPosition(const L3Address& address, const Coord& coord, const L3Address& viaAddress) 
{
    ASSERT(!address.isUnspecified());
    if (addressToPositionMapOneHop.find(address) == addressToPositionMapOneHop.end() && addressToPositionMapTwoHop.find(address) == addressToPositionMapTwoHop.end() && addressToPositionMapThreeHop.find(address) == addressToPositionMapThreeHop.end()) {
        addressToPositionMapFourHop[address] = AddressToPositionMapValue(simTime(), coord, viaAddress);
    }
}

void PositionTableModified::setPositionBasedOnHops(const L3Address& address, const Coord& coord, const L3Address& viaAddress, int hopCount) {
    ASSERT(!address.isUnspecified());

    if (hopCount == 1) {
        // Set as one-hop neighbor and remove from two-hop and three-hop if present
        addressToPositionMapOneHop[address] = AddressToPositionMapValue(simTime(), coord, viaAddress);
        removeTwoHopNeighborPosition(address);
        removeThreeHopNeighborPosition(address);
        removeFourHopNeighborPosition(address);
    } 
    else if (hopCount == 2) {
        // Set as two-hop neighbor only if not already a one-hop neighbor
        if (addressToPositionMapOneHop.find(address) == addressToPositionMapOneHop.end()) {
            addressToPositionMapTwoHop[address] = AddressToPositionMapValue(simTime(), coord, viaAddress);
            removeThreeHopNeighborPosition(address);
            removeFourHopNeighborPosition(address);
        }
    }
    else if (hopCount == 3) {
        // Set as three-hop neighbor only if not known through fewer hops
        if (addressToPositionMapOneHop.find(address) == addressToPositionMapOneHop.end() &&
            addressToPositionMapTwoHop.find(address) == addressToPositionMapTwoHop.end()) {
            addressToPositionMapThreeHop[address] = AddressToPositionMapValue(simTime(), coord, viaAddress);
            removeFourHopNeighborPosition(address);
        }
    }
    else if (hopCount == 4) {
        // Set as three-hop neighbor only if not known through fewer hops
        if (addressToPositionMapOneHop.find(address) == addressToPositionMapOneHop.end() &&
            addressToPositionMapTwoHop.find(address) == addressToPositionMapTwoHop.end() &&
            addressToPositionMapThreeHop.find(address) == addressToPositionMapThreeHop.end()) {
            addressToPositionMapFourHop[address] = AddressToPositionMapValue(simTime(), coord, viaAddress);
        }
    }
    // Additional logic for more hops can be added here if necessary
}

// Remove position for a 1-hop neighbor
void PositionTableModified::removeOneHopNeighborPosition(const L3Address& address) 
{
    auto it = addressToPositionMapOneHop.find(address);
    if (it != addressToPositionMapOneHop.end())
        addressToPositionMapOneHop.erase(it);
}

// Remove position for a 2-hop neighbor
void PositionTableModified::removeTwoHopNeighborPosition(const L3Address& address) 
{
    auto it = addressToPositionMapTwoHop.find(address);
    if (it != addressToPositionMapTwoHop.end())
        addressToPositionMapTwoHop.erase(it);
}

// Remove position for a 3-hop neighbor
void PositionTableModified::removeThreeHopNeighborPosition(const L3Address& address) 
{
    auto it = addressToPositionMapThreeHop.find(address);
    if (it != addressToPositionMapThreeHop.end())
        addressToPositionMapThreeHop.erase(it);
}

// Remove position for a 4-hop neighbor
void PositionTableModified::removeFourHopNeighborPosition(const L3Address& address) 
{
    auto it = addressToPositionMapFourHop.find(address);
    if (it != addressToPositionMapFourHop.end())
        addressToPositionMapFourHop.erase(it);
}

// Remove old positions for 1-hop neighbors
void PositionTableModified::removeOldOneHopNeighborPositions(simtime_t timestamp) 
{
    for (auto it = addressToPositionMapOneHop.begin(); it != addressToPositionMapOneHop.end(); ) {
        if (it->second.timestamp <= timestamp)
            it = addressToPositionMapOneHop.erase(it);
        else
            ++it;
    }
}

// Remove old positions for 2-hop neighbors
void PositionTableModified::removeOldTwoHopNeighborPositions(simtime_t timestamp) 
{
    for (auto it = addressToPositionMapTwoHop.begin(); it != addressToPositionMapTwoHop.end(); ) {
        if (it->second.timestamp <= timestamp)
            it = addressToPositionMapTwoHop.erase(it);
        else
            ++it;
    }
}

// Remove old positions for 3-hop neighbors
void PositionTableModified::removeOldThreeHopNeighborPositions(simtime_t timestamp) 
{
    for (auto it = addressToPositionMapThreeHop.begin(); it != addressToPositionMapThreeHop.end(); ) {
        if (it->second.timestamp <= timestamp)
            it = addressToPositionMapThreeHop.erase(it);
        else
            ++it;
    }
}

// Remove old positions for 4-hop neighbors
void PositionTableModified::removeOldFourHopNeighborPositions(simtime_t timestamp) 
{
    for (auto it = addressToPositionMapFourHop.begin(); it != addressToPositionMapFourHop.end(); ) {
        if (it->second.timestamp <= timestamp)
            it = addressToPositionMapFourHop.erase(it);
        else
            ++it;
    }
}

// Clear 1-hop neighbor map
void PositionTableModified::clearOneHopMap() 
{
    addressToPositionMapOneHop.clear();
}

// Clear 2-hop neighbor map
void PositionTableModified::clearTwoHopMap() 
{
    addressToPositionMapTwoHop.clear();
}

// Clear 3-hop neighbor map
void PositionTableModified::clearThreeHopMap() 
{
    addressToPositionMapThreeHop.clear();
}

// Clear 4-hop neighbor map
void PositionTableModified::clearFourHopMap() 
{
    addressToPositionMapFourHop.clear();
}

// Get the oldest position from the 1-hop map
simtime_t PositionTableModified::getOldestOneHopPosition() const
{
    simtime_t oldestPosition = SimTime::getMaxTime();
    for (const auto & elem : addressToPositionMapOneHop) {
        const simtime_t& time = elem.second.timestamp;
        if (time < oldestPosition)
            oldestPosition = time;
    }
    return oldestPosition;
}

// Get the oldest position from the 2-hop map
simtime_t PositionTableModified::getOldestTwoHopPosition() const {
    simtime_t oldestPosition = SimTime::getMaxTime();
    for (const auto & elem : addressToPositionMapTwoHop) {
        const simtime_t& time = elem.second.timestamp;
        if (time < oldestPosition)
            oldestPosition = time;
    }
    return oldestPosition;
}

// Get the oldest position from the 3-hop map
simtime_t PositionTableModified::getOldestThreeHopPosition() const {
    simtime_t oldestPosition = SimTime::getMaxTime();
    for (const auto & elem : addressToPositionMapThreeHop) {
        const simtime_t& time = elem.second.timestamp;
        if (time < oldestPosition)
            oldestPosition = time;
    }
    return oldestPosition;
}

// Get the oldest position from the 4-hop map
simtime_t PositionTableModified::getOldestFourHopPosition() const {
    simtime_t oldestPosition = SimTime::getMaxTime();
    for (const auto & elem : addressToPositionMapFourHop) {
        const simtime_t& time = elem.second.timestamp;
        if (time < oldestPosition)
            oldestPosition = time;
    }
    return oldestPosition;
}

std::ostream& operator << (std::ostream& o, const PositionTableModified& t) {
    o << "{ ||1-hop||: ";
    for (const auto& elem : t.addressToPositionMapOneHop) {
        o << elem.first << ":( t: " << elem.second.timestamp << "; pos: " << elem.second.position << "; via: " << elem.second.viaAddress<< ") ";
    }

    o << " ||2-hop||: ";
    for (const auto& elem : t.addressToPositionMapTwoHop) {
        o << elem.first << ":( t: " << elem.second.timestamp << "; pos: " << elem.second.position << "; via: " << elem.second.viaAddress<< ") ";
    }
    o << "}";

    o << " ||3-hop||: ";
    for (const auto& elem : t.addressToPositionMapThreeHop) {
        o << elem.first << ":( t: " << elem.second.timestamp << "; pos: " << elem.second.position << "; via: " << elem.second.viaAddress<< ") ";
    }
    o << "}";

    o << " ||4-hop||: ";
    for (const auto& elem : t.addressToPositionMapFourHop) {
        o << elem.first << ":( t: " << elem.second.timestamp << "; pos: " << elem.second.position << "; via: " << elem.second.viaAddress<< ") ";
    }
    o << "}";

    return o;
}

} // namespace inet

