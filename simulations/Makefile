# The LDACS Greedy K-Hop Routing implements the greedy routing algorithm with a subset of k-hop neighborhood.
# Copyright (C) 2024  Musab Ahmed, Konrad Fuger, Koojana Kuladinithi, Andreas Timm-Giel, Institute of Communication Networks, Hamburg University of Technology, Hamburg, Germany

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.

# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


###
# This file is responsible for providing the building the toolchain .
###

build-debug:	
	cd ../../; \
	echo -e "\nLDACSAbstractRadio"; \
	cd ldacs_abstract_radio/src; opp_makemake --make-so -f --deep -KINET_PROJ=../../inet4 -DINET_IMPORT -I../../inet4/src -L../../inet4/src -lINET_dbg; make -j8 MODE=debug; cd ../..; \
	echo -e "\nLDACSAbstractTdma"; \
	cd ldacs_abstract_tdma_mac/src; opp_makemake --make-so -f --deep -KINET_PROJ=../../inet4 -DINET_IMPORT -I../../inet4/src -L../../inet4/src -lINET_dbg; make -j8 MODE=debug; cd ../..; \
	echo -e "\nGreedyForwarding"; \
	cd ldacs_greedy_k_hop_routing/src; opp_makemake -f --deep -O out -KINET4_PROJ=../../inet4 -DINET_IMPORT -I../../inet4 -I../../ldacs_abstract_radio/src -I../../ldacs_abstract_tdma_mac/src -I. -I../../inet4/src -L../../inet4/src -L../../ldacs_abstract_radio/out/gcc-debug/src/ -L../../ldacs_abstract_tdma_mac/out/gcc-debug/src/ -lINET_dbg -lldacs_abstract_radio_dbg -lldacs_abstract_tdma_mac_dbg; make -j$(NUM_CPUS) MODE=debug; \

build-release:
	cd ../../; \
	echo -e "\nLDACSAbstractRadio"; \
	cd ldacs_abstract_radio/src; opp_makemake --make-so -f --deep -KINET_PROJ=../../inet4 -DINET_IMPORT -I../../inet4/src -L../../inet4/src -lINET; make -j8 MODE=release; cd ../..; \
	echo -e "\nLDACSAbstractTdma"; \
	cd ldacs_abstract_tdma_mac/src; opp_makemake --make-so -f --deep -KINET_PROJ=../../inet4 -DINET_IMPORT -I../../inet4/src -L../../inet4/src -lINET; make -j8 MODE=release; cd ../..; \
	echo -e "\nGreedyForwarding"; \
	cd ldacs_greedy_k_hop_routing/src; opp_makemake -f --deep -O out -KINET4_PROJ=../../inet4 -DINET_IMPORT -I../../inet4 -I../../ldacs_abstract_radio/src -I../../ldacs_abstract_tdma_mac/src -I. -I../../inet4/src -L../../inet4/src -L../../ldacs_abstract_radio/out/gcc-release/src/ -L../../ldacs_abstract_tdma_mac/out/gcc-release/src/ -lINET -lldacs_abstract_radio -lldacs_abstract_tdma_mac; make -j$(NUM_CPUS) MODE=release