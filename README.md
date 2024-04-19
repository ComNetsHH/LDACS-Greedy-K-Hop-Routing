[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.10995656.svg)](https://doi.org/10.5281/zenodo.10995656)

# LDACS-Greedy-K-Hop-Routing

## Overview
LDACS-Greedy-K-Hop-Routing is a core component of the LDACS-Greedy-K-Hop-Simulator project. It introduces three novel routing methods—Greedy-FFT, Greedy-EFFT, and Greedy-Random—for selecting subsets of neighbors at precisely k-hop distances. This module implements the Greedy-k forwarding technique to enhance the routing efficiency over the conventional Greedy-1 approach by leveraging knowledge of the k-hop neighborhood. To accommodate the limitations of LDACS A2A slot capacities, our approach integrates a subset of 1-hop neighbors within beacon messages, while utilizing unused beacon slots to extend this insight to 2-hop and partially to 3-hop neighbors. This strategy provides a balanced view of the network topology, optimizing routing decisions and communication overhead within the constraints of LDACS A2A communications.

## Installation
Clone the repository and follow the setup instructions:
```bash
git clone https://github.com/ComNetsHH/LDACS-Greedy-K-Hop-Routing.git
cd LDACS-Greedy-K-Hop-Routing/simulation
make build-releases
```
