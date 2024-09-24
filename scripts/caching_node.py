# This file defines the availbe nodes of the V2X Demonstrator for local configuration
ETHERNET_IP_PREFIX = '192.168.3.'

# List of all systems available for v2x experiments
system_dict = {}

# Class representing the parameters need for adhoc configuration of a v2x enable system
class cachingNode:
    _num_systems = 1
    def __init__(self, label: str ,user: str, ethernet_interface: str, ethernet_ip_suffix: str):
        self.index = cachingNode._num_systems
        cachingNode._num_systems += 1
        self.label = label
        self.user = user
        self.ethernet_interface = ethernet_interface
        self.ethernet_ip = ETHERNET_IP_PREFIX + ethernet_ip_suffix
        system_dict[self.label] = self
        
# Caching Systems
kitt = cachingNode('kitt', 'kitt', 'enp4s0', '100')
bumblebee = cachingNode('bumblebee', 'bumblebee', 'enp4s0', '101')


