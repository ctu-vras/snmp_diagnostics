<!--
SPDX-License-Identifier: BSD-3-Clause
SPDX-FileCopyrightText: Czech Technical University in Prague
-->
# snmp_diagnostics

ROS diagnostics created from SNMP agent data

## Nodes

### snmp_diagnostics

ROS node that reads various data over SNMP and turns them into ROS diagnostics messages.

#### Parameters

- `~agent_address` (str, default '127.0.0.1'): Address of the SNMP agent.
- `~agent_port` (int, default 161): UDP port of the SNMP agent.
- `~community` (str, default 'public'): SNMP community.
- `~snmp_v2` (bool, default True): Whether to use SNMPv2c or SNMPv1.
- `~udpv6` (bool, default False): Whether to use UDPv6 transport or UDPv4.
- `~rate` (float, default 1.0): The rate at which the SNMP agent is polled.
- `~modules` (dict): Module-specific configuration. Keys are names of the modules (e.g. `if_mib`).
    Values are dictionaries described further for each module.

## Known MIB Processing Modules

So far, there is only a single module, but more are planned to be added (e.g. host resources).

### if_mib

This module reads `IF-MIB` (1.3.6.1.2.1.2 and 1.3.6.1.2.1.31) and extracts information about network
ports and their status.

#### Parameters

- `num_ports` (int, optional): If specified, the diagnostics will check that exactly this number of
    ports is reported.
- `ports` (dict `port_name` => `port_params`): Specification of the expected ports. `port_name` is
    a string identifying the port (e.g. `eth0`), mangled either from `ifAlias`, `ifName` or
    `ifDescr` (see below). `port_params` is a dict with following keys:
  - `connected` (bool, optional): Check that the connection state of this port is as specified.
  - `speed` (int or tuple (int, int), optional): The expected speed of the port in Mbps. If a tuple
      is specified, it denotes a (min, max) range.
  - `mtu` (int or tuple (int, int), optional): The expected MTU of the port in B. If a tuple
      is specified, it denotes a (min, max) range.
  - `dummy` (whatever): If you only need to specify a port to be present, but do not require
      any particular properties or state, just specify a dummy parameter so that the port's key
      is present in the `ports` dictionary.

The `port_name` is mangled so that it is a valid ROS graph resource name - i.e. matching regex 
`^[a-zA-Z][a-zA-Z0-9_]*$`. To perform this mangling, iconv //TRANSLIT feature is used
to find the "closest" ASCII character to all non-ASCII ones, and then all non-alphanumeric
characters are replaced by underscores (e.g. spaces), and multiple underscores are coalesced into
a single one. If the resulting name is not a valid graph resource name (e.g. it starts with a number),
the next name "source" is tried - in the order `ifAlias`, `ifName`, `ifDescr`.

## Easy configuration of local machine to provide SNMP info about itself

This is a simplified version of the process described at https://askubuntu.com/a/223734/153828 .

- Install system packages `snmp`, `snmpd`, `snmp-mibs-downloader` .
- Run `echo "view   systemonly  included   .1.3.6.1.2.1.2" | sudo tee -a /etc/snmp/snmpd.conf`
    for each MIB you want to enable. Just substitute the exact MIB number.
- `sudo sed -i 's/^mibs :$/# mibs :/' /etc/snmp/snmp.conf`
- `sudo systemctl enable snmpd && sudo systemctl start snmpd`
- Then you should be able to walk your local machine: `snmpwalk -v 2c -c public 127.0.0.1` . If not,
    try restarting the `snmpd` daemon.