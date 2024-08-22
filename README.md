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
- `~modules` (dict): Module-specific configuration.

## Known MIB Processing Modules

So far, there is only a single module, but more are planned to be added (e.g. host resources).

### if_mib

This module reads `IF-MIB` and extracts information about network ports and their status