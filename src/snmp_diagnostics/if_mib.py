# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""A module that can process SNMP IF-MIB data into ROS diagnostics."""

from __future__ import print_function

__all__ = ['IfMibDiagnostics']

import sys

from pysnmp.hlapi import ObjectType, ObjectIdentity
from pysnmp.hlapi.varbinds import AbstractVarBinds

import rospy

from diagnostic_msgs.msg import DiagnosticStatus

from .snmp_diag_module import SnmpDiagModule


if_speed_names = {
    10000000: "10 Mbps",
    100000000: "100 Mbps",
    1000000000: "1 Gbps",
    2500000000: "2.5 Gbps",
    5000000000: "5 Gbps",
    10000000000: "10 Gbps",
}


port_oper_status_names = {
    1: "up",
    2: "down",
    3: "testing",
    4: "unknown",
    5: "dormant",
    6: "notPresent",
    7: "lowerLayerDown",
}


class NetworkInterfaceStatus:
    name = ""
    alias = ""
    speed = 0
    operational_status = 0
    connector_present = True
    mtu = 1500

    def __str__(self):
        status = "N/C"
        if self.connector_present:
            if self.operational_status == 1:
                status = if_speed_names[self.speed]
            else:
                status = "Down"

        return "%s (%s)" % (self.name, status)


class NetworkInterfaceDesiredStatus:
    connected = None
    speed = None
    mtu = None


def startswith(oid1, oid2):
    if len(oid1) < len(oid2):
        return False
    for i in range(len(oid2)):
        if oid1[i] != oid2[i]:
            return False
    return True


ifDescr = ifMtu = ifSpeed = ifOperStatus = ifName = ifHighSpeed = ifConnectorPresent = ifAlias = None


class IfMibDiagnostics(SnmpDiagModule):
    """A module that can process SNMP IF-MIB data into ROS diagnostics.
    
    ROS parameters of the module are:

    - `if_mib` (dict)

      - `num_ports` (int, optional): If specified, the diagnostics will check that exactly this number of
        ports is reported.
      - `ports` (dict `port_name` => `port_params`): Specification of the expected ports. `port_name` is
        a string identifying the port (e.g. `eth0`), decoded either from `ifAlias`, `ifName` or
        `ifDescr` (in this order). `port_params` is a dict with following keys:

        - `connected` (bool, optional): Check that the connection state of this port is as specified.
        - `speed` (int or tuple (int, int), optional): The expected speed of the port in Mbps. If a tuple
          is specified, it denotes a (min, max) range.
        - `mtu` (int or tuple (int, int), optional): The expected MTU of the port in B. If a tuple
          is specified, it denotes a (min, max) range.
        - `dummy` (whatever): If you only need to specify a port to be present, but do not require
          any particular properties or state, just specify a dummy parameter so that the port's key
          is present in the `ports` dictionary.
    """

    oids = [
        ObjectType(ObjectIdentity('IF-MIB', 'ifAlias')),
        ObjectType(ObjectIdentity('IF-MIB', 'ifName')),
        ObjectType(ObjectIdentity('IF-MIB', 'ifDescr')),
        ObjectType(ObjectIdentity('IF-MIB', 'ifHighSpeed')),
        ObjectType(ObjectIdentity('IF-MIB', 'ifSpeed')),
        ObjectType(ObjectIdentity('IF-MIB', 'ifOperStatus')),
        ObjectType(ObjectIdentity('IF-MIB', 'ifConnectorPresent')),
        ObjectType(ObjectIdentity('IF-MIB', 'ifMtu')),
    ]

    def __init__(self, engine):
        """
        :param pysnmp.hlapi.v3arch.SnmpEngine engine: The SNMP engine instance.
        """
        super(IfMibDiagnostics, self).__init__(engine)

        global ifDescr, ifMtu, ifSpeed, ifOperStatus, ifName, ifHighSpeed, ifConnectorPresent, ifAlias
        (
            ifDescr, ifMtu, ifSpeed, ifOperStatus, ifName, ifHighSpeed, ifConnectorPresent, ifAlias,
        ) = self.mib_builder.importSymbols(
            'IF-MIB',
            'ifDescr', 'ifMtu', 'ifSpeed', 'ifOperStatus', 'ifName', 'ifHighSpeed', 'ifConnectorPresent', 'ifAlias',
        )

        config = rospy.get_param("~modules/if_mib", {})
        self.num_ports = config.get("num_ports", None)
        self.desired_port_status = {}
        ports = config.get("ports", {})
        for port in ports:
            s = NetworkInterfaceDesiredStatus()
            if "connected" in ports[port]:
                s.connected = bool(ports[port]["connected"])
            if "speed" in ports[port]:
                s.speed = ports[port]["speed"] * 1000000
            if "mtu" in ports[port]:
                s.mtu = ports[port]["mtu"]
            self.desired_port_status[port] = s
        self.last_response = None

    def get_diag_name(self):
        return "Network Interfaces"

    def get_oids(self):
        return self.oids if (self.num_ports is not None or len(self.desired_port_status) > 0) else []

    def parse_response(self, response_iterator):
        response = list()
        for errorIndication, errorStatus, errorIndex, varBinds in response_iterator:
            if errorIndication:
                print(errorIndication, file=sys.stderr)
                break
            elif errorStatus:
                print('%s at %s' % (errorStatus.prettyPrint(), errorIndex and varBinds[int(errorIndex)-1][0] or '?'),
                      file=sys.stderr)
                break
            else:
                status = NetworkInterfaceStatus()
                for oid, val in varBinds:
                    oid = oid.getOid().asTuple()
                    if startswith(oid, ifAlias.name):
                        status.alias = status.name = str(val)
                    elif startswith(oid, ifName.name):
                        if status.name == "":
                            status.name = str(val)
                    elif startswith(oid, ifDescr.name):
                        if status.name == "":
                            status.name = str(val)
                    elif startswith(oid, ifHighSpeed.name):
                        if val != 0:
                            status.speed = int(val) * 1000000
                    elif startswith(oid, ifSpeed.name):
                        if val != 0 and status.speed == 0:
                            status.speed = int(val)
                    elif startswith(oid, ifOperStatus.name):
                        status.operational_status = int(val)
                    elif startswith(oid, ifConnectorPresent.name):
                        status.connector_present = int(val) == 1
                    elif startswith(oid, ifMtu.name):
                        status.mtu = int(val)
                response.append(status)
        if len(response) == 0:
            response = None
        return response

    def process_response(self, response):
        self.last_response = response

    def produce_diagnostics(self, diagnostics):
        if self.last_response is None:
            diagnostics.summary(DiagnosticStatus.STALE, "No diagnostics received")
            return
        response = self.last_response
        self.last_response = None

        diagnostics.summary(DiagnosticStatus.OK, "Network interfaces are okay.")

        port_indices = dict()
        for i in range(len(response)):
            port_indices[response[i].name] = i

        if self.num_ports is not None and len(response) != self.num_ports:
            diagnostics.mergeSummary(DiagnosticStatus.ERROR, "Wrong number of ports")
        for port in self.desired_port_status:
            if port not in port_indices:
                diagnostics.mergeSummary(DiagnosticStatus.ERROR, "Port %s missing in SNMP response" % (port,))
                continue

        for i in range(len(response)):
            port = response[i].name

            if port in self.desired_port_status:
                diagnostics.add(port + " connection status", response[i].operational_status)
                diagnostics.add(port + " connection status string",
                                port_oper_status_names[response[i].operational_status])
                if self.desired_port_status[port].connected is not None:
                    if self.desired_port_status[port].connected:
                        if response[i].operational_status != 1:
                            diagnostics.mergeSummary(DiagnosticStatus.ERROR, "Port %s not connected" % (port,))
                    else:
                        if response[i].operational_status == 1:
                            diagnostics.mergeSummary(DiagnosticStatus.ERROR, "Port %s connected" % (port,))

                diagnostics.add(port + " speed [Mbps]", int(response[i].speed / 1000000))
                if self.desired_port_status[port].speed is not None:
                    if isinstance(self.desired_port_status[port].speed, int):
                        if response[i].speed != self.desired_port_status[port].speed:
                            diagnostics.mergeSummary(DiagnosticStatus.ERROR, "Wrong speed of port %s." % (port,))
                            diagnostics.add(port + " desired speed [Mbps]",
                                            int(self.desired_port_status[port].speed / 1000000))
                    elif isinstance(self.desired_port_status[port].speed, (tuple, list)):
                        min_speed = self.desired_port_status[port].speed[0]
                        max_speed = self.desired_port_status[port].speed[1]
                        if not (min_speed <= response[i].speed <= max_speed):
                            diagnostics.mergeSummary(DiagnosticStatus.ERROR, "Wrong speed of port %s" % (port,))
                            diagnostics.add(port + " desired speed [Mbps]", "%i - %i" % (
                                int(min_speed / 1000000), int(max_speed / 1000000)))

                diagnostics.add(port + " MTU [B]", response[i].mtu)
                if self.desired_port_status[port].mtu is not None:
                    if isinstance(self.desired_port_status[port].mtu, int):
                        if response[i].mtu != self.desired_port_status[port].mtu:
                            diagnostics.mergeSummary(DiagnosticStatus.ERROR, "Wrong MTU of port %s" % (port,))
                            diagnostics.add(port + " desired MTU [B]", self.desired_port_status[port].speed)
                    elif isinstance(self.desired_port_status[port].mtu, (tuple, list)):
                        min_mtu = self.desired_port_status[port].mtu[0]
                        max_mtu = self.desired_port_status[port].mtu[1]
                        if not (min_mtu <= response[i].mtu <= max_mtu):
                            diagnostics.mergeSummary(DiagnosticStatus.ERROR, "Wrong MTU of port %s" % (port,))
                            diagnostics.add(port + " desired MTU [B]", "%i - %i" % self.desired_port_status[port].mtu)
            else:
                status = "Not Connected"
                if response[i].connector_present:
                    if response[i].operational_status == 1:
                        status = if_speed_names.get(response[i].speed, "%i Mbps" % (int(response[i].speed / 1000000),))
                    else:
                        status = "Down"
                diagnostics.add(port + " status", status)
