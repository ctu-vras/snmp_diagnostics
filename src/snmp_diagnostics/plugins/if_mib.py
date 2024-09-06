# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""A plugins that can process SNMP IF-MIB data into ROS diagnostics."""

from __future__ import print_function

__all__ = ['IfMibDiagnostics', 'NetworkInterfaceStatus']

import sys

from pysnmp.hlapi import ObjectType, ObjectIdentity
from pysnmp.hlapi.varbinds import AbstractVarBinds

import cras
import rospy

from cras.string_utils import to_valid_ros_name
from diagnostic_msgs.msg import DiagnosticStatus

from snmp_diagnostics.snmp_diag_plugin import SnmpDiagPlugin

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
    descr = ""
    alias = ""
    config_key = ""
    pretty_name = ""
    speed = 0
    operational_status = 0
    connector_present = True
    mtu = 1500

    def determine_names(self):
        if self.alias:
            try:
                self.config_key = to_valid_ros_name(self.alias)
                self.pretty_name = self.alias
            except ValueError:
                pass

        if not self.config_key and self.name:
            try:
                self.config_key = to_valid_ros_name(self.name)
                self.pretty_name = self.name
            except ValueError:
                pass

        if not self.config_key and self.descr:
            try:
                self.config_key = to_valid_ros_name(self.descr)
                self.pretty_name = self.descr
            except ValueError:
                pass

        if not self.config_key:
            raise ValueError("Could not determine any name of network interface. "
                             "ifAlias='%s', ifName='%s', ifDescr='%s'" % (self.alias, self.name, self.descr))

        if self.pretty_name != self.config_key:
            cras.loginfo_once_identical("Port '%s' uses mangled config key '%s'" % (self.pretty_name, self.config_key))

    def __str__(self):
        status = "N/C"
        if self.connector_present:
            if self.operational_status == 1:
                status = if_speed_names[self.speed]
            else:
                status = "Down"

        return "%s (%s)" % (self.pretty_name, status)


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


class IfMibDiagnostics(SnmpDiagPlugin):
    """A plugin that can process SNMP IF-MIB data into ROS diagnostics.

    ROS parameters of the plugin are:

    - `if_mib` (dict)

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

    def __init__(self, engine, config):
        """
        :param pysnmp.hlapi.v3arch.SnmpEngine engine: The SNMP engine instance.
        :param dict config: The configuration dictionary.
        """
        super(IfMibDiagnostics, self).__init__(engine, config)

        global ifDescr, ifMtu, ifSpeed, ifOperStatus, ifName, ifHighSpeed, ifConnectorPresent, ifAlias
        (
            ifDescr, ifMtu, ifSpeed, ifOperStatus, ifName, ifHighSpeed, ifConnectorPresent, ifAlias,
        ) = self.mib_builder.importSymbols(
            'IF-MIB',
            'ifDescr', 'ifMtu', 'ifSpeed', 'ifOperStatus', 'ifName', 'ifHighSpeed', 'ifConnectorPresent', 'ifAlias',
        )

        self.num_ports = config.get("num_ports", None)
        self.desired_port_status = {}
        ports = config.get("ports", {})
        for port in ports:
            s = NetworkInterfaceDesiredStatus()
            if "connected" in ports[port]:
                s.connected = bool(ports[port]["connected"])
            if "speed" in ports[port]:
                s.speed = ports[port]["speed"]
                if isinstance(s.speed, (tuple, list)):
                    s.speed[0] *= 1000000
                    s.speed[1] *= 1000000
                else:
                    s.speed *= 1000000
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
                print('%s at %s' % (errorStatus.prettyPrint(), errorIndex and varBinds[int(errorIndex) - 1][0] or '?'),
                      file=sys.stderr)
                break
            else:
                status = NetworkInterfaceStatus()
                for oid, val in varBinds:
                    oid = oid.getOid().asTuple()
                    if startswith(oid, ifAlias.name):
                        status.alias = str(val)
                    elif startswith(oid, ifName.name):
                        status.name = str(val)
                    elif startswith(oid, ifDescr.name):
                        status.descr = str(val)
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
                try:
                    status.determine_names()
                except ValueError as e:
                    rospy.logerr_throttle_identical(10, str(e))

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
            port_indices[response[i].config_key] = i

        if self.num_ports is not None and len(response) != self.num_ports:
            diagnostics.mergeSummary(DiagnosticStatus.ERROR, "Wrong number of ports")
        for port_key in self.desired_port_status:
            if port_key not in port_indices:
                diagnostics.mergeSummary(DiagnosticStatus.ERROR, "Port %s missing in SNMP response" % (port_key,))
                continue

        for i in range(len(response)):
            port = response[i]
            port_key = port.config_key
            port_name = port.pretty_name

            if port_key in self.desired_port_status:
                desired_status = self.desired_port_status[port_key]

                diagnostics.add(port_name + " connection status", port.operational_status)
                diagnostics.add(port_name + " connection status string",
                                port_oper_status_names[port.operational_status])

                if desired_status.connected is True and port.operational_status != 1:
                    diagnostics.mergeSummary(DiagnosticStatus.ERROR, "Port %s not connected" % (port_name,))
                elif desired_status.connected is False and port.operational_status == 1:
                    diagnostics.mergeSummary(DiagnosticStatus.ERROR, "Port %s connected" % (port_name,))

                diagnostics.add(port_name + " speed [Mbps]", int(port.speed / 1000000))
                if isinstance(desired_status.speed, int) and port.speed != desired_status.speed:
                    diagnostics.mergeSummary(DiagnosticStatus.ERROR, "Wrong speed of port %s" % (port_name,))
                    diagnostics.add(port_name + " desired speed [Mbps]", int(desired_status.speed / 1000000))
                elif isinstance(desired_status.speed, (tuple, list)):
                    min_speed = desired_status.speed[0]
                    max_speed = desired_status.speed[1]
                    if not (min_speed <= port.speed <= max_speed):
                        diagnostics.mergeSummary(DiagnosticStatus.ERROR, "Wrong speed of port %s" % (port_name,))
                        diagnostics.add(port_name + " desired speed [Mbps]", "%i - %i" % (
                            int(min_speed / 1000000), int(max_speed / 1000000)))

                diagnostics.add(port_name + " MTU [B]", port.mtu)
                if isinstance(desired_status.mtu, int) and port.mtu != desired_status.mtu:
                    diagnostics.mergeSummary(DiagnosticStatus.ERROR, "Wrong MTU of port %s" % (port_name,))
                    diagnostics.add(port_name + " desired MTU [B]", desired_status.speed)
                elif isinstance(desired_status.mtu, (tuple, list)):
                    min_mtu = desired_status.mtu[0]
                    max_mtu = desired_status.mtu[1]
                    if not (min_mtu <= port.mtu <= max_mtu):
                        diagnostics.mergeSummary(DiagnosticStatus.ERROR, "Wrong MTU of port %s" % (port_name,))
                        diagnostics.add(port_name + " desired MTU [B]", "%i - %i" % tuple(desired_status.mtu))
            else:
                status = "Not Connected"
                if port.connector_present:
                    if port.operational_status == 1:
                        status = if_speed_names.get(port.speed, "%i Mbps" % (int(port.speed / 1000000),))
                    else:
                        status = "Down"
                diagnostics.add(port_name + " status", status)
