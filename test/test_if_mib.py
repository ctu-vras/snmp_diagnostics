#!/usr/bin/env python
# coding: utf-8

# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Unit test for IfMibDiagnostics."""

import rospy
import rostest
import unittest

from pysnmp.hlapi import SnmpEngine

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from diagnostic_updater import DiagnosticStatusWrapper
from snmp_diagnostics.plugins.if_mib import IfMibDiagnostics, NetworkInterfaceStatus


class IfMibTest(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super(IfMibTest, self).__init__(*args, **kwargs)
        rospy.init_node("if_mib_test")

    def test_if_mib(self):
        engine = SnmpEngine()
        config = {
            "num_ports": 3,
            "ports": {
                "eth0": {},
                "eth1": {},
                "eth2": {},
            }
        }
        plugin = IfMibDiagnostics(engine, config)
        self.assertEqual("Network Interfaces", plugin.get_diag_name())
        self.assertGreaterEqual(len(plugin.get_oids()), 8)

        eth0_status = NetworkInterfaceStatus()
        eth0_status.name = eth0_status.config_key = eth0_status.pretty_name = "eth0"
        eth0_status.operational_status = 1
        eth0_status.connector_present = True

        eth1_status = NetworkInterfaceStatus()
        eth1_status.name = eth1_status.config_key = eth1_status.pretty_name = "eth1"
        eth1_status.operational_status = 1
        eth1_status.connector_present = True

        eth2_status = NetworkInterfaceStatus()
        eth2_status.name = eth2_status.config_key = eth2_status.pretty_name = "eth2"
        eth2_status.operational_status = 1
        eth2_status.connector_present = True

        response = [eth0_status, eth1_status]
        plugin.process_response(response)

        diag = DiagnosticStatusWrapper()
        plugin.produce_diagnostics(diag)

        self.assertEqual(DiagnosticStatus.ERROR, diag.level)
        self.assertEqual("Wrong number of ports; Port eth2 missing in SNMP response", diag.message)

        response = [eth0_status, eth1_status, eth2_status]
        plugin.process_response(response)

        diag = DiagnosticStatusWrapper()
        plugin.produce_diagnostics(diag)

        self.assertEqual(DiagnosticStatus.OK, diag.level)
        self.assertEqual("Network interfaces are okay.", diag.message)

        self.assertIn(KeyValue("eth0 connection status", "1"), diag.values)
        self.assertIn(KeyValue("eth0 connection status string", "up"), diag.values)
        self.assertIn(KeyValue("eth0 speed [Mbps]", "0"), diag.values)
        self.assertIn(KeyValue("eth0 MTU [B]", "1500"), diag.values)
        self.assertIn(KeyValue("eth1 connection status", "1"), diag.values)
        self.assertIn(KeyValue("eth1 connection status string", "up"), diag.values)
        self.assertIn(KeyValue("eth1 speed [Mbps]", "0"), diag.values)
        self.assertIn(KeyValue("eth1 MTU [B]", "1500"), diag.values)
        self.assertIn(KeyValue("eth2 connection status", "1"), diag.values)
        self.assertIn(KeyValue("eth2 connection status string", "up"), diag.values)
        self.assertIn(KeyValue("eth2 speed [Mbps]", "0"), diag.values)
        self.assertIn(KeyValue("eth2 MTU [B]", "1500"), diag.values)

        eth0_status.operational_status = 2
        eth0_status.connector_present = False
        eth1_status.operational_status = 5
        eth1_status.connector_present = True
        eth2_status.operational_status = 6
        eth2_status.connector_present = False
        plugin.process_response(response)

        diag = DiagnosticStatusWrapper()
        plugin.produce_diagnostics(diag)

        self.assertEqual(DiagnosticStatus.OK, diag.level)
        self.assertEqual("Network interfaces are okay.", diag.message)

        self.assertIn(KeyValue("eth0 connection status", "2"), diag.values)
        self.assertIn(KeyValue("eth0 connection status string", "down"), diag.values)
        self.assertIn(KeyValue("eth1 connection status", "5"), diag.values)
        self.assertIn(KeyValue("eth1 connection status string", "dormant"), diag.values)
        self.assertIn(KeyValue("eth2 connection status", "6"), diag.values)
        self.assertIn(KeyValue("eth2 connection status string", "notPresent"), diag.values)

        config = {
            "num_ports": 3,
            "ports": {
                "eth0": {"connected": True, "speed": 1000, "mtu": 9000},
                "eth1": {"connected": True, "speed": [100, 1000], "mtu": [1000, 2000]},
                "eth2": {"connected": False, "speed": 100, "mtu": 1436},
            }
        }
        eth0_status.operational_status = 1
        eth0_status.connector_present = True
        eth0_status.speed = 1000 * 1000000
        eth0_status.mtu = 9000
        eth1_status.operational_status = 1
        eth1_status.connector_present = True
        eth1_status.speed = 100 * 1000000
        eth1_status.mtu = 1436
        eth2_status.operational_status = 2
        eth2_status.connector_present = True
        eth2_status.speed = 100 * 1000000
        eth2_status.mtu = 1436
        plugin = IfMibDiagnostics(engine, config)
        plugin.process_response(response)

        diag = DiagnosticStatusWrapper()
        plugin.produce_diagnostics(diag)

        self.assertEqual(DiagnosticStatus.OK, diag.level)
        self.assertEqual("Network interfaces are okay.", diag.message)

        self.assertIn(KeyValue("eth0 connection status", "1"), diag.values)
        self.assertIn(KeyValue("eth0 connection status string", "up"), diag.values)
        self.assertIn(KeyValue("eth1 connection status", "1"), diag.values)
        self.assertIn(KeyValue("eth1 connection status string", "up"), diag.values)
        self.assertIn(KeyValue("eth2 connection status", "2"), diag.values)
        self.assertIn(KeyValue("eth2 connection status string", "down"), diag.values)

        eth0_status.speed = 100 * 1000000
        eth0_status.mtu = 8000
        eth1_status.speed = 10 * 1000000
        eth1_status.mtu = 436
        eth2_status.speed = 1000 * 1000000
        eth2_status.mtu = 14361
        plugin.process_response(response)

        diag = DiagnosticStatusWrapper()
        plugin.produce_diagnostics(diag)

        self.assertEqual(DiagnosticStatus.ERROR, diag.level)
        self.assertEqual("Wrong speed of port eth0; Wrong MTU of port eth0; Wrong speed of port eth1; "
                         "Wrong MTU of port eth1; Wrong speed of port eth2; Wrong MTU of port eth2", diag.message)

        self.assertIn(KeyValue("eth0 connection status", "1"), diag.values)
        self.assertIn(KeyValue("eth0 connection status string", "up"), diag.values)
        self.assertIn(KeyValue("eth0 speed [Mbps]", "100"), diag.values)
        self.assertIn(KeyValue("eth0 MTU [B]", "8000"), diag.values)
        self.assertIn(KeyValue("eth1 connection status", "1"), diag.values)
        self.assertIn(KeyValue("eth1 connection status string", "up"), diag.values)
        self.assertIn(KeyValue("eth1 speed [Mbps]", "10"), diag.values)
        self.assertIn(KeyValue("eth1 MTU [B]", "436"), diag.values)
        self.assertIn(KeyValue("eth2 connection status", "2"), diag.values)
        self.assertIn(KeyValue("eth2 connection status string", "down"), diag.values)
        self.assertIn(KeyValue("eth2 speed [Mbps]", "1000"), diag.values)
        self.assertIn(KeyValue("eth2 MTU [B]", "14361"), diag.values)

        diag = DiagnosticStatusWrapper()
        plugin.produce_diagnostics(diag)

        self.assertEqual(DiagnosticStatus.STALE, diag.level)
        self.assertEqual('No diagnostics received', diag.message)

    def test_name_mangling(self):
        config = {
            "num_ports": 3,
            "ports": {
                "eth0": {"connected": True},
                "Muj_interfejc": {"connected": True},
                "some_e_3d_stuff_": {"connected": False},
            }
        }

        eth0_status = NetworkInterfaceStatus()
        eth0_status.name = eth0_status.config_key = eth0_status.pretty_name = "eth0"
        eth0_status.operational_status = 1
        eth0_status.connector_present = True

        eth1_status = NetworkInterfaceStatus()
        eth1_status.name = "eth1"
        eth1_status.alias = eth1_status.pretty_name = u"Můj ïnterfejč"
        eth1_status.config_key = "Muj_interfejc"
        eth1_status.operational_status = 1
        eth1_status.connector_present = True

        eth2_status = NetworkInterfaceStatus()
        eth2_status.descr = eth2_status.pretty_name = u" some € 3d ? stuff???"
        eth2_status.config_key = "some_e_3d_stuff_"
        eth2_status.operational_status = 2
        eth2_status.connector_present = True

        response = [eth0_status, eth1_status, eth2_status]

        engine = SnmpEngine()
        plugin = IfMibDiagnostics(engine, config)
        plugin.process_response(response)

        diag = DiagnosticStatusWrapper()
        plugin.produce_diagnostics(diag)

        self.assertEqual("Network interfaces are okay.", diag.message)
        self.assertEqual(DiagnosticStatus.OK, diag.level)

        self.assertIn(KeyValue("eth0 connection status", "1"), diag.values)
        self.assertIn(KeyValue("eth0 connection status string", "up"), diag.values)
        self.assertIn(KeyValue(u"Můj ïnterfejč connection status", "1"), diag.values)
        self.assertIn(KeyValue(u"Můj ïnterfejč connection status string", "up"), diag.values)
        self.assertIn(KeyValue(u" some € 3d ? stuff??? connection status", "2"), diag.values)
        self.assertIn(KeyValue(u" some € 3d ? stuff??? connection status string", "down"), diag.values)


if __name__ == '__main__':
    rostest.rosrun("snmp_diagnostics", "test_if_mib", IfMibTest)
