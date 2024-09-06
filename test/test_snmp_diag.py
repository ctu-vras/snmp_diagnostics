#!/usr/bin/env python

# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Unit test for snmp_diagnostics."""

import rospy
import rostest
import unittest

from pysnmp.hlapi import SnmpEngine

from snmp_diagnostics.plugin_manager import PluginManager


class SnmpDiag(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super(SnmpDiag, self).__init__(*args, **kwargs)
        rospy.init_node("snmp_diag_test")

    def test_plugin_instantiate(self):
        engine = SnmpEngine()
        plugin = PluginManager._instantiate_plugin(
            "snmp_diagnostics.plugins.if_mib.IfMibDiagnostics", engine, {"if_mib": {}})
        self.assertEqual("Network Interfaces", plugin.get_diag_name())

    def test_empty_plugin_manager(self):
        engine = SnmpEngine()
        plugin_manager = PluginManager(engine, {})
        self.assertEqual(0, len(plugin_manager.plugins))

    def test_plugin_manager(self):
        engine = SnmpEngine()
        plugin_manager = PluginManager(engine, {"if_mib": {}})
        self.assertEqual(1, len(plugin_manager.plugins))
        plugin = list(plugin_manager.plugins)[0]
        self.assertTrue(plugin.__class__.__name__.endswith("IfMibDiagnostics"))
        self.assertEqual("Network Interfaces", plugin.get_diag_name())


if __name__ == '__main__':
    rostest.rosrun("snmp_diagnostics", "test_snmp_diag", SnmpDiag)
