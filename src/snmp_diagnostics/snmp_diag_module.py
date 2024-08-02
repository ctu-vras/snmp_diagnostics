# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""A module that can process SNMP OIDs into ROS diagnostics."""

__all__ = ['SnmpDiagModule']


from pysnmp.entity.rfc3413 import context


class SnmpDiagModule(object):
    """A module that can process SNMP OIDs into ROS diagnostics."""

    def __init__(self, engine):
        """
        :param SnmpEngine engine: The SNMP engine instance. 
        """
        self.engine = engine
        self.mib_builder = context.SnmpContext(self.engine).getMibInstrum().getMibBuilder()

    def get_diag_name(self):
        """The name that will be shown in the top-level diagnostics item.
        :return: The name.
        :rtype: str
        """
        raise NotImplementedError()

    def register(self, updater):
        """Register this module with the diagnostic updater.
        :param diagnostic_updater.Updater updater: The updater instance.
        """
        updater.add(self.get_diag_name(), self.produce_diagnostics)

    def get_oids(self):
        """Get the list of OIDs that should be queried for this module.
        :return: The list of OIDs.
        :rtype: list of pysnmp.smi.rfc1902.ObjectType
        """
        raise NotImplementedError()

    def parse_response(self, response_iterator):
        """Parse the SNMP response iterator into something this module understands.
        :param iterator response_iterator: The SNMP response iterator.
        :return: The processed response data.
        :rtype: any
        """
        raise NotImplementedError()

    def process_response(self, response):
        """Process the response of SNMP agent and save the result in the module for later use by
        :meth:`produce_diagnostics`.
        :param any response: The SNMP response processed by :meth:`parse_response`.
        """
        raise NotImplementedError()

    def produce_diagnostics(self, diagnostics):
        """Produce diagnostics from the data stored by :meth:`process_response`.
        :param diagnostic_updater.DiagnosticStatusWrapper diagnostics: The diagnostics wrapper to update.
        """
        raise NotImplementedError()
