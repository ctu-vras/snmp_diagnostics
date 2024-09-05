# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""A plugin that can process SNMP OIDs into ROS diagnostics."""

__all__ = ['SnmpDiagPlugin']


from pysnmp.entity.rfc3413 import context


class SnmpDiagPlugin(object):
    """A plugin that can process SNMP OIDs into ROS diagnostics."""

    def __init__(self, engine, config):
        """
        :param pysnmp.hlapi.v3arch.SnmpEngine engine: The SNMP engine instance.
        :param dict config: The configuration dictionary.
        """
        self.engine = engine
        self.config = config
        self.mib_builder = context.SnmpContext(self.engine).getMibInstrum().getMibBuilder()

    def get_diag_name(self):
        """The name that will be shown in the top-level diagnostics item.

        :return: The name.
        :rtype: str
        """
        raise NotImplementedError()

    def register(self, updater):
        """Register this plugin with the diagnostic updater.

        :param diagnostic_updater.Updater updater: The updater instance.
        """
        updater.add(self.get_diag_name(), self.produce_diagnostics)

    def get_oids(self):
        """Get the list of OIDs that should be queried for this plugin.

        :return: The list of OIDs.
        :rtype: list of :class:`pysnmp.smi.rfc1902.ObjectType`
        """
        raise NotImplementedError()

    def parse_response(self, response_iterator):
        """Parse the SNMP response iterator into something this plugin understands.

        :param iterator response_iterator: The SNMP response iterator.
        :return: The processed response data.
        :rtype: any
        """
        raise NotImplementedError()

    def process_response(self, response):
        """Process the response of SNMP agent and save the result in the plugin for later use by
        :meth:`produce_diagnostics`.

        :param any response: The SNMP response processed by :meth:`parse_response`.
        """
        raise NotImplementedError()

    def produce_diagnostics(self, diagnostics):
        """Produce diagnostics from the data stored by :meth:`process_response`.

        :param diagnostic_updater.DiagnosticStatusWrapper diagnostics: The diagnostics wrapper to update.
        """
        raise NotImplementedError()
