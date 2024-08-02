# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague


class SnmpDiagModule(object):

    def get_diag_name(self):
        raise NotImplementedError()

    def register(self, updater):
        updater.add(self.get_diag_name(), self.produce_diagnostics)

    def get_oids(self):
        raise NotImplementedError()

    def parse_response(self, response_iterator):
        raise NotImplementedError()

    def process_response(self, response):
        raise NotImplementedError()

    def produce_diagnostics(self, diagnostics):
        raise NotImplementedError()
