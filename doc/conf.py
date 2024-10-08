# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

# Configuration file for the Sphinx documentation builder.

import os
os.environ['CRAS_DOCS_COMMON_SPHINX_PACKAGE_PATH'] = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# include the master configuration
from cras_docs_common.sphinx_docs_conf import *

# TODO(peci1): Uncomment when https://github.com/ros/diagnostics/pull/399 is merged .
# intersphinx_mapping["diagnostic_updater"] = (ros_api + 'diagnostic_updater/html/python', None)
intersphinx_mapping["pysnmp"] = ('https://pysnmp.readthedocs.io/en/latest', None)

default_role = 'code'