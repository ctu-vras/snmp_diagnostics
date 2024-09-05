# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

import importlib
import traceback

import rospkg
import rospy

from snmp_diagnostics import SnmpDiagPlugin


__all__ = ['PluginManager']


class PluginManager(object):
    def __init__(self, engine, config):
        self._plugins = dict()

        self._load_plugins(engine, config)

    @property
    def plugins(self):
        """Get the loaded plugins.

        :return: The loaded plugins.
        :rtype: list of SnmpDiagPlugin
        """
        return self._plugins.values()

    @staticmethod
    def _instantiate_plugin(p_class, engine, config):
        try:
            # import the specified plugin module
            p_pkg, p_classname = p_class.rsplit(".", 1)
            mod = importlib.import_module(p_pkg)
            try:
                p_class_obj = getattr(mod, p_classname)
            except AttributeError:
                rospy.logerr("Cannot load plugin [%s]: Class '%s' not found in module '%s'.",
                             p_class, p_classname, p_pkg)
                return None

            try:
                p_instance = p_class_obj(engine, config)
            except Exception as e:
                rospy.logerr(
                    "Cannot load plugin [%s]: Exception while calling __init__(engine, config): %s", p_class, e)
                traceback.print_exc()
                return None

            if not isinstance(p_instance, SnmpDiagPlugin):
                rospy.logerr(
                    "Cannot load plugin [%s]: The instantiated class is not an instance of SnmpDiagPlugin.", p_class)
                return None

            return p_instance
        except Exception as e:
            rospy.logerr("Cannot load plugin [%s]: %s", p_class, e)
            traceback.print_exc()
            return None

    def _load_plugins(self, engine, config):
        rospack = rospkg.RosPack()
        to_check = rospack.get_depends_on("snmp_diagnostics", implicit=False)
        to_check.append("snmp_diagnostics")

        for pkg in to_check:
            m = rospack.get_manifest(pkg)
            for export in m.exports:
                if export.tag != "snmp_diagnostics":
                    continue
                if export.get("class") is None or len(export.get("class")) == 0:
                    continue
                p_class = export.get("class")

                if "." not in p_class:
                    rospy.logerr("Cannot load plugin [%s]: 'class' has to be in format 'package.Class'.", p_class)

                if export.get("config_key") is None or len(export.get("config_key")) == 0:
                    rospy.logdebug("Skipping plugin %s because it has no config_key", p_class)
                    continue
                p_config_key = export.get("config_key")

                if p_config_key not in config:
                    rospy.logdebug("Skipping plugin %s as it is not configured.", p_class)
                    continue

                rospy.logdebug("Trying to load plugin %s", p_class)

                p_instance = self._instantiate_plugin(p_class, engine, config)
                if p_instance is not None:
                    self._plugins[p_config_key] = p_instance
                    rospy.loginfo("Loaded plugin %s = %s" % (p_config_key, p_class))
