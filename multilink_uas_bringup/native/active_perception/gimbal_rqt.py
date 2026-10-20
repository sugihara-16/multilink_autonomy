#!/usr/bin/env python3
"""Open standard rqt Reconfigure on the camera node after ROS discovery."""
import sys

from python_qt_binding.QtCore import QTimer
from rqt_gui.main import Main
from rqt_reconfigure.param_plugin import ParamPlugin
from rqt_reconfigure.param_widget import ParamWidget


original_restore = ParamWidget.restore_settings


def restore_camera(self, plugin_settings, instance_settings):
    original_restore(self, plugin_settings, instance_settings)
    # Humble's standard node selector scans once, before DDS discovery often
    # finishes. Retry on the Qt thread, then stop refreshing after selection.
    self._camera_discovery_timer = QTimer(self)

    def select_camera():
        selector = self._nodesel_widget
        selector._refresh_nodes()
        if '/camera_gimbal_manual' in selector.get_nodeitems():
            self.filter_lineedit.set_text('camera_gimbal_manual')
            self.sig_selected.emit('/camera_gimbal_manual', True)
            self._camera_discovery_timer.stop()

    self._camera_discovery_timer.timeout.connect(select_camera)
    self._camera_discovery_timer.start(500)


def main():
    # Process-local adaptation; installed rqt packages remain unchanged.
    ParamWidget.restore_settings = restore_camera
    return Main().main(sys.argv, standalone='rqt_reconfigure',
                       plugin_argument_provider=ParamPlugin.add_arguments)


if __name__ == '__main__':
    sys.exit(main())
