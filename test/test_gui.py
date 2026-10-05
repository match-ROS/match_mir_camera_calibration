import os

os.environ['QT_QPA_PLATFORM'] = 'offscreen'
import pytest
pytest.importorskip('rclpy')
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtTest import QTest
from match_mir_camera_calibration import gui


class FakeMonitor(QtCore.QObject):
    status = QtCore.pyqtSignal(object)
    preview = QtCore.pyqtSignal(str, object)
    result = QtCore.pyqtSignal(str, bool, str)
    raw = QtCore.pyqtSignal(str, object)

    def __init__(self, owner):
        super().__init__()
    def start(self): pass
    def pulse(self): pass
    def request(self, action): pass
    def edit(self, waypoints): self.edited = waypoints
    def shutdown(self): pass
    def wait(self, timeout): pass


class Context:
    def __init__(self):
        self.window = QtWidgets.QMainWindow()
        self.window.processes = {}
        self.central = QtWidgets.QWidget()
        self.window.setCentralWidget(self.central)
        self.layout = QtWidgets.QVBoxLayout(self.central)
        self.logs = []
    def append_log(self, text): self.logs.append(text)
    def add_panel(self, panel): self.layout.addWidget(panel)


def test_gui_form_roundtrip_preserves_plan_edits_and_renders(config, monkeypatch):
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    monkeypatch.setattr(gui, 'RosMonitor', FakeMonitor)
    monkeypatch.setattr(gui.CalibrationModule, '_template', lambda self: config)
    context = Context()
    module = gui.CalibrationModule()
    module.setup_ui(context)
    try:
        assert module._config() == config
        module.backend_config = config
        data = {'state': 'READY', 'reason': 'test', 'index': 0, 'counts': {'left': 0, 'right': 0},
                'plan': [{'pose': [2., 0., .1], 'path': [[2., 0.], [2., 0.]]}],
                'dataset': None, 'mocap': {}, 'cameras': {}, 'detections': {}}
        module._status(data)
        module.table.item(0, 0).setText('2.2')
        module._status(data)
        assert module.table.item(0, 0).text() == '2.2'
        module.edit_waypoints()
        assert module.monitor.edited[0][0] == 2.2
        module._status(data)  # Old status must not acknowledge the queued edit.
        assert module.pending_edit is not None
        assert not module.buttons['start'].isEnabled()
        assert module.table.item(0, 0).text() == '2.2'
        accepted = dict(data, plan=[{'pose': module.monitor.edited[0], 'path': [[2., 0.], [2.2, 0.]]}])
        module._status(accepted)
        assert module.pending_edit is None
        assert module.buttons['start'].isEnabled()
        assert not module.buttons['qr'].isEnabled()
        accepted['web_url'] = 'http://10.145.8.71:8080/?token=test-phone'
        module._status(accepted)
        assert module.buttons['qr'].isEnabled()
        # The real modal dialog renders and closes; its nested event loop keeps
        # the GUI heartbeat timer responsive while the phone scans the code.
        QtCore.QTimer.singleShot(100, lambda: QtWidgets.QApplication.activeModalWidget().reject())
        module.show_qr()
        module.top_view.config = config
        module.top_view.status = data
        canvas = QtGui.QPixmap(640, 480)
        module.top_view.render(canvas)
        assert not canvas.isNull()
    finally:
        module.on_shutdown()
        context.window.close()


def test_real_base_gui_starts_and_closes_without_hardware(tmp_path, monkeypatch):
    from match_mur_gui import base_gui
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path/'ros_logs'))
    monkeypatch.setattr(base_gui, 'GUI_LOG_DIR', str(tmp_path/'gui_logs'))
    monkeypatch.setattr(base_gui, 'GUI_LATEST_LOG', str(tmp_path/'gui_logs/latest.log'))
    # Closing the shared base GUI normally performs remote process cleanup.
    # This test explicitly excludes all SSH and robot actions.
    monkeypatch.setattr(base_gui.MurBaseGui, 'stop_managed_processes',
                        lambda self: self.stop_module_motion_like_actions())
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    module = gui.CalibrationModule()
    window = base_gui.MurBaseGui(modules=[module], window_title='Calibration smoke test')
    try:
        window.show()
        QTest.qWait(250)
        assert window.selected_robots() == ['mur620a', 'mur620d']
        assert not window.arm_r.isChecked() and not window.arm_l.isChecked()
        assert window.mir_enabled_check.isChecked() and window.mir_camera_check.isChecked()
        assert not window.processes
        assert module.backend_config is None
        settings = module._config()  # Manual defaults work with blank area/contour fields.
        assert settings['acquisition_mode'] == 'manual' and settings['observer_robot'] == 'mur620d'
        assert settings['markers'] == {'front_left': {'id': 7, 'length_m': .16},
                                       'front_right': {'id': 24, 'length_m': .16}}
        assert settings['height_anchor'] == {'marker': 'front_left', 'z_m': .44}
        assert settings['bounds']['x_min'] is None
        module._pulse()
        assert not module.tabs.isTabEnabled(2)
        assert all(not module.buttons[action].isEnabled() for action in ('prepare', 'start', 'verify'))
        assert window.grab().save(str(tmp_path/'gui.png'))
    finally:
        window.close()
        app.processEvents()
