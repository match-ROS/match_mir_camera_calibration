import os

os.environ['QT_QPA_PLATFORM'] = 'offscreen'
import pytest
pytest.importorskip('rclpy')
from PyQt5 import QtCore, QtGui, QtWidgets
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
        module.top_view.config = config
        module.top_view.status = data
        canvas = QtGui.QPixmap(640, 480)
        module.top_view.render(canvas)
        assert not canvas.isNull()
    finally:
        module.on_shutdown()
        context.window.close()
