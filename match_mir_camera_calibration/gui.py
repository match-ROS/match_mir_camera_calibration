"""Independent MurBaseGui application; heartbeat depends on the GUI event loop."""
import json
import math
import os
from pathlib import Path
import queue
import shlex
import signal
import tempfile
import time
import uuid

import cv2
import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
import yaml

from match_mur_gui.base_gui import MurBaseGui, MurGuiModule, setup_prefix

from .config import validate
from .controller import ACTIVE
from .session_node import PREFIX
from .qr import qr_image


class RosMonitor(QtCore.QThread):
    status = QtCore.pyqtSignal(object)
    preview = QtCore.pyqtSignal(str, object)
    result = QtCore.pyqtSignal(str, bool, str)
    raw = QtCore.pyqtSignal(str, object)

    def __init__(self, owner):
        super().__init__()
        self.owner = owner
        self.requests = queue.Queue()
        self.finished = False
        self.last_pulse = -math.inf

    def pulse(self):
        self.last_pulse = time.monotonic()

    def request(self, name):
        self.requests.put(('service', name))

    def edit(self, waypoints):
        self.requests.put(('edit', waypoints))

    def shutdown(self):
        self.last_pulse = -math.inf
        self.finished = True

    def run(self):
        context = rclpy.context.Context()
        node, executor = None, None
        pending = []
        try:
            rclpy.init(args=[], context=context)
            node = rclpy.create_node('mir_calibration_gui_monitor', context=context)
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            def status(msg):
                try:
                    data = json.loads(msg.data)
                    if data.get('owner_token') != self.owner:
                        self.result.emit('ownership', False, 'Another calibration session is running; stop it before launching this GUI session')
                        return
                    self.status.emit(data)
                except ValueError:
                    self.result.emit('status', False, 'Invalid session status JSON')
            node.create_subscription(String, PREFIX+'/status', status, qos)
            bridge = CvBridge()
            def image(side, msg):
                try:
                    rgb = np.ascontiguousarray(bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8'))
                    qimage = QtGui.QImage(rgb.data, rgb.shape[1], rgb.shape[0], rgb.strides[0], QtGui.QImage.Format_RGB888).copy()
                    self.preview.emit(side, qimage)
                except Exception as exc:
                    self.result.emit('preview', False, str(exc))
            for side in ('left', 'right'):
                node.create_subscription(Image, PREFIX+f'/preview/{side}', lambda msg, s=side: image(s, msg), qos_profile_sensor_data)
            for robot in ('mur620a', 'mur620b', 'mur620c', 'mur620d'):
                node.create_subscription(PoseStamped, f'/qualisys/{robot}/pose',
                                         lambda msg, r=robot: self.raw.emit(r, msg), 10)
            heartbeat = node.create_publisher(String, PREFIX+'/heartbeat', 1)
            edits = node.create_publisher(String, PREFIX+'/waypoints', 1)
            clients = {name: node.create_client(Trigger, PREFIX+'/'+name)
                       for name in ('prepare', 'start', 'pause', 'stop', 'capture', 'verify')}
            last_heartbeat = 0.0
            while context.ok() and not self.finished:
                executor.spin_once(timeout_sec=0.02)
                now = time.monotonic()
                if now-self.last_pulse < 0.4 and now-last_heartbeat > 0.15:
                    heartbeat.publish(String(data=self.owner))
                    last_heartbeat = now
                while not self.requests.empty():
                    kind, data = self.requests.get_nowait()
                    if kind == 'edit':
                        edits.publish(String(data=json.dumps(data, allow_nan=False)))
                    elif clients[data].service_is_ready():
                        future = clients[data].call_async(Trigger.Request())
                        pending.append((data, future, now+5.0))
                    else:
                        self.result.emit(data, False, 'Session service is unavailable; launch backend first')
                for name, future, deadline in list(pending):
                    if future.done():
                        try:
                            reply = future.result()
                            self.result.emit(name, bool(reply.success), reply.message)
                        except Exception as exc:
                            self.result.emit(name, False, str(exc))
                        pending.remove((name, future, deadline))
                    elif now > deadline:
                        future.cancel()
                        self.result.emit(name, False, 'Session service timed out')
                        pending.remove((name, future, deadline))
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        except Exception as exc:
            self.result.emit('ROS', False, str(exc))
        finally:
            if executor:
                executor.shutdown()
            if node:
                node.destroy_node()
            if context.ok():
                rclpy.shutdown(context=context)


class TopView(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setMinimumSize(320, 260)
        self.config, self.status = None, {}

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.fillRect(self.rect(), QtGui.QColor('#20252b'))
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        if not self.config or any(value is None for value in self.config['bounds'].values()) or any(
                self.config[key] is None for key in ('target_radius_m', 'observer_radius_m')):
            painter.setPen(QtCore.Qt.white)
            painter.drawText(self.rect(), QtCore.Qt.AlignCenter, 'Manuelle Aufnahme: Fahrt mit Joystick\nBereichsdraufsicht optional')
            return
        bounds = self.config['bounds']
        lo = np.array([bounds['x_min'], bounds['y_min']])
        hi = np.array([bounds['x_max'], bounds['y_max']])
        scale = min((self.width()-40)/(hi[0]-lo[0]), (self.height()-40)/(hi[1]-lo[1]))
        def point(xy):
            return QtCore.QPointF(20+(xy[0]-lo[0])*scale, self.height()-20-(xy[1]-lo[1])*scale)
        p1, p2 = point(lo), point(hi)
        painter.setPen(QtGui.QPen(QtGui.QColor('#dddddd'), 2))
        painter.drawRect(QtCore.QRectF(p1, p2).normalized())
        observer = self.status.get('mocap', {}).get(self.config['observer_robot'], {}).get('pose')
        if observer:
            from .planner import Planner, PlanningError
            try:
                planner = Planner(self.config, observer[:2])
                painter.setPen(QtGui.QPen(QtGui.QColor('#66bbcc'), 1, QtCore.Qt.DashLine))
                painter.drawRect(QtCore.QRectF(point(planner.lower), point(planner.upper)).normalized())
                painter.setPen(QtGui.QPen(QtGui.QColor('#ec7777'), 1, QtCore.Qt.DashLine))
                radius = planner.exclusion*scale
                painter.drawEllipse(point(observer[:2]), radius, radius)
            except PlanningError:
                pass
        for index, item in enumerate(self.status.get('plan', [])):
            painter.setPen(QtGui.QPen(QtGui.QColor('#689bce'), 1))
            path = item['path']
            for a, b in zip(path, path[1:]):
                painter.drawLine(point(a), point(b))
            x, y, angle = item['pose']
            painter.setPen(QtGui.QPen(QtGui.QColor('#f0c15a'), 2))
            p = point([x, y])
            painter.drawEllipse(p, 3, 3)
            painter.drawLine(p, point([x+0.12*math.cos(angle), y+0.12*math.sin(angle)]))
        for role, color in (('target', '#6ece82'), ('observer', '#f08080')):
            robot = self.config[role+'_robot']
            pose = self.status.get('mocap', {}).get(robot, {}).get('pose')
            if pose:
                painter.setPen(QtGui.QPen(QtGui.QColor(color), 2))
                center = point(pose[:2])
                radius = self.config[role+'_radius_m']*scale
                painter.drawEllipse(center, radius, radius)
                painter.drawText(center, robot)
        painter.setPen(QtCore.Qt.white)
        painter.drawText(8, 15, 'Qualisys mocap · x/y [m] · Kreise: komplette Roboterkontur')


class CalibrationModule(MurGuiModule):
    def setup_ui(self, context):
        self.context = context
        self.owner = uuid.uuid4().hex
        self.backend_config = None
        self.backend_status = {}
        self.status_at = -math.inf
        self.other_owner = False
        self.config_file = None
        self.plan_signature = None
        self.backend_plan_signature = None
        self.pending_edit = None
        self.last_dataset = None
        self._temp_paths = []
        self.raw_poses = {}
        self.panel = QtWidgets.QGroupBox('MiR Kamera- und Markerkalibrierung')
        panel_layout = QtWidgets.QVBoxLayout(self.panel)
        self.status_label = QtWidgets.QLabel('Backend nicht gestartet — keine Fahrbefehle')
        self.status_label.setWordWrap(True)
        panel_layout.addWidget(self.status_label)
        self.mocap_label = QtWidgets.QLabel('Mocap: warte auf rohe 6D-Posen')
        panel_layout.addWidget(self.mocap_label)
        self.tabs = QtWidgets.QTabWidget()
        panel_layout.addWidget(self.tabs)
        self.fields = {}
        self._setup_config_tab()
        self._setup_camera_tab()
        self._setup_plan_tab()
        self._setup_results_tab()
        buttons = QtWidgets.QGridLayout()
        panel_layout.addLayout(buttons)
        self.buttons = {}
        for index, (label, action) in enumerate([('Backend laden', 'load'), ('Plan erzeugen', 'prepare'),
                              ('Kurze Stoppprüfung', 'verify'),
                              ('Start / Fortsetzen', 'start'), ('Pause', 'pause'),
                              ('STOPP', 'stop'), ('Einzelaufnahme', 'capture'), ('iPhone-QR-Code', 'qr')]):
            button = QtWidgets.QPushButton(label)
            button.clicked.connect(lambda checked=False, a=action: self.action(a))
            buttons.addWidget(button, index//3, index % 3)
            self.buttons[action] = button
        self.buttons['stop'].setStyleSheet('background:#ad3333;color:white;font-weight:bold')
        self.buttons['qr'].setEnabled(False)
        context.add_panel(self.panel)
        self.monitor = RosMonitor(self.owner)
        self.monitor.status.connect(self._status)
        self.monitor.preview.connect(self._preview)
        self.monitor.result.connect(self._result)
        self.monitor.raw.connect(self._raw)
        self.monitor.start()
        self.timer = QtCore.QTimer(self.panel)
        self.timer.timeout.connect(self._pulse)
        self.timer.start(150)
        self._apply_template(self._template())
        self._base_defaults()

    def _base_defaults(self):
        window = self.context.window
        for robot, check in getattr(window, 'robot_checks', {}).items():
            check.setChecked(robot in ('mur620a', 'mur620d'))
        for name, checked in [('mir_enabled_check', True), ('mir_camera_check', True),
                              ('arm_r', False), ('arm_l', False), ('opt_integrated', False),
                              ('opt_moveit', False), ('opt_ft', False)]:
            widget = getattr(window, name, None)
            if widget:
                widget.setChecked(checked)

    def _template(self):
        from ament_index_python.packages import get_package_share_directory
        installed = Path(get_package_share_directory('match_mir_camera_calibration'))/'config/session.yaml'
        return yaml.safe_load(installed.read_text())

    def _setup_config_tab(self):
        tab = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(tab)
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        form_widget = QtWidgets.QWidget()
        self.form = QtWidgets.QFormLayout(form_widget)
        scroll.setWidget(form_widget)
        layout.addWidget(scroll)
        for key, label in [('target_robot', 'Fahrender MuR'), ('observer_robot', 'Kamera-MuR')]:
            combo = QtWidgets.QComboBox()
            combo.addItems(['mur620a', 'mur620b', 'mur620c', 'mur620d'])
            self.fields[key] = combo
            self.form.addRow(label, combo)
        dictionary = QtWidgets.QComboBox()
        dictionary.addItems(sorted(name for name in dir(cv2.aruco) if name.startswith('DICT_')))
        self.fields['dictionary'] = dictionary
        self.form.addRow('ArUco-Dictionary (Print prüfen)', dictionary)
        mode = QtWidgets.QComboBox()
        mode.addItems(['manual', 'automatic'])
        self.fields['acquisition_mode'] = mode
        self.form.addRow('Aufnahmemodus (manual = Joystick + Web)', mode)
        self.form.addRow(QtWidgets.QLabel('Manuell: Backend laden startet auch die iPhone-Webansicht auf Port 8080.\n'
                                          'Danach „iPhone-QR-Code“ drücken und mit dem iPhone scannen.'))
        for name, label in [('rear_left', 'Marker hinten links'), ('rear_right', 'Marker hinten rechts')]:
            row = QtWidgets.QWidget()
            box = QtWidgets.QHBoxLayout(row)
            box.setContentsMargins(0, 0, 0, 0)
            for sub, caption in [('id', 'ID'), ('length_m', 'schwarze Kante [m]')]:
                edit = QtWidgets.QLineEdit()
                edit.setPlaceholderText('Pflichtwert')
                self.fields[f'{name}.{sub}'] = edit
                box.addWidget(QtWidgets.QLabel(caption))
                box.addWidget(edit)
            self.form.addRow(label, row)
        anchor = QtWidgets.QComboBox()
        anchor.addItems(['rear_left', 'rear_right'])
        self.fields['anchor_marker'] = anchor
        self.form.addRow('Marker der Höhenreferenz', anchor)
        for key, label in [('anchor_z', 'Markermitte: z in base_link [m]'),
                           ('x_min', 'Bereich x_min [m]'), ('x_max', 'Bereich x_max [m]'),
                           ('y_min', 'Bereich y_min [m]'), ('y_max', 'Bereich y_max [m]'),
                           ('target_radius_m', 'Konturradius A inkl. Arme [m]'),
                           ('observer_radius_m', 'Konturradius B inkl. Arme [m]'),
                           ('yaw_offsets', 'Drehwinkel-Offsets [Grad, Kommas]')]:
            edit = QtWidgets.QLineEdit()
            edit.setPlaceholderText('Pflichtwert')
            self.fields[key] = edit
            self.form.addRow(label, edit)
        for key in ('nx', 'ny'):
            spin = QtWidgets.QSpinBox()
            spin.setRange(1, 12)
            self.fields[key] = spin
            self.form.addRow('Raster '+key, spin)
        for key, label in [('motion_enabled', 'Reale Fahrt freigeben'),
                           ('boundary_verified', 'Grenz- und Stoppprüfung am Aufbau durchgeführt'),
                           ('exclusive_control_confirmed', 'Keine andere aktive Fahrsteuerung'),
                           ('arms_stowed_confirmed', 'Arme abgestellt; Konturradien schließen alles ein')]:
            check = QtWidgets.QCheckBox(label)
            self.fields[key] = check
            self.form.addRow(check)
        row = QtWidgets.QHBoxLayout()
        for text, callback in [('YAML laden', self.load_config), ('YAML speichern', self.save_config)]:
            button = QtWidgets.QPushButton(text)
            button.clicked.connect(callback)
            row.addWidget(button)
        layout.addLayout(row)
        mocap_row = QtWidgets.QHBoxLayout()
        start_mocap = QtWidgets.QPushButton('Mocap-Bridge starten')
        start_mocap.clicked.connect(self.start_mocap)
        stop_mocap = QtWidgets.QPushButton('Eigene Mocap-Bridge stoppen')
        stop_mocap.clicked.connect(self.stop_mocap)
        mocap_row.addWidget(start_mocap)
        mocap_row.addWidget(stop_mocap)
        layout.addLayout(mocap_row)
        self.tabs.addTab(tab, 'Konfiguration')
        advanced = QtWidgets.QWidget()
        al = QtWidgets.QVBoxLayout(advanced)
        al.addWidget(QtWidgets.QLabel('Erweiterte Einstellungen: YAML bearbeiten und anschließend übernehmen.'))
        self.editor = QtWidgets.QPlainTextEdit()
        al.addWidget(self.editor)
        apply_button = QtWidgets.QPushButton('YAML in die Formularfelder übernehmen')
        apply_button.clicked.connect(self.apply_yaml)
        al.addWidget(apply_button)
        self.tabs.addTab(advanced, 'Erweitertes YAML')

    def _setup_camera_tab(self):
        tab = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(tab)
        self.camera_labels = {}
        self.camera_status = QtWidgets.QLabel('MiR-Kamera-Bridge auf B aktivieren; Backend laden für Live-Erkennung.')
        self.camera_status.setWordWrap(True)
        layout.addWidget(self.camera_status)
        for side in ('left', 'right'):
            label = QtWidgets.QLabel(f'Kamera {side}: warte auf Bild')
            label.setAlignment(QtCore.Qt.AlignCenter)
            label.setMinimumSize(280, 160)
            layout.addWidget(label)
            self.camera_labels[side] = label
        self.tabs.addTab(tab, 'Kameras')

    def _setup_plan_tab(self):
        tab = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(tab)
        self.top_view = TopView()
        layout.addWidget(self.top_view)
        self.table = QtWidgets.QTableWidget(0, 3)
        self.table.setHorizontalHeaderLabels(['x [m]', 'y [m]', 'yaw [Grad]'])
        self.table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        layout.addWidget(self.table)
        self.edit_button = QtWidgets.QPushButton('Editierte Messposen übernehmen und Wege neu prüfen')
        self.edit_button.clicked.connect(self.edit_waypoints)
        layout.addWidget(self.edit_button)
        remove_button = QtWidgets.QPushButton('Ausgewählte Messpose entfernen')
        remove_button.clicked.connect(lambda: self.table.removeRow(self.table.currentRow()) if self.table.currentRow() >= 0 else None)
        layout.addWidget(remove_button)
        self.tabs.addTab(tab, 'Fahrplan')

    def _setup_results_tab(self):
        tab = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(tab)
        self.dataset_edit = QtWidgets.QLineEdit()
        self.dataset_edit.setPlaceholderText('Messordner für Offline-Auswertung')
        layout.addWidget(self.dataset_edit)
        choose = QtWidgets.QPushButton('Messordner auswählen')
        choose.clicked.connect(self.choose_dataset)
        layout.addWidget(choose)
        self.solve_button = QtWidgets.QPushButton('Offline kalibrieren')
        self.solve_button.clicked.connect(self.solve)
        layout.addWidget(self.solve_button)
        self.result_text = QtWidgets.QPlainTextEdit()
        self.result_text.setReadOnly(True)
        layout.addWidget(self.result_text)
        self.tabs.addTab(tab, 'Auswertung')

    def _apply_template(self, config):
        self.editor.setPlainText(yaml.safe_dump(config, sort_keys=False))
        for key in ('target_robot', 'observer_robot', 'dictionary'):
            self.fields[key].setCurrentText(str(config[key]))
        self.fields['acquisition_mode'].setCurrentText(config.get('acquisition_mode', 'automatic'))
        for marker in ('rear_left', 'rear_right'):
            for sub in ('id', 'length_m'):
                v = config['markers'][marker][sub]
                self.fields[f'{marker}.{sub}'].setText('' if v is None else str(v))
        self.fields['anchor_marker'].setCurrentText(config['height_anchor']['marker'])
        self.fields['anchor_z'].setText('' if config['height_anchor']['z_m'] is None else str(config['height_anchor']['z_m']))
        for key in ('x_min', 'x_max', 'y_min', 'y_max'):
            v = config['bounds'][key]
            self.fields[key].setText('' if v is None else str(v))
        for key in ('target_radius_m', 'observer_radius_m'):
            self.fields[key].setText('' if config[key] is None else str(config[key]))
        self.fields['yaw_offsets'].setText(', '.join(str(v) for v in config['grid']['yaw_offsets_deg']))
        for key in ('nx', 'ny'):
            self.fields[key].setValue(config['grid'][key])
        for key in ('motion_enabled', 'boundary_verified', 'exclusive_control_confirmed', 'arms_stowed_confirmed'):
            self.fields[key].setChecked(bool(config[key]))

    def _config(self):
        config = yaml.safe_load(self.editor.toPlainText())
        for key in ('target_robot', 'observer_robot', 'dictionary'):
            config[key] = self.fields[key].currentText()
        config['acquisition_mode'] = self.fields['acquisition_mode'].currentText()
        for marker in ('rear_left', 'rear_right'):
            config['markers'][marker]['id'] = int(self.fields[f'{marker}.id'].text())
            config['markers'][marker]['length_m'] = float(self.fields[f'{marker}.length_m'].text())
        config['height_anchor'] = {'marker': self.fields['anchor_marker'].currentText(), 'z_m': float(self.fields['anchor_z'].text())}
        for key in ('x_min', 'x_max', 'y_min', 'y_max'):
            value = self.fields[key].text().strip()
            config['bounds'][key] = float(value) if value else None
        for key in ('target_radius_m', 'observer_radius_m'):
            value = self.fields[key].text().strip()
            config[key] = float(value) if value else None
        for key in ('nx', 'ny'):
            config['grid'][key] = self.fields[key].value()
        config['grid']['yaw_offsets_deg'] = [float(v.strip()) for v in self.fields['yaw_offsets'].text().split(',')]
        for key in ('motion_enabled', 'boundary_verified', 'exclusive_control_confirmed', 'arms_stowed_confirmed'):
            config[key] = self.fields[key].isChecked()
        return validate(config)

    def _error(self, message):
        self.context.append_log('[calibration] '+str(message))
        QtWidgets.QMessageBox.warning(self.panel, 'Kalibrierung', str(message))

    def apply_yaml(self):
        try:
            self._apply_template(yaml.safe_load(self.editor.toPlainText()))
        except (ValueError, KeyError, TypeError, yaml.YAMLError) as exc:
            self._error(exc)

    def load_config(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self.panel, 'Konfiguration laden', '', 'YAML (*.yaml *.yml)')
        if path:
            try:
                self._apply_template(yaml.safe_load(Path(path).read_text()))
            except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as exc:
                self._error(exc)

    def save_config(self):
        try:
            config = self._config()
            path, _ = QtWidgets.QFileDialog.getSaveFileName(self.panel, 'Konfiguration speichern', 'session.yaml', 'YAML (*.yaml)')
            if path:
                Path(path).write_text(yaml.safe_dump(config, sort_keys=False))
        except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as exc:
            self._error(exc)

    def _temporary_config(self, config):
        with tempfile.NamedTemporaryFile('w', suffix='.yaml', prefix='mir_calibration_', delete=False) as f:
            yaml.safe_dump(config, f, sort_keys=False)
            path = f.name
        self._temp_paths.append(path)
        return path

    def action(self, action):
        if action == 'qr':
            self.show_qr()
            return
        if action == 'load':
            try:
                if self.other_owner:
                    raise ValueError('Another session owns these ROS services; stop it first')
                if self.backend_status.get('state') in ACTIVE:
                    raise ValueError('Stop the active session before loading a configuration')
                config = self._config()
                path = self._temporary_config(config)
                process = self.context.window.processes.get('mir_calibration_session')
                if process and process.state() != QtCore.QProcess.NotRunning:
                    self.monitor.request('stop')
                    process.terminate()
                    if not process.waitForFinished(1500):
                        raise ValueError('Backend did not stop; configuration reload cancelled')
                self.backend_status, self.plan_signature, self.backend_plan_signature = {}, None, None
                self.buttons['qr'].setEnabled(False)
                self.pending_edit = None
                self.backend_config = config
                self.top_view.config = config
                if config['acquisition_mode'] == 'manual':
                    command = setup_prefix() + 'exec python3 -m match_mir_camera_calibration.web ' + \
                        '--config '+shlex.quote(path)+' --owner-token '+shlex.quote(self.owner)
                else:
                    command = setup_prefix() + 'exec python3 -m match_mir_camera_calibration.session_node --ros-args ' + \
                        '-p config_path:='+shlex.quote(path)+' -p owner_token:='+shlex.quote(self.owner)
                self.context.start_process('mir_calibration_session', command)
                self.status_label.setText('Backend startet; manuell: danach „iPhone-QR-Code“ drücken. '
                                          'Automatisch: Plan erzeugen und prüfen.')
            except (ValueError, OSError, KeyError, TypeError, yaml.YAMLError) as exc:
                self._error(exc)
            return
        if action in ('start', 'capture', 'verify'):
            try:
                if self.pending_edit is not None:
                    raise ValueError('Wait for the backend to accept and preview the edited plan')
                if self._config() != self.backend_config:
                    raise ValueError('Configuration changed; load the backend again before capture/motion')
                if action in ('start', 'verify') and self._table_waypoints() != self.plan_signature:
                    raise ValueError('Edited waypoints have not been accepted by the backend; apply the edit first')
                if action in ('start', 'verify'):
                    for dialog in self.context.window.findChildren(QtWidgets.QDialog):
                        dialog.close()
            except (ValueError, KeyError, TypeError, yaml.YAMLError) as exc:
                self._error(exc)
                return
        self.monitor.request(action)

    def start_mocap(self):
        if any(time.monotonic()-at < 0.5 for at, _ in self.raw_poses.values()):
            self.context.append_log('[calibration] Raw mocap is already live; using existing bridge')
            return
        from ament_index_python.packages import get_package_prefix
        executable = Path(get_package_prefix('match_mocap_ros2'))/'lib/match_mocap_ros2/qualisys_ssh_bridge'
        self.context.start_process('mir_calibration_mocap', setup_prefix()+'exec '+shlex.quote(str(executable))+
                                   ' --ros-args -p publish_map_pose:=false -p publish_robot_tf:=false')

    def stop_mocap(self):
        self.stop_motion_like_actions()
        process = self.context.window.processes.get('mir_calibration_mocap')
        if process and process.state() != QtCore.QProcess.NotRunning:
            process.terminate()

    def _table_waypoints(self):
        poses = []
        for row in range(self.table.rowCount()):
            poses.append([float(self.table.item(row, 0).text()), float(self.table.item(row, 1).text()),
                          math.radians(float(self.table.item(row, 2).text()))])
        return poses

    def edit_waypoints(self):
        try:
            waypoints = self._table_waypoints()
            self.pending_edit = [list(p[:2])+[math.atan2(math.sin(p[2]), math.cos(p[2]))] for p in waypoints]
            self.plan_signature = None
            self.backend_plan_signature = None
            self.monitor.edit(waypoints)
        except (ValueError, AttributeError) as exc:
            self._error(exc)

    def _status(self, data):
        self.backend_status, self.status_at = data, time.monotonic()
        self.buttons['qr'].setEnabled(bool(data.get('web_url')))
        progress = (f"Gespeicherte Messposen: {data.get('measurements_saved', 0)}"
                    if data.get('acquisition_mode') == 'manual' else
                    f"Messpose {data['index']+1}/{len(data['plan'])}")
        self.status_label.setText(f"{data['state']} · {data['reason']} · {progress} · Bilder L/R {data['counts']}")
        signature = [item['pose'] for item in data['plan']]
        if self.pending_edit is not None and len(signature) == len(self.pending_edit) and np.allclose(signature, self.pending_edit, rtol=0, atol=1e-8):
            self.pending_edit = None
        if signature != self.backend_plan_signature and data['state'] == 'READY' and self.pending_edit is None:
            self.table.setRowCount(len(signature))
            for row, pose in enumerate(signature):
                for col, v in enumerate([pose[0], pose[1], math.degrees(pose[2])]):
                    self.table.setItem(row, col, QtWidgets.QTableWidgetItem(f'{v:.10f}'))
            # Compare against the displayed numerical representation, avoiding
            # false edit detection from radians/degree formatting roundoff.
            self.plan_signature = self._table_waypoints()
            self.backend_plan_signature = signature
        self.top_view.status = data
        self.top_view.update()
        if data.get('dataset') and data['dataset'] != self.last_dataset:
            self.last_dataset = data['dataset']
            self.dataset_edit.setText(self.last_dataset)
        self.camera_status.setText(' · '.join(f"{side}: Alter {stats['age_sec']:.3f}s, Marker {data['detections'].get(side, [])}"
                                             for side, stats in data['cameras'].items()))
        active = data['state'] in ACTIVE
        manual = data.get('acquisition_mode') == 'manual'
        self.table.setEnabled(not active)
        self.edit_button.setEnabled(not active)
        self.solve_button.setEnabled(not active)
        for action in ('load', 'prepare', 'capture'):
            self.buttons[action].setEnabled(not active)
        self.buttons['prepare'].setEnabled(not active and not manual)
        self.edit_button.setEnabled(not active and not manual)
        self.buttons['start'].setEnabled(not manual and data['state'] in ('READY', 'PAUSED') and self.pending_edit is None)
        self.buttons['pause'].setEnabled(active)
        self.buttons['verify'].setEnabled(not manual and data['state'] == 'READY' and self.pending_edit is None)
        # Prevent the inherited GUI from introducing a second motion command.
        for button in self.context.window.findChildren(QtWidgets.QPushButton):
            if self.panel.isAncestorOf(button):
                continue
            if button.text() not in ('Stop Managed Processes', 'Save GUI Log'):
                if active and not manual and button.isEnabled():
                    button.setProperty('calibration_locked', True)
                    button.setEnabled(False)
                elif not active and button.property('calibration_locked'):
                    button.setEnabled(True)
                    button.setProperty('calibration_locked', False)

    def _preview(self, side, image):
        label = self.camera_labels[side]
        label.setPixmap(QtGui.QPixmap.fromImage(image).scaled(label.size(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation))

    def _raw(self, robot, msg):
        if msg.header.frame_id == 'mocap':
            self.raw_poses[robot] = (time.monotonic(), msg)

    def _pulse(self):
        self.monitor.pulse()
        manual = self.fields['acquisition_mode'].currentText() == 'manual'
        self.tabs.setTabEnabled(2, not manual)
        if manual:
            for action in ('prepare', 'start', 'verify'):
                self.buttons[action].setEnabled(False)
            self.edit_button.setEnabled(False)
        robots = [self.fields[key].currentText() for key in ('target_robot', 'observer_robot')]
        now = time.monotonic()
        self.mocap_label.setText('Mocap: '+' · '.join(f'{r}: '+('live' if r in self.raw_poses and now-self.raw_poses[r][0] < 0.2 else 'fehlt/veraltet') for r in robots))
        if self.backend_status and now-self.status_at > 1.0:
            self.status_label.setText('Backend-Status fehlt — Sitzung prüfen, keine automatische Wiederaufnahme')
            self.buttons['start'].setEnabled(False)
            self.buttons['qr'].setEnabled(False)

    def show_qr(self):
        url = self.backend_status.get('web_url')
        if not url or time.monotonic()-self.status_at > 1.:
            self._error('Zuerst das manuelle Backend laden und auf dessen Status warten.')
            return
        pixels = np.ascontiguousarray(qr_image(url))
        image = QtGui.QImage(pixels.data, pixels.shape[1], pixels.shape[0], pixels.strides[0],
                             QtGui.QImage.Format_Grayscale8).copy()
        dialog = QtWidgets.QDialog(self.panel)
        dialog.setWindowTitle('iPhone-Webansicht: QR-Code scannen')
        layout = QtWidgets.QVBoxLayout(dialog)
        layout.addWidget(QtWidgets.QLabel('Mit der iPhone-Kamera scannen und den Link öffnen.'))
        label = QtWidgets.QLabel()
        label.setPixmap(QtGui.QPixmap.fromImage(image))
        label.setAlignment(QtCore.Qt.AlignCenter)
        layout.addWidget(label)
        link = QtWidgets.QLabel(url)
        link.setWordWrap(True)
        link.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        layout.addWidget(link)
        close = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Close)
        close.rejected.connect(dialog.reject)
        layout.addWidget(close)
        dialog.exec_()
        dialog.deleteLater()

    def _result(self, action, success, message):
        self.context.append_log(f'[calibration] {action}: {success} — {message}')
        if action == 'ownership':
            self.other_owner = True
            self.buttons['load'].setEnabled(False)
        if not success:
            self.status_label.setText(message)

    def choose_dataset(self):
        path = QtWidgets.QFileDialog.getExistingDirectory(self.panel, 'Messordner auswählen')
        if path:
            self.dataset_edit.setText(path)

    def solve(self):
        if self.backend_status.get('state') in ACTIVE:
            self._error('Stop or finish the measurement before fitting')
            return
        root = Path(self.dataset_edit.text()).expanduser()
        if not (root/'manifest.json').is_file():
            self._error('Select a recorded measurement directory')
            return
        try:
            current = yaml.safe_load(self.editor.toPlainText())
            guess_path = self._temporary_config(current.get('camera_initial_guesses', {}))
            command = setup_prefix()+'exec python3 -m match_mir_camera_calibration.calibration '+shlex.quote(str(root))+' --initial-guesses '+shlex.quote(guess_path)
            def finished(code, process_status):
                report = root/'quality_report.json'
                if report.exists():
                    prefix = '' if code == 0 else f'Auswertung fehlgeschlagen (Exit {code}). Bericht:\n'
                    self.result_text.setPlainText(prefix+report.read_text())
                else:
                    self.result_text.setPlainText(f'Keine Auswertung geschrieben (Exit {code}); Prozesslog prüfen.')
                self.solve_button.setEnabled(True)
            self.solve_button.setEnabled(False)
            self.context.start_process('mir_calibration_solver', command, on_finished=finished)
        except (OSError, ValueError, TypeError, yaml.YAMLError) as exc:
            self._error(exc)

    def stop_motion_like_actions(self):
        if hasattr(self, 'monitor'):
            self.monitor.request('stop')

    def on_shutdown(self):
        self.timer.stop()
        self.monitor.shutdown()
        self.monitor.wait(2000)
        for path in self._temp_paths:
            Path(path).unlink(missing_ok=True)


def main():
    os.environ.setdefault('ROS_DOMAIN_ID', '62')
    # Override the shared GUI's C-only default for this two-robot application.
    os.environ.setdefault('ROS_STATIC_PEERS', 'mur620a;mur620d')
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    app = QtWidgets.QApplication([])
    window = MurBaseGui(modules=[CalibrationModule()], window_title='MuR MiR Camera Calibration')
    window.show()
    raise SystemExit(app.exec_())


if __name__ == '__main__':
    main()
