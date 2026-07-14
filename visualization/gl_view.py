import numpy as np
from PyQt5.QtWidgets import QOpenGLWidget
from OpenGL.GL import *
from OpenGL.GLU import gluPerspective

import config
from visualization.stl_loader import load_stl, load_stl_filtered


class _Mesh:
    __slots__ = ("vbo_vertex", "vbo_normal", "vertex_count")

    def __init__(self, vertices: np.ndarray, normals: np.ndarray):
        self.vertex_count = vertices.shape[0]

        self.vbo_vertex = glGenBuffers(1)
        glBindBuffer(GL_ARRAY_BUFFER, self.vbo_vertex)
        glBufferData(GL_ARRAY_BUFFER, vertices.nbytes, vertices, GL_STATIC_DRAW)

        self.vbo_normal = glGenBuffers(1)
        glBindBuffer(GL_ARRAY_BUFFER, self.vbo_normal)
        glBufferData(GL_ARRAY_BUFFER, normals.nbytes, normals, GL_STATIC_DRAW)

        glBindBuffer(GL_ARRAY_BUFFER, 0)

    def draw(self, color):
        if not self.vertex_count:
            return
        glEnableClientState(GL_VERTEX_ARRAY)
        glEnableClientState(GL_NORMAL_ARRAY)

        glBindBuffer(GL_ARRAY_BUFFER, self.vbo_vertex)
        glVertexPointer(3, GL_FLOAT, 0, None)

        glBindBuffer(GL_ARRAY_BUFFER, self.vbo_normal)
        glNormalPointer(GL_FLOAT, 0, None)

        glColor3f(*color)
        glDrawArrays(GL_TRIANGLES, 0, self.vertex_count)

        glBindBuffer(GL_ARRAY_BUFFER, 0)
        glDisableClientState(GL_VERTEX_ARRAY)
        glDisableClientState(GL_NORMAL_ARRAY)


def _transform_point(raw_xyz, center_x, center_z, min_y, scale):
    x, y, z = raw_xyz
    return ((x - center_x) * scale, (y - min_y) * scale, (z - center_z) * scale)


class GLModelView(QOpenGLWidget):
    def __init__(self, parent=None):
        super().__init__(parent)

        self._fixed = None
        self._pan = None
        self._tilt = None

        self._pan_pivot = (0.0, 0.0, 0.0)
        self._tilt_pivot = (0.0, 0.0, 0.0)
        self._laser_origin = (0.0, 0.0, 0.0)

        self._azimuth_deg = 0.0
        self._elevation_deg = 0.0

        self._view_yaw = -35.0
        self._view_pitch = 20.0
        self._view_distance = config.GL_CAMERA_DISTANCE
        self._last_mouse_pos = None

        self.setMinimumHeight(120)

    def initializeGL(self):
        glClearColor(0.05, 0.06, 0.07, 1.0)
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glEnable(GL_COLOR_MATERIAL)
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)

        glLightfv(GL_LIGHT0, GL_POSITION, (3.0, 4.0, 5.0, 1.0))
        glLightfv(GL_LIGHT0, GL_AMBIENT, (0.25, 0.25, 0.28, 1.0))
        glLightfv(GL_LIGHT0, GL_DIFFUSE, (0.9, 0.9, 0.9, 1.0))

        glEnable(GL_NORMALIZE)
        glShadeModel(GL_SMOOTH)

        self._load_models()

    def _load_models(self):
        fv1, fn1 = load_stl(config.STL_FIXED_PATHS[0])
        fv2, fn2 = load_stl(config.STL_FIXED_PATHS[1])
        fixed_v = np.concatenate([fv1, fv2], axis=0)
        fixed_n = np.concatenate([fn1, fn2], axis=0)

        pan_v, pan_n = load_stl(config.STL_MOTOR_BRACKET_PAN)

        tv1, tn1 = load_stl(config.STL_MOTOR_BRACKET_TILT)
        tv2, tn2 = load_stl_filtered(
            config.STL_CAMERA_ASSEMBLY, y_min=config.CAMERA_ASSEMBLY_Y_CUTOFF
        )
        tilt_v = np.concatenate([tv1, tv2], axis=0)
        tilt_n = np.concatenate([tn1, tn2], axis=0)

        all_v = np.concatenate([fixed_v, pan_v, tilt_v], axis=0)
        mins = all_v.min(axis=0)
        maxs = all_v.max(axis=0)
        center_x = (mins[0] + maxs[0]) / 2.0
        center_z = (mins[2] + maxs[2]) / 2.0
        min_y = mins[1]
        total_height = maxs[1] - min_y
        scale = config.GL_TOTAL_HEIGHT / total_height if total_height > 0 else 1.0

        def transform_vertices(v):
            out = v.copy()
            out[:, 0] = (out[:, 0] - center_x) * scale
            out[:, 1] = (out[:, 1] - min_y) * scale
            out[:, 2] = (out[:, 2] - center_z) * scale
            return out.astype(np.float32)

        fixed_v = transform_vertices(fixed_v)
        pan_v = transform_vertices(pan_v)
        tilt_v = transform_vertices(tilt_v)

        self._fixed = _Mesh(fixed_v, fixed_n)
        self._pan = _Mesh(pan_v, pan_n)
        self._tilt = _Mesh(tilt_v, tilt_n)

        self._pan_pivot = _transform_point(config.PAN_PIVOT_RAW, center_x, center_z, min_y, scale)
        self._tilt_pivot = _transform_point(config.TILT_PIVOT_RAW, center_x, center_z, min_y, scale)
        self._laser_origin = _transform_point(config.LASER_ORIGIN_RAW, center_x, center_z, min_y, scale)

    def resizeGL(self, w, h):
        h = max(h, 1)
        glViewport(0, 0, w, h)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45.0, w / h, 0.1, 100.0)
        glMatrixMode(GL_MODELVIEW)

    def paintGL(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()

        glTranslatef(0.0, -config.GL_TOTAL_HEIGHT * 0.5, -self._view_distance)
        glRotatef(self._view_pitch, 1.0, 0.0, 0.0)
        glRotatef(self._view_yaw, 0.0, 1.0, 0.0)

        if self._fixed:
            glPushMatrix()
            self._fixed.draw(config.GL_FIXED_COLOR)
            glPopMatrix()

        glPushMatrix()
        px, py, pz = self._pan_pivot
        glTranslatef(px, py, pz)
        glRotatef(self._azimuth_deg, 0.0, 1.0, 0.0)
        glTranslatef(-px, -py, -pz)

        if self._pan:
            self._pan.draw(config.GL_PAN_COLOR)

        glPushMatrix()
        tx, ty, tz = self._tilt_pivot
        glTranslatef(tx, ty, tz)
        glRotatef(self._elevation_deg, 1.0, 0.0, 0.0)
        glTranslatef(-tx, -ty, -tz)

        if self._tilt:
            self._tilt.draw(config.GL_TILT_COLOR)

        self._draw_laser()

        glPopMatrix()  # TILT
        glPopMatrix()  # PAN

    def _draw_laser(self):

        ox, oy, oz = self._laser_origin
        length = config.GL_LASER_LENGTH

        glDisable(GL_LIGHTING)
        glDisable(GL_DEPTH_TEST)
        glLineWidth(config.GL_LASER_WIDTH)
        glColor3f(*config.GL_LASER_COLOR)

        glPushMatrix()
        glTranslatef(ox, oy, oz)
        glRotatef(config.GL_LASER_YAW_OFFSET_DEG, 0.0, 1.0, 0.0)
        glRotatef(config.GL_LASER_PITCH_OFFSET_DEG, 1.0, 0.0, 0.0)

        glBegin(GL_LINES)
        glVertex3f(0.0, 0.0, 0.0)
        glVertex3f(0.0, 0.0, length)
        glEnd()

        glPopMatrix()

        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)

    def set_orientation(self, azimuth_deg: float, elevation_deg: float):
        el_deg = max(config.ELEVATION_DEG_MIN, min(config.ELEVATION_DEG_MAX, elevation_deg))

        if azimuth_deg != self._azimuth_deg or el_deg != self._elevation_deg:
            self._azimuth_deg = azimuth_deg
            self._elevation_deg = el_deg
            self.update()

    def mousePressEvent(self, event):
        self._last_mouse_pos = event.pos()

    def mouseMoveEvent(self, event):
        if self._last_mouse_pos is None:
            return
        dx = event.pos().x() - self._last_mouse_pos.x()
        dy = event.pos().y() - self._last_mouse_pos.y()
        self._last_mouse_pos = event.pos()

        self._view_yaw += dx * 0.4
        self._view_pitch = max(-89.0, min(89.0, self._view_pitch + dy * 0.4))
        self.update()

    def mouseReleaseEvent(self, event):
        self._last_mouse_pos = None

    def wheelEvent(self, event):
        delta = event.angleDelta().y() / 120.0
        self._view_distance = max(1.0, min(30.0, self._view_distance - delta * 0.3))
        self.update()
