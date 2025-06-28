import sys
import numpy as np
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
from PyQt5.QtWidgets import QOpenGLWidget
from OpenGL.GL import *
from OpenGL.GLU import *
from stl import mesh


class STLViewer(QOpenGLWidget):
    def __init__(self, stl_path, parent=None):
        super(STLViewer, self).__init__(parent)
        self.mesh = mesh.Mesh.from_file(stl_path)
        self.angle = 0.0
        self.vbo_vertices = None
        self.vbo_normals = None
        self.vertex_count = 0
        self.last_mouse_pos = None
        self.yaw = 0.0     # horizontal rotation
        self.pitch = 0.0   # vertical rotation
        self.zoom = 100.0
        self.sphere_quadric = None

    def initializeGL(self):
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glEnable(GL_COLOR_MATERIAL)
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)
        glClearColor(0.1, 0.1, 0.1, 1.0)

        light_position = [20.0, 20.0, 40.0, 1.0]
        glLightfv(GL_LIGHT0, GL_POSITION, light_position)
        glLightfv(GL_LIGHT0, GL_AMBIENT,  [0.3, 0.3, 0.3, 1.0])
        glLightfv(GL_LIGHT0, GL_DIFFUSE,  [1.5, 1.5, 1.5, 1.0])
        glLightfv(GL_LIGHT0, GL_SPECULAR, [2.0, 2.0, 2.0, 1.0])

        self.sphere_quadric = gluNewQuadric()

        self.setupBuffers()

    def mousePressEvent(self, event):
        self.last_mouse_pos = event.pos()

    def mouseMoveEvent(self, event):
        if self.last_mouse_pos is None:
            return

        dx = event.x() - self.last_mouse_pos.x()
        dy = event.y() - self.last_mouse_pos.y()

        self.yaw += dx * 0.5
        self.pitch += dy * 0.5
        self.pitch = max(-90, min(90, self.pitch))  # clamp vertical rotation

        self.last_mouse_pos = event.pos()
        self.update()

    def setupBuffers(self):
        from OpenGL.arrays import vbo

        vertices = self.mesh.vectors.reshape(-1, 3).astype(np.float32)
        normals = np.repeat(self.mesh.normals, 3, axis=0).astype(np.float32)
        self.vertex_count = len(vertices)

        self.vbo_vertices = vbo.VBO(vertices)
        self.vbo_normals = vbo.VBO(normals)

    def wheelEvent(self, event):
        delta = event.angleDelta().y() / 120  # one notch = 1
        self.zoom += delta * 5.0
        self.zoom = min(max(self.zoom, 10), 500)  # clamp zoom range
        self.update()

    def resizeGL(self, w, h):
        glViewport(0, 0, w, h)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45.0, float(w) / float(h), 1.0, 1e9)
        glMatrixMode(GL_MODELVIEW)

    def paintGL(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        glTranslatef(0.0, 0.0, -self.zoom)
        glRotatef(self.pitch, 1.0, 0.0, 0.0)   # up/down
        glRotatef(self.yaw,   0.0, 1.0, 0.0)   # left/right

        center = np.mean(self.mesh.vectors.reshape(-1, 3), axis=0)
        glTranslatef(-center[0], -center[1], -center[2])

        glEnableClientState(GL_VERTEX_ARRAY)
        glEnableClientState(GL_NORMAL_ARRAY)

        # Draw a sphere at some position
        glPushMatrix()
        glTranslatef(0.0, 0.0, -6.47e7)       # move far back
        glColor3f(0.2, 0.4, 1.0)             # blue color
        gluSphere(self.sphere_quadric, 6.37e7, 64, 64)  # big, smooth sphere
        glPopMatrix()


        self.vbo_vertices.bind()
        glVertexPointer(3, GL_FLOAT, 0, self.vbo_vertices)
        self.vbo_normals.bind()
        glNormalPointer(GL_FLOAT, 0, self.vbo_normals)

        glColor3f(0.8, 0.8, 1.0)
        glDrawArrays(GL_TRIANGLES, 0, self.vertex_count)

        self.vbo_vertices.unbind()
        self.vbo_normals.unbind()

        glDisableClientState(GL_VERTEX_ARRAY)
        glDisableClientState(GL_NORMAL_ARRAY)

        self.angle += 1.0
        self.update()

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("STL Viewer with PyQt5 + OpenGL")
        self.resize(1600, 1200)

        # Layout
        central_widget = QWidget()
        layout = QVBoxLayout(central_widget)
        self.viewer = STLViewer("Voyager.stl")
        layout.addWidget(self.viewer)
        self.setCentralWidget(central_widget)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
