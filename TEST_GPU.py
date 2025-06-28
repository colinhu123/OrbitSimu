from PyQt5.QtWidgets import QApplication, QOpenGLWidget
from OpenGL.GL import glGetString, GL_RENDERER, GL_VENDOR, GL_VERSION
import sys

class InfoWidget(QOpenGLWidget):
    def initializeGL(self):
        renderer = glGetString(GL_RENDERER)
        vendor = glGetString(GL_VENDOR)
        version = glGetString(GL_VERSION)

        if renderer is None:
            print("❌ Failed to create OpenGL context!")
        else:
            print("✅ OpenGL Context Created")
            print("Renderer:", renderer.decode())
            print("Vendor:  ", vendor.decode())
            print("Version: ", version.decode())

        # Close after info is printed
        QApplication.instance().quit()

app = QApplication(sys.argv)
w = InfoWidget()
w.resize(200, 200)
w.show()
app.exec_()
