#!/usr/bin/env python3.7
import serial
from PySide2.QtCore import QEvent, QFile, QPluginLoader
from PySide2.QtGui import QFileOpenEvent, QHoverEvent
from pyqtgraph.Qt import QtCore, QtGui, QtWidgets
from PySide2.QtWidgets import QMainWindow, QApplication
from sideui2 import Ui_MainWindow
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from PySide2.QtUiTools import QUiLoader
import imageio_ffmpeg
import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl
import pyqtgraph.parametertree as ptree
import PySide2
import qimage2ndarray as q2a
import rospy
import cv2 as cv2
import sys
import time
import os
import serial
import random
import math

# try:
#    ser = serial.Serial(
#        port='/dev/ttyUSB1',
#        baudrate = 115200,
#        parity=serial.PARITY_NONE,
#        stopbits=serial.STOPBITS_ONE,
#        bytesize=serial.EIGHTBITS,
#        timeout=1
#    )
# except:
#    pass

use_cv_gui = False
if not use_cv_gui:
    ci_build_and_not_headless = False
    try:
        from cv2.version import ci_build, headless
        ci_and_not_headless = ci_build and not headless
    except:
        pass
    if sys.platform.startswith("linux") and ci_and_not_headless:
        os.environ.pop("QT_QPA_PLATFORM_PLUGIN_PATH")
    if sys.platform.startswith("linux") and ci_and_not_headless:
        os.environ.pop("QT_QPA_FONTDIR")

app = QApplication()


class CustomViewWidget(gl.GLViewWidget):
    def closeEvent(self, event):
        event.accept()


translate = QtCore.QCoreApplication.translate

param = ptree.Parameter.create(name=translate('ScatterPlot', 'Parameters'), type='group', children=[
    dict(name='pen', title='Pen:    ', type='list',
         values={'Grayscale': 'grayscale', 'Viridis': 'viridis'}, value='grayscale'),
    dict(name='axis', title='Axes:   ', type='bool', value=True),
    dict(name='circle', title='Circles:    ', type='bool', value=False),
])
for c in param.children():
    c.setDefault(c.value())
pt = ptree.ParameterTree(showHeader=False)
pt.setParameters(param)

w = CustomViewWidget(rotationMethod='quaternion')
q = QtGui.QQuaternion.fromEulerAngles(0, 0, 0)
w.setCameraPosition(pos=QtGui.QVector3D(0, 0, 0), distance=30, rotation=q)
w.setBackgroundColor(0, 0, 64)


label = QtWidgets.QLabel("")

splitter = QtWidgets.QSplitter()
splitter.setOrientation(QtCore.Qt.Orientation.Vertical)
splitter.addWidget(label)
splitter.addWidget(w)
splitter.resize(1000, 600)
label.setAlignment(QtCore.Qt.AlignHCenter | QtCore.Qt.AlignVCenter)


print("Init variables...")
theta_res = 200
radii_res = 397
count = 0
radi = []
goc = []
ad_low = 25*255/80
ad_span = 30*255/80
ad_max = 80
ad_high = ad_low + ad_span
value = np.zeros((radii_res,))
a = 0
b = 0
k = 0
new = 0
count = 0
temp = 0
final = []
bars = []
radi = []
img = []
goc = 0
t = 0
deltaT = 2*np.pi/(theta_res)
c = np.cos(deltaT)
s = np.sin(deltaT)
verts = np.zeros((theta_res+1, radii_res+1, 3))
radii_scale = np.linspace(0, 10, radii_res+1)
verts[0, :, 0] = radii_scale
count = 0


for i in range(theta_res-1):
    i_ = i+1
    verts[i_, :, 0] = verts[i, :, 0]*c-verts[i, :, 1]*s
    verts[i_, :, 1] = verts[i, :, 0]*s+verts[i, :, 1]*c
verts[theta_res, :, 0] = verts[0, :, 0]
verts[theta_res, :, 1] = verts[0, :, 1]
verts = verts.reshape(-1, 3, 1)
verts = verts.reshape(verts.shape[0], verts.shape[1])
faces = np.zeros((theta_res*radii_res*2, 3), dtype=int)
for i in range(theta_res):
    offset_v = i*(radii_res+1)
    offset_f = i*radii_res*2
    for j in range(radii_res):
        pos_v = offset_v+j
        pos_f = offset_f+j*2
        faces[pos_f, :] = [pos_v, pos_v+1, pos_v+2+radii_res]
        faces[pos_f+1, :] = [pos_v, pos_v+radii_res+1, pos_v+2+radii_res]
colors = np.zeros((theta_res*radii_res*2, 4))
colors[:, 3] = 1
m1 = gl.GLMeshItem(vertexes=verts, faces=faces,
                   faceColors=colors, smooth=False, computeNormals=False)
w.addItem(m1)

lpts = np.array([[0, 0, 0], [10, 0, 0]])
# lpts = np.vstack([x,yi,z]).transpose()
line = gl.GLLinePlotItem(pos=lpts, color=(255, 255, 0, 1), antialias=True)
w.addItem(line)

arrows_data = [gl.MeshData.cylinder(
    1, 10, radius=[0.2, 0], length=0.5) for i in range(2)]
arrows = [gl.GLMeshItem(meshdata=data, smooth=True, drawFaces=True, drawEdges=False, FaceColor=(
    1, 1, 1, 0.5), shader='balloon') for data in arrows_data]
for arrow in arrows:
    arrow.translate(0, 0, 11)
arrows[0].rotate(90, 0, 1, 0)
arrows[0].setColor([0, 0, 1, 1])
arrows[1].rotate(-90, 1, 0, 0)
arrows[1].setColor([1, 0, 0, 1])
for arrow in arrows:
    w.addItem(arrow)
pts = np.array([[0, 0, 0], [11, 0, 0]])
xax = gl.GLLinePlotItem(pos=pts, color=[0, 0, 1, 1], width=2)
pts = np.array([[0, 0, 0], [0, 11, 0]])
yax = gl.GLLinePlotItem(pos=pts, color=[1, 0, 0, 1], width=2)
w.addItem(xax)
w.addItem(yax)

axtheta = np.linspace(0, 2*np.pi, 100)
x = np.cos(axtheta)
y = np.sin(axtheta)
z = np.zeros(100)
pts = np.vstack([x, y, z]).transpose()
circles = [gl.GLLinePlotItem(pos=pts*i, color=[0, 1, 1, 1], width=1)
           for i in np.linspace(10, 0, 5, endpoint=False)]


for c in circles:
    print(c.pos[0][0])
    w.addItem(c)
count = 0
count1 = 0
intervals = 0
heading = 0
outvid = None
out_is_closed = False


def grab():
    global w, outvid, out_is_closed
    img = w.grabFramebuffer()
    if outvid is None:
        outvid = imageio_ffmpeg.write_frames('test4.mp4', (img.width(
        ), img.height()), pix_fmt_in="bgra", pix_fmt_out="yuv420p", fps=30)
        outvid.send(None)
        print(img.width(), ",", img.height())
    elif out_is_closed:
        return
    else:
        try:
            arr = q2a.byte_view(img, byteorder='little')
            outvid.send(arr)
        except:
            pass


def update_settings():
    xax.setVisible(param['axis'])
    yax.setVisible(param['axis'])
    for arrow in arrows:
        arrow.setVisible(param['axis'])
    for c in circles:
        c.setVisible(param['circle'])


def header(data):
    global heading
    heading = -data.data


def distance_callback(data):
    global intervals, count1, t, ser, heading, line
#    try:
#        heading = ser.readline().decode("utf-8").split(":").strip("yaw")
#    except:
#        heading = 0
    #w.setCameraPosition(pos=QtGui.QVector3D(0, 0, 0), distance=10, rotation=QtGui.QQuaternion.fromEulerAngles(0, 0, heading))
    m1.rotate(heading, 0, 0, 1)

    def update(data):
        global i, tim, intervals, scat, fig, ax, x, y, colors, count, value, color_Thien, grid_x, grid_y, final, temp, rho, theta, radi, goc, radii, goc, bars, radii_res, theta_res, init_flag, radius, img, bg, t, colors_new, line, heading
        tim = time.time()
        print(heading)
        max = 0
        for i in range(len(data.intensities)):
            temp = data.intensities[i]
            if max < temp:
                max = temp
                rmin = data.ranges[i]

            if temp <= ad_low:
                temp = 0
            elif temp >= ad_low + ad_span:
                temp = 255
            else:
                temp = round(((temp - ad_low)/ad_span)*255)
            temp = ((temp - ad_low)/ad_span)*255

            value[i] = temp
        
        scan_angle = data.angle_max
        goc = scan_angle + np.pi + heading/180*np.pi
        # if goc > 2*np.pi:
        #     goc = goc - 2*np.pi
        # if goc < 0:
        #     goc = goc + 2*np.pi
        goc = goc - 2*np.pi*np.floor(goc/(2*np.pi))
        # rint(goc)
        count = (np.round(goc/(2*np.pi)*theta_res)).astype(int)
        count *= (radii_res*2)
        # print(count)
        count = 0 if (count == theta_res*radii_res*2) else count
        colors[count:count+radii_res*2:2,
               0:3] = np.tile((value/255).reshape(-1, 1), (1, 3))
        colors[count+1:count+radii_res*2:2,
               0:3] = np.tile((value/255).reshape(-1, 1), (1, 3))
        m1.setMeshData(vertexes=verts, faces=faces, faceColors=colors)
        m1.resetTransform()
        m1.rotate(-heading, 0, 0, 1)
        line.resetTransform()
        line.rotate(scan_angle*180/np.pi+180, 0, 0, 1)
        count = 0
        return rmin
    intervals = intervals + 1
    text = ''
    if intervals == 1:
        t1 = time.time()
        fps = 1 / (t1 - t)
        # print(f'{10 / (t1 - t):.{4}} fps')
        text = f'Frame rate: \n {fps:.4} fps'
        t = t1
        intervals = 0

    rmin = update(data)
    text += f'\n Distance: {rmin:.4} (m)'
    label.setText(text)


for name in ['axis', 'circle']:
    param.child(name).sigValueChanged.connect(update_settings)
"""
Backend dependent code
"""


class MainWindow(QtWidgets.QMainWindow):
    """
    This is the custom window class for main window
    """

    def __init__(self):
        """
        Create and initialize an instance of the custom MainWindow class. The ui
        is loaded from Ui_Mainwindow()
        """
        super(MainWindow, self).__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

    def toggleState(self):
        if self.isMaximized():
            self.showNormal()
        else:
            self.showMaximized()

    def toggleSidebar(self):
        if self.ui.actionSidebar.isChecked():
            self.ui.dockWidget_2.show()
        else:
            self.ui.dockWidget_2.hide()

    def resetCameraPosition(self):
        q = QtGui.QQuaternion.fromEulerAngles(0, 0, 0)
        w.setCameraPosition(pos=QtGui.QVector3D(
            0, 0, 0), distance=15, rotation=q)

    def startRecording(self):

        timer_25.timeout.connect(grab)
        timer_25.start(40)

    def stopRecording(self):
        timer_25.stop()
        time.sleep(0.04)
        outvid.close()


def connect():
    print("Connect function")


def main():
    window = MainWindow()
    window.show()
    window.ui.gridLayout.addWidget(splitter)
    window.ui.gridLayout_6.addWidget(pt)
    window.ui.actionExit.triggered.connect(window.close)
    window.ui.actionMaximize.triggered.connect(window.showMaximized)
    window.ui.actionMinimize.triggered.connect(window.showMinimized)
    window.ui.actionRestore.triggered.connect(window.showNormal)
    window.ui.actionToggle_state.triggered.connect(window.toggleState)
    window.ui.actionConnect.triggered.connect(connect)
    window.ui.actionOpen.setToolTip("Open")
    window.ui.actionExit.setToolTip("Exit")
    window.ui.actionSidebar.triggered.connect(window.toggleSidebar)
    window.ui.actionReset_camera_position.triggered.connect(
        window.resetCameraPosition)
    window.ui.actionStart_Recording.triggered.connect(window.startRecording)
    window.ui.actionStop_Recording.triggered.connect(window.stopRecording)
    window.ui.actionExit.triggered.connect(window.close)
    app.exec_()


def listener():

    rospy.init_node("plot_serial", anonymous=True)
    rospy.Subscriber("/sonar_micron_ros", LaserScan, distance_callback)
    rospy.Subscriber("/heading/", Float64, header)


if __name__ == '__main__':

    listener()
    main()
