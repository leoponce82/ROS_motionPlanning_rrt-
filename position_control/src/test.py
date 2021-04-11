from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
import sys
import time
import multiprocessing


def mp_worker(a):
    for i in range(a):
        time.sleep(1)
        print('keep going :' +str(i))
    return

def Multifunc(k):
    ps = []
    for i in range(k):
        p = multiprocessing.Process(target= mp_worker , args=(10,))
        ps.append(p)
        p.start()
    return ps



class SurfViewer(QMainWindow):
    def __init__(self, parent=None):
        super(SurfViewer, self).__init__()
        self.parent = parent

        self.centralWidget = QWidget()
        self.setCentralWidget(self.centralWidget)
        self.mainHBOX_param_scene = QHBoxLayout()

        self.Param_box = QGroupBox(" ")
        self.layout_Param_box = QVBoxLayout()
        self.Button_Threads = QPushButton('Run Threads')
        self.Button_stop = QPushButton('stop Threads')
        self.layout_Param_box.addWidget(self.Button_Threads)
        self.layout_Param_box.addWidget(self.Button_stop)
        self.Param_box.setLayout(self.layout_Param_box)

        self.mainHBOX_param_scene.addWidget(self.Param_box)
        self.centralWidget.setLayout(self.mainHBOX_param_scene)

        self.Button_Threads.clicked.connect(self.RunThreads)
        self.Button_stop.clicked.connect(self.stop)

    def RunThreads(self):
        self.ps = Multifunc(10)
        return

    def stop(self):
        # How stop the multi process ?
        for p in self.ps:
            p.terminate()


def main():
    app = QApplication(sys.argv)
    ex = SurfViewer(app)
    ex.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
