import robinion_gui as rb_gui
from PySide2 import QtWidgets

class RobinionQtGUI(rb_gui.Ui_MainWindow, QtWidgets.QMainWindow):
    def __init__(self):
        super(RobinionQtGUI, self).__init__()
        # Setup ROS Node
        self.setupUi(self)

if __name__ == "__main__":
    rb_node = QtWidgets.QApplication()
    rb_node_gui = RobinionQtGUI()
    rb_node_gui.show()
    rb_node.exec_()