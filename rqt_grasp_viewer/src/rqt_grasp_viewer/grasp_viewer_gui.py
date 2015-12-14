#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 
# derived from sr_gui_controller_tuner
#   original Authors Shadow Robot Team
#
#
# Copyright 2011 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation, either version 2 of the License, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program.  If not, see <http://www.gnu.org/licenses/>.
#

# author Guillaume WALCK (2015)

import rospy
import rospkg
import os
from copy import deepcopy

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi

from python_qt_binding.QtCore import QSize

from QtCore import Qt, QThread, SIGNAL, QObject
from QtGui import QWidget, QStandardItemModel, QStandardItem, QTableView, QCheckBox, QFileDialog, QMessageBox, QPushButton, QFrame, QHBoxLayout, QVBoxLayout

from grasping_msgs.msg import FindGraspableObjectsActionResult

from grasp_viewer.srv import DisplayGrasps, DisplayGraspsRequest

class GraspViewerGUI(Plugin):
    """
    a rqtgui plugin for the grasp viewer
    """

    def __init__(self, context):
        Plugin.__init__(self, context)
        self.setObjectName('RqtGraspViewer')

        self._widget = QWidget()

        
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_grasp_viewer'), 'ui', 'RqtGraspViewer.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('RqtGraspViewerUi')
        context.add_widget(self._widget)
                
        main_layout = QHBoxLayout()
        self._default_labels = ["obj_id", "object", "grasp_id", "grasp", "quality"]
        self._filemodel = QStandardItemModel(0, 5)
        self._filemodel.setHorizontalHeaderLabels(self._default_labels)
        
        
        self._table_view = QTableView()
        self._table_view.setModel(self._filemodel)
        self._table_view.resizeColumnsToContents()
        
        main_layout.addWidget(self._table_view)
        self._widget.scrollarea.setLayout(main_layout)
        
        self.init_services()
        self.init_subscribers()
        
        # self._table_view.clicked.connect(self.on_table_view_clicked)
        self._table_view.selectionModel().selectionChanged.connect(self.on_table_view_select)
        #self._widget.treeWidget.itemClicked.connect(self.on_posture_clicked)


    def init_services(self):
        """
        Init services
        """

        service_name = '/display_grasp'
        try:
            rospy.wait_for_service(service_name)
            self._grasp_viz_client = rospy.ServiceProxy(service_name, DisplayGrasps)
            rospy.loginfo("Found %s", service_name)            
        except rospy.ROSException:
            rospy.logerr("%s did not show up. Giving up", service_name)

    def init_subscribers(self):
        """
        Sets up an action client to communicate with the trajectory controller
        """
        self._grasp_sub = rospy.Subscriber("/grasp_manager/result", FindGraspableObjectsActionResult, self.graspable_result_cb)
    
    def graspable_result_cb(self, msg):
        result = msg.result
        if len(result.objects) > 0:
            fm = self._filemodel
            fm.clear()
            fm.setHorizontalHeaderLabels(self._default_labels)
            self.objs = result.objects
            self.populate_table()

    def populate_table(self):
        """
        update table
        """
        fm = self._filemodel
        for i, obj in enumerate(self.objs):
            for j, grasp in enumerate(obj.grasps):
                item_id = QStandardItem(str(i))
                item_id.setEditable(False)
                item_obj = QStandardItem(obj.object.name)
                item_obj.setEditable(False)
                idem_grasp_id = QStandardItem(str(j))
                idem_grasp_id.setEditable(False)
                item_grasp = QStandardItem(grasp.id)
                item_grasp.setEditable(False)
                item_grasp_quality = QStandardItem(str(grasp.grasp_quality))
                item_grasp_quality.setEditable(False)
                fm.appendRow([item_id, item_obj, idem_grasp_id, item_grasp, item_grasp_quality])
        self._table_view.resizeColumnsToContents()
        #fm = self._filemodel[group_name]
        #item_idx = fm.index(idx, 0)
        #fm.setData(item_idx, str(time_from_start))
        
    def on_table_view_clicked(self, index):
      
        items = self._table_view.selectedIndexes()
        for it in items:
            print 'selected item index found at %s' % it.row()
    
    def on_table_view_select(self, selected, deselected):
        fm = self._filemodel
        items = self._table_view.selectedIndexes()
        if len(items) > 0:
            req = DisplayGraspsRequest()
            for it in items:
                row = it.row()
                obj_idx = fm.index(it.row(), 0)
                obj_id = int(fm.data(obj_idx))
                grasp_idx = fm.index(it.row(), 2)
                grasp_id = int(fm.data(grasp_idx))
                req.grasps.append(self.objs[obj_id].grasps[grasp_id])
                #print "selected item index found at ", obj_id, grasp_id
            self._grasp_viz_client(req)
    
    
    #########
    # Default methods for the rqtgui plugins

    def save_settings(self, global_settings, perspective_settings):
        pass

    def restore_settings(self, global_settings, perspective_settings):
        pass
