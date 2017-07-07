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
from PyQt5.uic import loadUi

from QtCore import Qt, QThread, pyqtSignal, QObject
from QtGui import QStandardItemModel, QStandardItem
from QtWidgets import QWidget, QTableView, QCheckBox, QFileDialog, \
     QMessageBox, QPushButton, QFrame, QHBoxLayout, QVBoxLayout, QRadioButton

from geometry_msgs.msg import TransformStamped, Vector3Stamped, Vector3, Point, PointStamped, Quaternion
from grasping_msgs.msg import GraspPlanningActionResult, GenerateGraspsActionResult
from grasp_viewer.srv import DisplayGrasps, DisplayGraspsRequest
from visualization_msgs.msg import InteractiveMarkerFeedback as IM
import tf2_geometry_msgs


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

        self.radio = {}
        for group, group_name in zip([self._widget.groupBoxGoal, self._widget.groupBoxStart],
                                     ['goal', 'start']):
            vl = QVBoxLayout()
            self.radio[group_name] = {}
            for button in ['grasp', 'approach', 'retreat']:
                self.radio[group_name][button] = QRadioButton()
                self.radio[group_name][button].setText(button)
                vl.addWidget(self.radio[group_name][button])
            group.setLayout(vl)

        self.init_services()
        self.init_subscribers()
        self.init_publishers()

        # self._table_view.clicked.connect(self.on_table_view_clicked)
        self._table_view.selectionModel().selectionChanged.connect(self.on_table_view_select)
        # self._widget.treeWidget.itemClicked.connect(self.on_posture_clicked)

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
        self._grasp_sub = rospy.Subscriber("/grasp_manager/result", GraspPlanningActionResult,
                                           self.graspable_result_cb)
        self._grasp_sub2 = rospy.Subscriber("/grasp_provider/result", GenerateGraspsActionResult,
                                            self.graspable_result_cb)

    def init_publishers(self):
        self._int_marker = rospy.Publisher("/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/feedback", IM, queue_size=2)

    def graspable_result_cb(self, msg):
        result = msg.result
        if len(result.grasps) > 0:
            fm = self._filemodel
            fm.clear()
            fm.setHorizontalHeaderLabels(self._default_labels)
            self.grasps = result.grasps
            self.populate_table()

    def populate_table(self):
        """
        update table
        """
        fm = self._filemodel
        i = 0
        for j, grasp in enumerate(self.grasps):
            item_id = QStandardItem(str(i))
            item_id.setEditable(False)
            item_obj = QStandardItem("unknown")
            item_obj.setEditable(False)
            idem_grasp_id = QStandardItem(str(j))
            idem_grasp_id.setEditable(False)
            item_grasp = QStandardItem(grasp.id)
            item_grasp.setEditable(False)
            item_grasp_quality = QStandardItem(str(grasp.grasp_quality))
            item_grasp_quality.setEditable(False)
            fm.appendRow([item_id, item_obj, idem_grasp_id, item_grasp, item_grasp_quality])
        self._table_view.resizeColumnsToContents()
        # fm = self._filemodel[group_name]
        # item_idx = fm.index(idx, 0)
        # fm.setData(item_idx, str(time_from_start))

    def on_table_view_clicked(self, index):

        items = self._table_view.selectedIndexes()
        # for it in items:
        #    print 'selected item index found at %s' % it.row()

    def make_marker_grasp(self, marker, last_grasp_id):
        marker.pose = self.grasps[last_grasp_id].grasp_pose.pose

    def make_marker_approach(self, marker, last_grasp_id):
        approach = self.grasps[last_grasp_id].pre_grasp_approach.direction
        approach_as_point = PointStamped()
        approach_as_point.point = Point(approach.vector.x, approach.vector.y, approach.vector.z)
        approach_as_point.point.x *= -1 * self.grasps[last_grasp_id].pre_grasp_approach.desired_distance
        approach_as_point.point.y *= -1 * self.grasps[last_grasp_id].pre_grasp_approach.desired_distance
        approach_as_point.point.z *= -1 * self.grasps[last_grasp_id].pre_grasp_approach.desired_distance

        t = TransformStamped()
        t.transform.translation = Vector3(self.grasps[last_grasp_id].grasp_pose.pose.position.x,
                                          self.grasps[last_grasp_id].grasp_pose.pose.position.y,
                                          self.grasps[last_grasp_id].grasp_pose.pose.position.z)
        t.transform.rotation = Quaternion(self.grasps[last_grasp_id].grasp_pose.pose.orientation.x,
                                          self.grasps[last_grasp_id].grasp_pose.pose.orientation.y,
                                          self.grasps[last_grasp_id].grasp_pose.pose.orientation.z,
                                          self.grasps[last_grasp_id].grasp_pose.pose.orientation.w)

        new_approach_as_point = tf2_geometry_msgs.do_transform_point(approach_as_point, t)
        marker.pose = deepcopy(self.grasps[last_grasp_id].grasp_pose.pose)
        marker.pose.position = new_approach_as_point.point

    def make_marker_retreat(self, marker, last_grasp_id):
        retreat = self.grasps[last_grasp_id].post_grasp_retreat.direction
        retreat_as_point = PointStamped()
        retreat_as_point.point = Point(retreat.vector.x, retreat.vector.y, retreat.vector.z)
        retreat_as_point.point.x *= self.grasps[last_grasp_id].post_grasp_retreat.desired_distance
        retreat_as_point.point.y *= self.grasps[last_grasp_id].post_grasp_retreat.desired_distance
        retreat_as_point.point.z *= self.grasps[last_grasp_id].post_grasp_retreat.desired_distance

        marker.pose = deepcopy(self.grasps[last_grasp_id].grasp_pose.pose)
        marker.pose.position.x += retreat_as_point.point.x
        marker.pose.position.y += retreat_as_point.point.y
        marker.pose.position.z += retreat_as_point.point.z

    def on_table_view_select(self, selected, deselected):
        fm = self._filemodel
        items = self._table_view.selectedIndexes()
        if len(items) > 0:
            if len(items) > 5:
                rospy.logwarn("More than 5 items selected, \
                              but grasp viewer can only display 5 grasps at the same time")
            req = DisplayGraspsRequest()
            last_grasp_id = None
            for it in items:
                row = it.row()
                obj_idx = fm.index(it.row(), 0)
                obj_id = int(fm.data(obj_idx))
                grasp_idx = fm.index(it.row(), 2)
                grasp_id = int(fm.data(grasp_idx))
                req.grasps.append(self.grasps[grasp_id])
                last_grasp_id = grasp_id
                # print "selected item index found at ", obj_id, grasp_id
            try:
                self._grasp_viz_client(req)
            except Exception as e:
                print e

            # the approach is in frame ee_link, retrieve it from there
            ee_link = self.grasps[last_grasp_id].pre_grasp_approach.direction.header.frame_id
            markerFeedBack = IM()

            markerFeedBack.header.frame_id = "base_link"
            markerFeedBack.control_name = "move"
            markerFeedBack.event_type = IM.POSE_UPDATE

            for checkbox, name in zip([self._widget.int_mk_goal_chk, self._widget.int_mk_start_chk],
                                      ['goal', 'start']):
                if checkbox.isChecked():
                    markerFeedBack.marker_name = "EE:" + name + "_" + ee_link
                    if self.radio[name]['grasp'].isChecked():
                        self.make_marker_grasp(markerFeedBack, last_grasp_id)
                    if self.radio[name]['approach'].isChecked():
                        self.make_marker_approach(markerFeedBack, last_grasp_id)
                    if self.radio[name]['retreat'].isChecked():
                        self.make_marker_retreat(markerFeedBack, last_grasp_id)
                    self._int_marker.publish(markerFeedBack)
                    # print markerFeedBack

    #########
    # Default methods for the rqtgui plugins

    def save_settings(self, global_settings, perspective_settings):
        pass

    def restore_settings(self, global_settings, perspective_settings):
        pass
