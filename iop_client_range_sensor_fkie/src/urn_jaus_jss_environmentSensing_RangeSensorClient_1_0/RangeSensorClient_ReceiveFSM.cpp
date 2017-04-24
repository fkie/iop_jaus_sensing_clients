/**
ROS/IOP Bridge
Copyright (c) 2017 Fraunhofer

This program is dual licensed; you can redistribute it and/or
modify it under the terms of the GNU General Public License
version 2 as published by the Free Software Foundation, or
enter into a proprietary license agreement with the copyright
holder.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; or you can read the full license at
<http://www.gnu.de/documents/gpl-2.0.html>
*/

/** \author Alexander Tiderko */


#include "urn_jaus_jss_environmentSensing_RangeSensorClient_1_0/RangeSensorClient_ReceiveFSM.h"
#include <iop_builder_fkie/timestamp.h>


using namespace JTS;

namespace urn_jaus_jss_environmentSensing_RangeSensorClient_1_0
{



RangeSensorClient_ReceiveFSM::RangeSensorClient_ReceiveFSM(urn_jaus_jss_core_Transport_1_0::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_EventsClient_1_0::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM, urn_jaus_jss_core_AccessControlClient_1_0::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM)
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new RangeSensorClient_ReceiveFSMContext(*this);

	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->pEventsClient_ReceiveFSM = pEventsClient_ReceiveFSM;
	this->pAccessControlClient_ReceiveFSM = pAccessControlClient_ReceiveFSM;
	p_pnh = ros::NodeHandle("~");
}



RangeSensorClient_ReceiveFSM::~RangeSensorClient_ReceiveFSM()
{
	delete context;
}

void RangeSensorClient_ReceiveFSM::setupNotifications()
{
	pAccessControlClient_ReceiveFSM->registerNotification("Receiving_Ready", ieHandler, "InternalStateChange_To_RangeSensorClient_ReceiveFSM_Receiving_Ready", "AccessControlClient_ReceiveFSM");
	pAccessControlClient_ReceiveFSM->registerNotification("Receiving", ieHandler, "InternalStateChange_To_RangeSensorClient_ReceiveFSM_Receiving_Ready", "AccessControlClient_ReceiveFSM");
	registerNotification("Receiving_Ready", pAccessControlClient_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControlClient_ReceiveFSM_Receiving_Ready", "RangeSensorClient_ReceiveFSM");
	registerNotification("Receiving", pAccessControlClient_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControlClient_ReceiveFSM_Receiving", "RangeSensorClient_ReceiveFSM");
	p_tf_frame_robot = "base_link";
	p_pnh.param("tf_frame_robot", p_tf_frame_robot, p_tf_frame_robot);
	ROS_INFO("tf_frame_robot: %s", p_tf_frame_robot.c_str());
	p_ocu_control_layer_slave.set_access_state_handler(&RangeSensorClient_ReceiveFSM::pAccessStateHandler, this);
	p_ocu_control_layer_slave.init(*(jausRouter->getJausAddress()), "urn:jaus:jss:environmentSensing:RangeSensor", 1, 0);
}

void RangeSensorClient_ReceiveFSM::pAccessStateHandler(JausAddress &address, unsigned char code)
{
	if (code == OcuControlSlave::ACCESS_STATE_CONTROL_ACCEPTED) {
		urn_jaus_jss_environmentSensing_RangeSensorClient_1_0::QueryRangeSensorConfiguration query;
		sendJausMessage(query, address);
	} else if (code == OcuControlSlave::ACCESS_CONTROL_RELEASE) {
		pEventsClient_ReceiveFSM->cancel_event(address, p_query_sensor_data);
		ROS_INFO_NAMED("RangeSensorClient", "cancel event for range sensor data by %d.%d.%d",
				address.getSubsystemID(), address.getNodeID(), address.getComponentID());
	}
}

void RangeSensorClient_ReceiveFSM::pHandleeventReportRangeSensorDataAction(Receive::Body::ReceiveRec &transport_data, urn_jaus_jss_core_EventsClient_1_0::Event &msg)
{
	ReportRangeSensorData report;
	report.decode(msg.getBody()->getEventRec()->getReportMessage()->getData());
	handleReportRangeSensorDataAction(report, transport_data);
}

void RangeSensorClient_ReceiveFSM::handleConfirmSensorConfigurationAction(ConfirmSensorConfiguration msg, Receive::Body::ReceiveRec transportData)
{
	ROS_WARN("RangeSensorClient: handleConfirmSensorConfigurationAction not implemented!");
}

void RangeSensorClient_ReceiveFSM::handleReportRangeSensorCapabilitiesAction(ReportRangeSensorCapabilities msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
	uint16_t subsystem_id = transportData.getSrcSubsystemID();
	uint8_t node_id = transportData.getSrcNodeID();
	uint8_t component_id = transportData.getSrcComponentID();
	JausAddress sender(subsystem_id, node_id, component_id);
	ROS_INFO_NAMED("RangeSensorClient", "received capabilities from %d.%d.%d",
			sender.getSubsystemID(), sender.getNodeID(), sender.getComponentID());
	// create for each sensor a publisher
	for (unsigned int i = 0; i < msg.getBody()->getRangeSensorCapabilitiesList()->getNumberOfElements(); i++) {
		ReportRangeSensorCapabilities::Body::RangeSensorCapabilitiesList::RangeSensorCapabilitiesRec *item = msg.getBody()->getRangeSensorCapabilitiesList()->getElement(i);
		unsigned int id = item->getSensorID();
		if (p_publisher_map.find(id) == p_publisher_map.end()) {
			p_publisher_map[id] = p_pnh.advertise<sensor_msgs::LaserScan>(item->getSensorName(), 1, false);
			p_sensor_names[id] = ros::this_node::getNamespace().substr(1) + item->getSensorName();
			p_sensor_max_range[id] = item->getMaximumRange();
			p_sensor_min_range[id] = item->getMinimumRange();
			try {
				p_tf_map[id].child_frame_id = p_sensor_names[id];
			} catch (std::exception &e) {
			}
		}
	}
	ROS_INFO_NAMED("RangeSensorClient", "create event to get range data from %d.%d.%d",
			sender.getSubsystemID(), sender.getNodeID(), sender.getComponentID());
	pEventsClient_ReceiveFSM->create_event(&RangeSensorClient_ReceiveFSM::pHandleeventReportRangeSensorDataAction, this, sender, p_query_sensor_data, 5.0, 1);
}

void RangeSensorClient_ReceiveFSM::handleReportRangeSensorCompressedDataAction(ReportRangeSensorCompressedData msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
	ROS_WARN("RangeSensorClient: handleReportRangeSensorCompressedDataAction not implemented!");
}

void RangeSensorClient_ReceiveFSM::handleReportRangeSensorConfigurationAction(ReportRangeSensorConfiguration msg, Receive::Body::ReceiveRec transportData)
{
	uint16_t subsystem_id = transportData.getSrcSubsystemID();
	uint8_t node_id = transportData.getSrcNodeID();
	uint8_t component_id = transportData.getSrcComponentID();
	JausAddress sender(subsystem_id, node_id, component_id);
	ROS_DEBUG_NAMED("RangeSensorClient", "ReportRangeSensorConfiguration, count sensors: %d", msg.getBody()->getRangeSensorConfigurationList()->getNumberOfElements());
	ROS_INFO_NAMED("RangeSensorClient", "request GeometricProperties and SensorCapabilities from %d.%d.%d",
				   sender.getSubsystemID(), sender.getNodeID(), sender.getComponentID());
	urn_jaus_jss_environmentSensing_RangeSensorClient_1_0::QuerySensorGeometricProperties query_geo;
	this->sendJausMessage(query_geo, sender);
	urn_jaus_jss_environmentSensing_RangeSensorClient_1_0::QueryRangeSensorCapabilities query_cap;
	this->sendJausMessage(query_cap, sender);
}

void RangeSensorClient_ReceiveFSM::handleReportRangeSensorDataAction(ReportRangeSensorData msg, Receive::Body::ReceiveRec transportData)
{
	uint16_t subsystem_id = transportData.getSrcSubsystemID();
	uint8_t node_id = transportData.getSrcNodeID();
	uint8_t component_id = transportData.getSrcComponentID();
	JausAddress sender(subsystem_id, node_id, component_id);
	for (unsigned int i = 0; i < msg.getBody()->getRangeSensorDataList()->getNumberOfElements(); i++) {
		ReportRangeSensorData::Body::RangeSensorDataList::RangeSensorDataVariant *itemvar = msg.getBody()->getRangeSensorDataList()->getElement(i);
		if (itemvar->getFieldValue() == 1) {
			ReportRangeSensorData::Body::RangeSensorDataList::RangeSensorDataVariant::RangeSensorDataSeq::RangeSensorDataRec *datarec = itemvar->getRangeSensorDataSeq()->getRangeSensorDataRec();
			ReportRangeSensorData::Body::RangeSensorDataList::RangeSensorDataVariant::RangeSensorDataSeq::RangeSensorDataPointList *pointlist = itemvar->getRangeSensorDataSeq()->getRangeSensorDataPointList();
			unsigned int id = datarec->getSensorID();
			if (p_publisher_map.find(id) != p_publisher_map.end()) {
				std::string sensor_id = p_sensor_names[id];
				// get timestamp
				ReportRangeSensorData::Body::RangeSensorDataList::RangeSensorDataVariant::RangeSensorDataSeq::RangeSensorDataRec::TimeStamp *ts = datarec->getTimeStamp();
				iop::Timestamp stamp(ts->getDay(), ts->getHour(), ts->getMinutes(), ts->getSeconds(), ts->getMilliseconds());
				try {
					geometry_msgs::TransformStamped tf_msg  = p_tf_map[id];
					if (! tf_msg.child_frame_id.empty()) {
						tf_msg.header.stamp = stamp.ros_time;
						p_tf_broadcaster.sendTransform(tf_msg);
					}
				} catch (std::exception &e) {
					ROS_WARN("RangeSensorClient: can not publish tf for sensor %s: %s", sensor_id.c_str(), e.what());
				}
				sensor_msgs::LaserScan rosmsg;
				rosmsg.header.frame_id = sensor_id;
				rosmsg.header.stamp = stamp.ros_time;
				// get points
				rosmsg.angle_min = 0;
				rosmsg.angle_max = 0;
				for (unsigned int i = 0; i < pointlist->getNumberOfElements(); i++) {
					ReportRangeSensorData::Body::RangeSensorDataList::RangeSensorDataVariant::RangeSensorDataSeq::RangeSensorDataPointList::RangeSensorDataPointRec *point = pointlist->getElement(i);
					if (i == 1) {
						rosmsg.angle_increment = point->getBearing() - pointlist->getElement(0)->getBearing();
					}
					if (point->getBearing() < rosmsg.angle_min) {
						rosmsg.angle_min = point->getBearing();
					}
					if (point->getBearing() > rosmsg.angle_max) {
						rosmsg.angle_max = point->getBearing();
					}
					double range = NAN;
					if (point->isRangeValidityValid() && point->getRangeValidity() == 1) {
						range = point->getRange();
					}
					rosmsg.ranges.push_back(range);
				}
				rosmsg.range_max = p_sensor_max_range[id];
				rosmsg.range_min = p_sensor_min_range[id];
				p_publisher_map[id].publish(rosmsg);
			} else {
				// no capabilities received for this sensor id
				ROS_WARN("RangeSensorClient: no capabilities received for this sensor id: %d", id);
			}
		} else {
		  // it is an error
		  ROS_WARN("RangeSensorClient: error rec received, code: %d, msg: %s", itemvar->getRangeSensorDataErrorRec()->getDataErrorCode(), itemvar->getRangeSensorDataErrorRec()->getErrorMessage().c_str());
		}
	}
	ROS_DEBUG_NAMED("RangeSensorClient", "ReportRangeSensorData, count sensors: %d", msg.getBody()->getRangeSensorDataList()->getNumberOfElements());
}

void RangeSensorClient_ReceiveFSM::handleReportSensorGeometricPropertiesAction(ReportSensorGeometricProperties msg, Receive::Body::ReceiveRec transportData)
{
	uint16_t subsystem_id = transportData.getSrcSubsystemID();
	uint8_t node_id = transportData.getSrcNodeID();
	uint8_t component_id = transportData.getSrcComponentID();
	JausAddress sender(subsystem_id, node_id, component_id);
	ROS_DEBUG_NAMED("RangeSensorClient", "ReportSensorGeometricProperties, count sensors: %d", msg.getBody()->getGeometricPropertiesList()->getNumberOfElements());
	// create tf data for all valid sensors positions
	for (unsigned int i = 0; i < msg.getBody()->getGeometricPropertiesList()->getNumberOfElements(); i++) {
		try {
			ReportSensorGeometricProperties::Body::GeometricPropertiesList::GeometricPropertiesSequence *element = msg.getBody()->getGeometricPropertiesList()->getElement(i);
			jUnsignedShortInteger sensor_id = element->getSensorIdRec()->getSensorID();
			// create and publish TF message
			ReportSensorGeometricProperties::Body::GeometricPropertiesList::GeometricPropertiesSequence::GeometricPropertiesVariant::StaticGeometricPropertiesRec *staticgeo = element->getGeometricPropertiesVariant()->getStaticGeometricPropertiesRec();
			geometry_msgs::TransformStamped msg;
			msg.transform.translation.x = staticgeo->getSensorPosition()->getPositionVectorElement(0);
			msg.transform.translation.y = staticgeo->getSensorPosition()->getPositionVectorElement(1);
			msg.transform.translation.z = staticgeo->getSensorPosition()->getPositionVectorElement(2);
			msg.transform.rotation.x = staticgeo->getUnitQuaternion()->getUnitQuaternionElement(0);
			msg.transform.rotation.y = staticgeo->getUnitQuaternion()->getUnitQuaternionElement(1);
			msg.transform.rotation.z = staticgeo->getUnitQuaternion()->getUnitQuaternionElement(2);
			msg.transform.rotation.w = staticgeo->getUnitQuaternion()->getUnitQuaternionElement(3);
			msg.header.stamp = ros::Time::now();
			msg.header.frame_id = this->p_tf_frame_robot;
			msg.child_frame_id = p_sensor_names[sensor_id];
			ROS_DEBUG_NAMED("RangeSensorClient", "tf %s -> %s", this->p_tf_frame_robot.c_str(), p_sensor_names[sensor_id].c_str());
			p_tf_map[sensor_id] = msg;
		} catch (std::exception &e) {
			ROS_WARN("RangeSensorClient: can not create tf for sensor: %s", e.what());
		}
	}
}





};
