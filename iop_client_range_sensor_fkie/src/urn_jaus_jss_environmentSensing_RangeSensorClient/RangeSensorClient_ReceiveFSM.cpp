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


#include "urn_jaus_jss_environmentSensing_RangeSensorClient/RangeSensorClient_ReceiveFSM.h"
#include <iop_builder_fkie/timestamp.h>
#include <iop_ocu_slavelib_fkie/Slave.h>
#include <iop_component_fkie/iop_config.h>


using namespace JTS;
using namespace iop::ocu;

namespace urn_jaus_jss_environmentSensing_RangeSensorClient
{



RangeSensorClient_ReceiveFSM::RangeSensorClient_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM, urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM)
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
	p_has_access = false;
	p_query_state = 0;
	p_by_query = false;
	p_hz = 5.0;
	QueryRangeSensorData::Body::QueryRangeSensorDataList::QueryRangeSensorDataRec drec;
	drec.setSensorID(0); // 0 is specified to get all sensors
	drec.setReportCoordinateSystem(1); // we use vehicle coordinate system
	p_query_sensor_data.getBody()->getQueryRangeSensorDataList()->addElement(drec);

	QueryRangeSensorConfiguration::Body::RangeSensorConfigurationList::QueryRangeSensorConfigurationRec crec;
	crec.setSensorID(0); // 0 is specified to get all sensors
	crec.setQueryPresenceVector(65535);
	p_query_cfg.getBody()->getRangeSensorConfigurationList()->addElement(crec);

	QuerySensorGeometricProperties::Body::SensorIdList::SensorIdRec sgrec;
	sgrec.setSensorID(0); // 0 is specified to get all sensors
	p_query_geo.getBody()->getSensorIdList()->addElement(sgrec);
	QueryRangeSensorCapabilities::Body::RangeSensorCapabilitiesList::QueryRangeSensorCapabilitiesRec rscrec;
	rscrec.setSensorID(0); // 0 is specified to get all sensors
	// miss the possibility to set the presence vector
	// rscrec.m_QueryPresenceVector = 65535;
	p_query_cap.getBody()->getRangeSensorCapabilitiesList()->addElement(rscrec);
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
	iop::Config cfg("~RangeSensorClient");
	cfg.param("hz", p_hz, p_hz, false, false);
	cfg.param("tf_frame_robot", p_tf_frame_robot, std::string("base_link"));
	Slave &slave = Slave::get_instance(*(jausRouter->getJausAddress()));
	slave.add_supported_service(*this, "urn:jaus:jss:environmentSensing:RangeSensor", 1, 0);
}

void RangeSensorClient_ReceiveFSM::control_allowed(std::string service_uri, JausAddress component, unsigned char authority)
{
	if (service_uri.compare("urn:jaus:jss:environmentSensing:RangeSensor") == 0) {
		p_remote_addr = component;
		p_has_access = true;
	} else {
		ROS_WARN_STREAM("[RangeSensorClient] unexpected control allowed for " << service_uri << " received, ignored!");
	}
}

void RangeSensorClient_ReceiveFSM::enable_monitoring_only(std::string service_uri, JausAddress component)
{
	p_remote_addr = component;
}

void RangeSensorClient_ReceiveFSM::access_deactivated(std::string service_uri, JausAddress component)
{
	p_has_access = false;
	p_remote_addr = JausAddress(0);
}

void RangeSensorClient_ReceiveFSM::create_events(std::string service_uri, JausAddress component, bool by_query)
{
	ROS_DEBUG_NAMED("RangeSensorClient", "create EVENT for range sensor %s", component.str().c_str());
	p_by_query = by_query;
	sendJausMessage(p_query_cfg, component);
	p_query_timer = p_nh.createTimer(ros::Duration(3), &RangeSensorClient_ReceiveFSM::pQueryCallback, this);
}

void RangeSensorClient_ReceiveFSM::cancel_events(std::string service_uri, JausAddress component, bool by_query)
{
	p_query_timer.stop();
	if (!by_query) {
		ROS_INFO_NAMED("RangeSensorClient", "cancel EVENT for range sensor data by %s", component.str().c_str());
		pEventsClient_ReceiveFSM->cancel_event(*this, component, p_query_sensor_data);
	}
	p_query_state = 0;
}

void RangeSensorClient_ReceiveFSM::pQueryCallback(const ros::TimerEvent& event)
{
	if (p_remote_addr.get() != 0) {
		if (p_query_state == 0) {
			sendJausMessage(p_query_cfg, p_remote_addr);
		} else if (p_query_state == 1) {
			ROS_DEBUG_NAMED("RangeSensorClient", "send query geometrics and capabilities for range sensor to %s", p_remote_addr.str().c_str());
			sendJausMessage(p_query_geo, p_remote_addr);
			sendJausMessage(p_query_cap, p_remote_addr);
		} else {
			sendJausMessage(p_query_sensor_data, p_remote_addr);
		}
	}
}

void RangeSensorClient_ReceiveFSM::event(JausAddress sender, unsigned short query_msg_id, unsigned int reportlen, const unsigned char* reportdata)
{
	ReportRangeSensorData report;
	report.decode(reportdata);
	Receive::Body::ReceiveRec transport_data;
	transport_data.setSrcSubsystemID(sender.getSubsystemID());
	transport_data.setSrcNodeID(sender.getNodeID());
	transport_data.setSrcComponentID(sender.getComponentID());
	handleReportRangeSensorDataAction(report, transport_data);
}

void RangeSensorClient_ReceiveFSM::handleConfirmSensorConfigurationAction(ConfirmSensorConfiguration msg, Receive::Body::ReceiveRec transportData)
{
	ROS_WARN_NAMED("RangeSensorClient", "handleConfirmSensorConfigurationAction not implemented!");
}

void RangeSensorClient_ReceiveFSM::handleReportRangeSensorCapabilitiesAction(ReportRangeSensorCapabilities msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
	uint16_t subsystem_id = transportData.getSrcSubsystemID();
	uint8_t node_id = transportData.getSrcNodeID();
	uint8_t component_id = transportData.getSrcComponentID();
	JausAddress sender(subsystem_id, node_id, component_id);
	ROS_INFO_NAMED("RangeSensorClient", "received capabilities for range data from %s", sender.str().c_str());
	// create for each sensor a publisher
	for (unsigned int i = 0; i < msg.getBody()->getRangeSensorCapabilitiesList()->getNumberOfElements(); i++) {
		ReportRangeSensorCapabilities::Body::RangeSensorCapabilitiesList::RangeSensorCapabilitiesRec *item = msg.getBody()->getRangeSensorCapabilitiesList()->getElement(i);
		unsigned int id = item->getSensorID();
		if (p_publisher_map.find(id) == p_publisher_map.end()) {
			p_publisher_map[id] = p_nh.advertise<sensor_msgs::LaserScan>(item->getSensorName(), 1, false);
			p_sensor_names[id] = ros::this_node::getNamespace().substr(2) + "/" + item->getSensorName();
			p_sensor_max_range[id] = item->getMaximumRange();
			p_sensor_min_range[id] = item->getMinimumRange();
			try {
				p_tf_map[id].child_frame_id = p_sensor_names[id];
			} catch (std::exception &e) {
			}
		}
	}
	p_query_state = 2;
	// create event or timer for queries
	if (p_remote_addr.get() != 0) {
		if (p_by_query) {
			p_query_timer.stop();
			if (p_hz > 0) {
				ROS_INFO_NAMED("RangeSensorClient", "create QUERY timer to get range data from %s", p_remote_addr.str().c_str());
				p_query_timer = p_nh.createTimer(ros::Duration(1.0 / p_hz), &RangeSensorClient_ReceiveFSM::pQueryCallback, this);
			} else {
				ROS_WARN_NAMED("RangeSensorClient", "invalid hz %.2f for QUERY timer to get range data from %s", p_hz, p_remote_addr.str().c_str());
			}
		} else {
			ROS_INFO_NAMED("RangeSensorClient", "create EVENT to get range data from %s", p_remote_addr.str().c_str());
			pEventsClient_ReceiveFSM->create_event(*this, p_remote_addr, p_query_sensor_data, p_hz);
		}
	}
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
	ROS_INFO_NAMED("RangeSensorClient", "request GeometricProperties and SensorCapabilities from %s", sender.str().c_str());
	p_query_state = 1;
	sendJausMessage(p_query_geo, sender);
	sendJausMessage(p_query_cap, sender);
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
					geometry_msgs::TransformStamped& tf_msg  = p_tf_map.at(id);
					if (! tf_msg.child_frame_id.empty()) {
						tf_msg.header.stamp = stamp.ros_time;
						if (tf_msg.header.frame_id.empty()) {
							tf_msg.header.frame_id = this->p_tf_frame_robot;
						}
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
					if (point->isRangeValidityValid()) {
						if (point->getRangeValidity() == 1) {
							range = point->getRange();
						}
					} else {
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
		  ROS_WARN("RangeSensorClient: error rec for ID: %d received, code: %d, msg: %s", itemvar->getRangeSensorDataErrorRec()->getSensorID(), itemvar->getRangeSensorDataErrorRec()->getDataErrorCode(), itemvar->getRangeSensorDataErrorRec()->getErrorMessage().c_str());
		}
	}
	ROS_DEBUG_NAMED("RangeSensorClient", "ReportRangeSensorData, count sensors: %d", msg.getBody()->getRangeSensorDataList()->getNumberOfElements());
}

void RangeSensorClient_ReceiveFSM::handleReportSensorGeometricPropertiesAction(ReportSensorGeometricProperties msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
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
			ROS_DEBUG_NAMED("RangeSensorClient", "initialized tf %s -> %s", this->p_tf_frame_robot.c_str(), p_sensor_names[sensor_id].c_str());
			p_tf_map[sensor_id] = msg;
		} catch (std::exception &e) {
			ROS_WARN("RangeSensorClient: can not create tf for sensor: %s", e.what());
		}
	}
}





};
