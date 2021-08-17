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
#include <fkie_iop_component/iop_config.hpp>


using namespace JTS;
using namespace iop::ocu;

namespace urn_jaus_jss_environmentSensing_RangeSensorClient
{



RangeSensorClient_ReceiveFSM::RangeSensorClient_ReceiveFSM(std::shared_ptr<iop::Component> cmp, urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM, urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM)
: SlaveHandlerInterface(cmp, "RangeSensorClient", 2.0),
  logger(cmp->get_logger().get_child("RangeSensorClient")),
  p_tf_broadcaster(cmp)
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new RangeSensorClient_ReceiveFSMContext(*this);

	this->pAccessControlClient_ReceiveFSM = pAccessControlClient_ReceiveFSM;
	this->pEventsClient_ReceiveFSM = pEventsClient_ReceiveFSM;
	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->cmp = cmp;
	p_query_state = 0;
	p_hz = 10.0;
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

}


void RangeSensorClient_ReceiveFSM::setupIopConfiguration()
{
	iop::Config cfg(cmp, "RangeSensorClient");
	cfg.declare_param<std::string>("tf_frame_robot", p_tf_frame_robot, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_STRING,
		"TF frame id of the robot.",
		"Default: 'base_link'");
	cfg.declare_param<double>("hz", p_hz, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE,
		"Sets how often the reports are requested. If use_queries is True hz must be greather then 0. In this case each time a Query message is sent to get a report. If use_queries is False an event is created to get Reports. In this case 0 disables the rate and an event of type on_change will be created.",
		"Default: 10.0");
	cfg.param("tf_frame_robot", p_tf_frame_robot, std::string("base_link"));
	cfg.param("hz", p_hz, p_hz, false);
	// initialize the control layer, which handles the access control staff
	this->set_rate(p_hz);
	this->set_supported_service(*this, "urn:jaus:jss:environmentSensing:RangeSensor", 1, 0);
	this->set_event_name("range sensor capabilities");
	this->set_query_before_event(true, 1.0);
}

void RangeSensorClient_ReceiveFSM::register_events(JausAddress remote_addr, double hz)
{
	pEventsClient_ReceiveFSM->create_event(*this, remote_addr, p_query_sensor_data, p_hz);
}

void RangeSensorClient_ReceiveFSM::unregister_events(JausAddress remote_addr)
{
	pEventsClient_ReceiveFSM->cancel_event(*this, remote_addr, p_query_sensor_data);
	stop_query(remote_addr);
}

void RangeSensorClient_ReceiveFSM::send_query(JausAddress remote_addr)
{
	// sendJausMessage(p_query_sensor_data, remote_addr);
	if (p_query_state == 0) {
		sendJausMessage(p_query_cfg, remote_addr);
	} else if (p_query_state == 1) {
		RCLCPP_DEBUG(logger, "send query geometrics and capabilities for range sensor to %s", remote_addr.str().c_str());
		sendJausMessage(p_query_geo, remote_addr);
		sendJausMessage(p_query_cap, remote_addr);
		p_query_state = 2;
		this->set_event_name("range data");
		this->set_query_before_event(false, p_hz);
	} else {
		sendJausMessage(p_query_sensor_data, remote_addr);
		this->set_event_name("range data");
		this->set_query_before_event(false, p_hz);
	}
}

void RangeSensorClient_ReceiveFSM::stop_query(JausAddress remote_addr)
{
	p_query_state = 0;
	this->set_event_name("range sensor capabilities");
	this->set_query_before_event(true, 1.0);
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
	RCLCPP_WARN(logger, "handleConfirmSensorConfigurationAction not implemented!");
}

void RangeSensorClient_ReceiveFSM::handleReportRangeSensorCapabilitiesAction(ReportRangeSensorCapabilities msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
	uint16_t subsystem_id = transportData.getSrcSubsystemID();
	uint8_t node_id = transportData.getSrcNodeID();
	uint8_t component_id = transportData.getSrcComponentID();
	JausAddress sender(subsystem_id, node_id, component_id);
	RCLCPP_INFO(logger, "received capabilities for range data from %s", sender.str().c_str());
	// create for each sensor a publisher
	iop::Config cfg(cmp, "RangeSensorClient");
	for (unsigned int i = 0; i < msg.getBody()->getRangeSensorCapabilitiesList()->getNumberOfElements(); i++) {
		ReportRangeSensorCapabilities::Body::RangeSensorCapabilitiesList::RangeSensorCapabilitiesRec *item = msg.getBody()->getRangeSensorCapabilitiesList()->getElement(i);
		unsigned int id = item->getSensorID();
		if (p_publisher_map.find(id) == p_publisher_map.end()) {
			p_publisher_map[id] = cfg.create_publisher<sensor_msgs::msg::LaserScan>(item->getSensorName(), 1);
			std::string sensorname = p_publisher_map[id]->get_topic_name();
			if (sensorname.at(0) == '/') {
				sensorname = sensorname.erase(0, 1);
			}
			p_sensor_names[id] = sensorname;
			p_sensor_max_range[id] = item->getMaximumRange();
			p_sensor_min_range[id] = item->getMinimumRange();
			try {
				p_tf_map[id].child_frame_id = p_sensor_names[id];
			} catch (std::exception &e) {
			}
		}
	}
	p_query_state = 2;
}

void RangeSensorClient_ReceiveFSM::handleReportRangeSensorCompressedDataAction(ReportRangeSensorCompressedData msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
	RCLCPP_WARN(logger, "handleReportRangeSensorCompressedDataAction not implemented!");
}

void RangeSensorClient_ReceiveFSM::handleReportRangeSensorConfigurationAction(ReportRangeSensorConfiguration msg, Receive::Body::ReceiveRec transportData)
{
	uint16_t subsystem_id = transportData.getSrcSubsystemID();
	uint8_t node_id = transportData.getSrcNodeID();
	uint8_t component_id = transportData.getSrcComponentID();
	JausAddress sender(subsystem_id, node_id, component_id);
	RCLCPP_DEBUG(logger, "ReportRangeSensorConfiguration, count sensors: %d", msg.getBody()->getRangeSensorConfigurationList()->getNumberOfElements());
	RCLCPP_INFO(logger, "request GeometricProperties and SensorCapabilities from %s", sender.str().c_str());
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
				iop::Timestamp stamp = cmp->from_iop(ts->getDay(), ts->getHour(), ts->getMinutes(), ts->getSeconds(), ts->getMilliseconds());
				try {
					geometry_msgs::msg::TransformStamped& tf_msg  = p_tf_map.at(id);
					if (! tf_msg.child_frame_id.empty()) {
						tf_msg.header.stamp = stamp.ros_time;
						if (tf_msg.header.frame_id.empty()) {
							tf_msg.header.frame_id = this->p_tf_frame_robot;
						}
						p_tf_broadcaster.sendTransform(tf_msg);
					}
				} catch (std::exception &e) {
					RCLCPP_WARN(logger, "can not publish tf for sensor %s: %s", sensor_id.c_str(), e.what());
				}
				auto rosmsg = sensor_msgs::msg::LaserScan();
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
				p_publisher_map[id]->publish(rosmsg);
			} else {
				// no capabilities received for this sensor id
				RCLCPP_WARN(logger, "no capabilities received for this sensor id: %d", id);
			}
		} else {
		  // it is an error
		  RCLCPP_WARN(logger, "error rec for ID: %d received, code: %d, msg: %s", itemvar->getRangeSensorDataErrorRec()->getSensorID(), itemvar->getRangeSensorDataErrorRec()->getDataErrorCode(), itemvar->getRangeSensorDataErrorRec()->getErrorMessage().c_str());
		}
	}
	RCLCPP_DEBUG(logger, "ReportRangeSensorData, count sensors: %d", msg.getBody()->getRangeSensorDataList()->getNumberOfElements());
}

void RangeSensorClient_ReceiveFSM::handleReportSensorGeometricPropertiesAction(ReportSensorGeometricProperties msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
	RCLCPP_DEBUG(logger, "ReportSensorGeometricProperties, count sensors: %d", msg.getBody()->getGeometricPropertiesList()->getNumberOfElements());
	// create tf data for all valid sensors positions
	for (unsigned int i = 0; i < msg.getBody()->getGeometricPropertiesList()->getNumberOfElements(); i++) {
		try {
			ReportSensorGeometricProperties::Body::GeometricPropertiesList::GeometricPropertiesSequence *element = msg.getBody()->getGeometricPropertiesList()->getElement(i);
			jUnsignedShortInteger sensor_id = element->getSensorIdRec()->getSensorID();
			// create and publish TF message
			ReportSensorGeometricProperties::Body::GeometricPropertiesList::GeometricPropertiesSequence::GeometricPropertiesVariant::StaticGeometricPropertiesRec *staticgeo = element->getGeometricPropertiesVariant()->getStaticGeometricPropertiesRec();
			auto msg = geometry_msgs::msg::TransformStamped();
			msg.transform.translation.x = staticgeo->getSensorPosition()->getPositionVectorElement(0);
			msg.transform.translation.y = staticgeo->getSensorPosition()->getPositionVectorElement(1);
			msg.transform.translation.z = staticgeo->getSensorPosition()->getPositionVectorElement(2);
			msg.transform.rotation.x = staticgeo->getUnitQuaternion()->getUnitQuaternionElement(0);
			msg.transform.rotation.y = staticgeo->getUnitQuaternion()->getUnitQuaternionElement(1);
			msg.transform.rotation.z = staticgeo->getUnitQuaternion()->getUnitQuaternionElement(2);
			msg.transform.rotation.w = staticgeo->getUnitQuaternion()->getUnitQuaternionElement(3);
			msg.header.stamp = cmp->now();
			msg.header.frame_id = this->p_tf_frame_robot;
			msg.child_frame_id = p_sensor_names[sensor_id];
			RCLCPP_DEBUG(logger, "initialized tf %s -> %s", this->p_tf_frame_robot.c_str(), p_sensor_names[sensor_id].c_str());
			p_tf_map[sensor_id] = msg;
		} catch (std::exception &e) {
			RCLCPP_WARN(logger, "can not create tf for sensor: %s", e.what());
		}
	}
}





}
