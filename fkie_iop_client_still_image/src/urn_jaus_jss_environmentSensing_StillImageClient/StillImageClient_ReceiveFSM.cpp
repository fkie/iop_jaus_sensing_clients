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

#include "urn_jaus_jss_environmentSensing_StillImageClient/StillImageClient_ReceiveFSM.h"
#include <fkie_iop_component/iop_config.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
// #include <opencv2/core/version.hpp>


using namespace JTS;

namespace urn_jaus_jss_environmentSensing_StillImageClient
{



StillImageClient_ReceiveFSM::StillImageClient_ReceiveFSM(std::shared_ptr<iop::Component> cmp, urn_jaus_jss_environmentSensing_VisualSensorClient::VisualSensorClient_ReceiveFSM* pVisualSensorClient_ReceiveFSM, urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM, urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM)
: SlaveHandlerInterface(cmp, "StillImageClient", 10.0),
  logger(cmp->get_logger().get_child("StillImageClient"))
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new StillImageClient_ReceiveFSMContext(*this);

	this->pVisualSensorClient_ReceiveFSM = pVisualSensorClient_ReceiveFSM;
	this->pAccessControlClient_ReceiveFSM = pAccessControlClient_ReceiveFSM;
	this->pEventsClient_ReceiveFSM = pEventsClient_ReceiveFSM;
	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->cmp = cmp;
	p_query_state = 0;
	p_hz = 1;
	p_use_id_for_topics = true;

	QueryStillImageSensorConfiguration::Body::QueryStillImageSensorConfigurationList::QueryStillImageSensorConfigurationRec crec;
	crec.setSensorID(0); // 0 is specified to get all sensors
	crec.setQueryPresenceVector(255);
	p_query_cfg.getBody()->getQueryStillImageSensorConfigurationList()->addElement(crec);

	QueryStillImageSensorCapabilities::Body::QueryStillImageSensorCapabilitiesList::QueryStillImageSensorCapabilitiesRec rscrec;
	rscrec.setSensorID(0); // 0 is specified to get all sensors
	rscrec.setQueryPresenceVector(255);
	p_query_cap.getBody()->getQueryStillImageSensorCapabilitiesList()->addElement(rscrec);
}

StillImageClient_ReceiveFSM::~StillImageClient_ReceiveFSM()
{
	delete context;
}

void StillImageClient_ReceiveFSM::setupNotifications()
{
	pVisualSensorClient_ReceiveFSM->registerNotification("Receiving_Ready", ieHandler, "InternalStateChange_To_StillImageClient_ReceiveFSM_Receiving_Ready", "VisualSensorClient_ReceiveFSM");
	pVisualSensorClient_ReceiveFSM->registerNotification("Receiving", ieHandler, "InternalStateChange_To_StillImageClient_ReceiveFSM_Receiving_Ready", "VisualSensorClient_ReceiveFSM");
	registerNotification("Receiving_Ready", pVisualSensorClient_ReceiveFSM->getHandler(), "InternalStateChange_To_VisualSensorClient_ReceiveFSM_Receiving_Ready", "StillImageClient_ReceiveFSM");
	registerNotification("Receiving", pVisualSensorClient_ReceiveFSM->getHandler(), "InternalStateChange_To_VisualSensorClient_ReceiveFSM_Receiving", "StillImageClient_ReceiveFSM");

}


void StillImageClient_ReceiveFSM::setupIopConfiguration()
{
	iop::Config cfg(cmp, "StillImageClient");
	cfg.declare_param<bool>("use_id_for_topics", p_use_id_for_topics, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_BOOL,
		"Allows to use ID if no name for sensor is available. In other case a warning will be printed!",
		"Default: true");
	cfg.declare_param<double>("hz", p_hz, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE,
		"Sets how often the reports are requested. If use_queries is True hz must be greather then 0. In this case each time a Query message is sent to get a report. If use_queries is False an event is created to get Reports. In this case 0 disables the rate and an event of type on_change will be created.",
		"Default: 10.0");
	cfg.param("hz", p_hz, p_hz, false);
	cfg.param("use_id_for_topics", p_use_id_for_topics, p_use_id_for_topics);
	// initialize the control layer, which handles the access control staff
	this->set_rate(p_hz);
	this->set_supported_service(*this, "urn:jaus:jss:environmentSensing:StillImage", 1, 0);
	this->set_event_name("image configuration");
	this->set_query_before_event(true, 1.0);
}

void StillImageClient_ReceiveFSM::register_events(JausAddress remote_addr, double hz)
{
	// pEventsClient_ReceiveFSM->create_event(*this, remote_addr, p_query_image_data, p_hz);
}

void StillImageClient_ReceiveFSM::unregister_events(JausAddress remote_addr)
{
	// pEventsClient_ReceiveFSM->cancel_event(*this, remote_addr, p_query_image_data);
	stop_query(remote_addr);
}

void StillImageClient_ReceiveFSM::send_query(JausAddress remote_addr)
{
	if (p_query_state == 0) {
		sendJausMessage(p_query_cfg, remote_addr);
	} else if (p_query_state == 1) {
		sendJausMessage(p_query_cap, remote_addr);
	} else {
		sendJausMessage(p_query_image_data, remote_addr);
	}
}

void StillImageClient_ReceiveFSM::stop_query(JausAddress remote_addr)
{
	p_query_state = 0;
	std::vector<unsigned int> tmp(p_requested_sensors);
	for (unsigned int i = 0; i < tmp.size(); i++) {
		p_disconnect_from_service(tmp[i]);
	}
}

void StillImageClient_ReceiveFSM::event(JausAddress reporter, unsigned short query_msg_id, unsigned int reportlen, const unsigned char* reportdata)
{
	ReportStillImageData report;
	report.decode(reportdata);
	Receive::Body::ReceiveRec transport_data;
	transport_data.setSrcSubsystemID(reporter.getSubsystemID());
	transport_data.setSrcNodeID(reporter.getNodeID());
	transport_data.setSrcComponentID(reporter.getComponentID());
	handleReportStillImageDataAction(report, transport_data);
}

void StillImageClient_ReceiveFSM::handleReportStillImageDataAction(ReportStillImageData msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
	JausAddress sender = transportData.getAddress();
	RCLCPP_DEBUG(logger, "received still image data from %s", sender.str().c_str());
	for (unsigned int i = 0; i < msg.getBody()->getStillImageDataList()->getNumberOfElements(); i++) {
		ReportStillImageData::Body::StillImageDataList::StillImageDataRec *rec = msg.getBody()->getStillImageDataList()->getElement(i);
		unsigned short int id = rec->getSensorID();
		if (!(rec->getImageFrame()->getFormat() == 0 || rec->getImageFrame()->getFormat() == 2)) {
			RCLCPP_WARN(logger, "received unsupported still image format '%s', id: %d from %s",
					get_image_format(rec->getImageFrame()->getFormat()).c_str(), id, sender.str().c_str());
			continue;
		}
		std::map<unsigned int, image_transport::CameraPublisher>::iterator it = p_publisher_map.find(id);
		if (it != p_publisher_map.end()) {
//#if CV_MAJOR_VERSION == 2
//// do opencv 2 code
//#elif CV_MAJOR_VERSION == 3
//// do opencv 3 code
//#endif
			std::vector<unsigned char> data(rec->getImageFrame()->getData(), rec->getImageFrame()->getData() + rec->getImageFrame()->getLength());
			cv::Mat m_img(data);
			cv::Mat d_img = cv::imdecode(m_img, 1); //CV_LOAD_IMAGE_COLOR);
			if( !d_img.data ) {
				RCLCPP_WARN(logger, "received invalid image data from %s", sender.str().c_str());
			} else {
				cv_bridge::CvImage img;
				img.image = d_img;
				img.header.stamp = cmp->now();
				img.encoding = sensor_msgs::image_encodings::BGR8;
				auto cam_info = sensor_msgs::msg::CameraInfo();
				cam_info.header.stamp = cmp->now();
				cam_info.header.frame_id = it->second.getTopic();
				cam_info.height = d_img.rows;
				cam_info.width = d_img.cols;
				cam_info.distortion_model = "plumb_bob";
				auto cam_info_ptr = std::make_shared<sensor_msgs::msg::CameraInfo>(cam_info);
				it->second.publish(img.toImageMsg(), cam_info_ptr);
			}
		}
	}
}

void StillImageClient_ReceiveFSM::handleReportStillImageSensorCapabilitiesAction(ReportStillImageSensorCapabilities msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
	RCLCPP_WARN(logger, "handleReportStillImageSensorCapabilitiesAction not implemented!");
	p_query_state = 2;
}

void StillImageClient_ReceiveFSM::handleReportStillImageSensorConfigurationAction(ReportStillImageSensorConfiguration msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
	JausAddress sender = transportData.getAddress();
	RCLCPP_INFO(logger, "received still image configuration from %s", sender.str().c_str());
	// create for each sensor a publisher
	for (unsigned int i = 0; i < msg.getBody()->getStillImageSensorConfigurationList()->getNumberOfElements(); i++) {
		ReportStillImageSensorConfiguration::Body::StillImageSensorConfigurationList::StillImageSensorConfigurationRec *item = msg.getBody()->getStillImageSensorConfigurationList()->getElement(i);
		unsigned int id = item->getSensorID();
		std::string sensor_name;
		sensor_name = pVisualSensorClient_ReceiveFSM->get_sensor_name(sender, id);
		if (sensor_name.empty() && p_use_id_for_topics) {
			std::ostringstream s;
			s << id;
			sensor_name = s.str();
		}
		if (p_publisher_map.find(id) == p_publisher_map.end()) {
			if (!sensor_name.empty()) {
				image_transport::ImageTransport it(cmp);
				RCLCPP_DEBUG(logger, "create image transport publisher for %s", sensor_name.c_str());
				// TODO(ros2) Implement when SubscriberStatusCallback is available
				p_publisher_map[id] = it.advertiseCamera(sensor_name, 1);
				p_connect_to_service(id);
				// p_publisher_map[id] = it.advertiseCamera(sensor_name, 1,
				// 		std::bind(&StillImageClient_ReceiveFSM::pConnectImageCallback, this, std::placeholders::_1),
				// 		std::bind(&StillImageClient_ReceiveFSM::pDisconnectImageCallback, this, std::placeholders::_1));
				p_topic_map[p_publisher_map[id].getTopic()] = id;
			} else {
				RCLCPP_WARN(logger, "No name in VisualSensor for sensor %d from %s defined", id, sender.str().c_str());
			}
		}
	}
	this->set_event_name("image capabilities");
	p_query_state = 1;
	// create event or timer for queries
	if (!p_lazy) {
	}
}

std::string StillImageClient_ReceiveFSM::get_image_format(unsigned short format)
{
	// convert byte type to string format
	switch(format) {
	case 0: return "jpeg";
	case 1: return "gif";
	case 2: return "png";
	case 3: return "bmp";
	case 4: return "tiff";
	case 5: return "ppm";
	case 6: return "pgm";
	case 7: return "pnm";
	case 8: return "nef";
	case 9: return "cr_2";
	case 10: return "dng";
	}
	return "";
}

void StillImageClient_ReceiveFSM::pConnectImageCallback(const image_transport::SingleSubscriberPublisher& pub)
{
	std::map<std::string, unsigned int>::iterator it = p_topic_map.find(pub.getTopic());
	if (it != p_topic_map.end()) {
		p_connect_to_service(it->second);
	}
}

void StillImageClient_ReceiveFSM::pDisconnectImageCallback(const image_transport::SingleSubscriberPublisher& pub)
{
	std::map<std::string, unsigned int>::iterator it = p_topic_map.find(pub.getTopic());
	if (it != p_topic_map.end()) {
		p_disconnect_from_service(it->second);
	}
}


void StillImageClient_ReceiveFSM::p_connect_to_service(unsigned int id)
{
	if (has_remote_addr()) {
		// add sensor id to query
		QueryStillImageData::Body::QueryStillImageDataList::QueryStillImageDataRec drec;
		drec.setSensorID(id); // 0 is specified to get all sensors
		drec.setReportCoordinateSystem(1); // we use vehicle coordinate system
		p_query_image_data.getBody()->getQueryStillImageDataList()->addElement(drec);
		RCLCPP_INFO(logger, "update query message to get still image with id: %d ", id);
		if (p_requested_sensors.size() == 0) {
			if (p_by_query) {
				p_timer.stop();
				if (p_hz > 0) {
					RCLCPP_INFO(logger, "create QUERY timer to get still image @%.2fHz from %s", p_hz, p_remote_addr.str().c_str());
					p_timer.set_rate(p_hz);
					p_timer.start();
				}
			}
		}
		if (p_requested_sensors.size() == 0) {
			RCLCPP_INFO(logger, "create EVENT to get still image %d @%.2fHz from %s", id, p_hz, p_remote_addr.str().c_str());
		} else {
			RCLCPP_INFO(logger, "update EVENT to get still image %d @%.2fHz from %s", id, p_hz, p_remote_addr.str().c_str());
		}
		pEventsClient_ReceiveFSM->create_event(*this, p_remote_addr, p_query_image_data, p_hz);
		p_requested_sensors.push_back(id);
	}
}

void StillImageClient_ReceiveFSM::p_disconnect_from_service(unsigned int id)
{
	// remove sensor id from query
	std::vector<unsigned int>::iterator it = std::find(p_requested_sensors.begin(), p_requested_sensors.end(), id);
	if (it != p_requested_sensors.end()) {
		p_requested_sensors.erase(it);
		QueryStillImageData::Body::QueryStillImageDataList qlist;
		for (unsigned int i = 0; i < p_requested_sensors.size(); i++) {
			QueryStillImageData::Body::QueryStillImageDataList::QueryStillImageDataRec drec;
			drec.setSensorID(p_requested_sensors[i]); // 0 is specified to get all sensors
			drec.setReportCoordinateSystem(1); // we use vehicle coordinate system
			qlist.addElement(drec);
		}
		p_query_image_data.getBody()->setQueryStillImageDataList(qlist);
		RCLCPP_INFO(logger, "update query to remove still image %d @%.2fHz from %s", id, p_hz, p_remote_addr.str().c_str());
		if (qlist.getNumberOfElements() == 0) {
			p_timer.stop();
			if (!p_by_query) {
				RCLCPP_INFO(logger, "cancel EVENT for still image for %s", p_remote_addr.str().c_str());
				pEventsClient_ReceiveFSM->cancel_event(*this, p_remote_addr, p_query_image_data);
			}
		}
	}
}

}
