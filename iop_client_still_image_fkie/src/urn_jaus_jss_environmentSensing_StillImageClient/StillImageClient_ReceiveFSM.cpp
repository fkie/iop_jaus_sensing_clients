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

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "opencv2/core/version.hpp"

#include <iop_ocu_slavelib_fkie/Slave.h>


#include "urn_jaus_jss_environmentSensing_StillImageClient/StillImageClient_ReceiveFSM.h"




using namespace JTS;
using namespace iop;

namespace urn_jaus_jss_environmentSensing_StillImageClient
{



StillImageClient_ReceiveFSM::StillImageClient_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM, urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM, urn_jaus_jss_environmentSensing_VisualSensorClient::VisualSensorClient_ReceiveFSM* pVisualSensorClient_ReceiveFSM)
: p_cfg("~StillImageClient")
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new StillImageClient_ReceiveFSMContext(*this);

	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->pEventsClient_ReceiveFSM = pEventsClient_ReceiveFSM;
	this->pAccessControlClient_ReceiveFSM = pAccessControlClient_ReceiveFSM;
	this->pVisualSensorClient_ReceiveFSM = pVisualSensorClient_ReceiveFSM;
	p_has_access = false;
	p_query_state = 0;
	p_by_query = false;
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
	p_cfg.param("hz", p_hz, p_hz, false, false);
	p_cfg.param("use_id_for_topics", p_use_id_for_topics, p_use_id_for_topics);
	ocu::Slave &slave = ocu::Slave::get_instance(*(jausRouter->getJausAddress()));
	slave.add_supported_service(*this, "urn:jaus:jss:environmentSensing:StillImage", 1, 0);

}

void StillImageClient_ReceiveFSM::control_allowed(std::string service_uri, JausAddress component, unsigned char authority)
{
	if (service_uri.compare("urn:jaus:jss:environmentSensing:StillImage") == 0) {
		p_has_access = true;
		p_remote_addr = component;
		ROS_INFO_NAMED("StillImageClient", "access granted for %d.%d.%d",
				component.getSubsystemID(), component.getNodeID(), component.getComponentID());
	} else {
		ROS_WARN_STREAM("[StillImageClient] unexpected control allowed for " << service_uri << " received, ignored!");
	}
}

void StillImageClient_ReceiveFSM::enable_monitoring_only(std::string service_uri, JausAddress component)
{
	ROS_INFO_NAMED("StillImageClient", "monitor enabled for %s", component.str().c_str());
	p_remote_addr = component;
}

void StillImageClient_ReceiveFSM::access_deactivated(std::string service_uri, JausAddress component)
{
	p_has_access = false;
	p_remote_addr = JausAddress(0);
	ROS_INFO_NAMED("StillImageClient", "access released for %s", component.str().c_str());
}

void StillImageClient_ReceiveFSM::create_events(std::string service_uri, JausAddress component, bool by_query)
{
	p_by_query = by_query;
	sendJausMessage(p_query_cfg, component);
	p_query_timer = p_nh.createTimer(ros::Duration(3), &StillImageClient_ReceiveFSM::pQueryCallback, this);
}

void StillImageClient_ReceiveFSM::cancel_events(std::string service_uri, JausAddress component, bool by_query)
{
	std::vector<unsigned int> tmp(p_requested_sensors);
	for (unsigned int i = 0; i < tmp.size(); i++) {
		p_disconnect_from_service(tmp[i]);
	}
	p_query_state = 0;
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


void StillImageClient_ReceiveFSM::pQueryCallback(const ros::TimerEvent& event)
{
	if (p_remote_addr.get() != 0) {
		if (p_query_state == 0) {
			sendJausMessage(p_query_cfg, p_remote_addr);
		} else if (p_query_state == 1) {
			sendJausMessage(p_query_cap, p_remote_addr);
		} else {
			sendJausMessage(p_query_image_data, p_remote_addr);
		}
	}
}

void StillImageClient_ReceiveFSM::handleReportStillImageDataAction(ReportStillImageData msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
	JausAddress sender = transportData.getAddress();
	ROS_DEBUG_NAMED("StillImageClient", "received still image data from %s", sender.str().c_str());
	for (unsigned int i = 0; i < msg.getBody()->getStillImageDataList()->getNumberOfElements(); i++) {
		ReportStillImageData::Body::StillImageDataList::StillImageDataRec *rec = msg.getBody()->getStillImageDataList()->getElement(i);
		unsigned short int id = rec->getSensorID();
		if (!(rec->getImageFrame()->getFormat() == 0 || rec->getImageFrame()->getFormat() == 2)) {
			ROS_WARN_NAMED("StillImageClient", "received unsupported still image format '%s', id: %d from %s",
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
			cv::Mat d_img = cv::imdecode(m_img, CV_LOAD_IMAGE_COLOR);
			if( !d_img.data ) {
				ROS_WARN_NAMED("StillImageClient", "received invalid image data from %s", sender.str().c_str());
			} else {
				cv_bridge::CvImage img;
				img.image = d_img;
				img.header.stamp = ros::Time::now();
				img.encoding = sensor_msgs::image_encodings::BGR8;
				sensor_msgs::CameraInfo cam_info;
				cam_info.header.stamp = ros::Time::now();
				cam_info.header.frame_id = it->second.getTopic();
				cam_info.height = d_img.rows;
				cam_info.width = d_img.cols;
				cam_info.distortion_model = "plumb_bob";
				sensor_msgs::CameraInfo::ConstPtr cam_info_ptr = boost::make_shared<sensor_msgs::CameraInfo>(cam_info);
				it->second.publish(img.toImageMsg(), cam_info_ptr);
			}
		}
	}
}

void StillImageClient_ReceiveFSM::handleReportStillImageSensorCapabilitiesAction(ReportStillImageSensorCapabilities msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
	ROS_WARN_NAMED("StillImageClient", "handleReportStillImageSensorCapabilitiesAction not implemented!");
	p_query_state = 2;
}

void StillImageClient_ReceiveFSM::handleReportStillImageSensorConfigurationAction(ReportStillImageSensorConfiguration msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
	JausAddress sender = transportData.getAddress();
	ROS_INFO_NAMED("StillImageClient", "received still image configuration from %s", sender.str().c_str());
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
				ros::NodeHandle nh;
				image_transport::ImageTransport it(nh);
				ROS_DEBUG_NAMED("StillImageClient", "create image transport publisher for %s", sensor_name.c_str());
				p_publisher_map[id] = it.advertiseCamera(sensor_name, 1,
						boost::bind(&StillImageClient_ReceiveFSM::pConnectImageCallback, this, _1),
						boost::bind(&StillImageClient_ReceiveFSM::pDisconnectImageCallback, this, _1));
				p_topic_map[p_publisher_map[id].getTopic()] = id;
			} else {
				ROS_WARN_NAMED("StillImageClient", "No name in VisualSensor for sensor %d from %s defined", id, sender.str().c_str());
			}
		}
	}
	p_query_state = 2;
	// create event or timer for queries
	if (p_remote_addr.get() != 0) {
		if (!p_lazy) {
		}
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
	if (p_remote_addr.get() != 0) {
		// add sensor id to query
		QueryStillImageData::Body::QueryStillImageDataList::QueryStillImageDataRec drec;
		drec.setSensorID(id); // 0 is specified to get all sensors
		drec.setReportCoordinateSystem(1); // we use vehicle coordinate system
		p_query_image_data.getBody()->getQueryStillImageDataList()->addElement(drec);
		ROS_INFO_NAMED("StillImageClient", "update query message to get still image with id: %d ", id);
		if (p_requested_sensors.size() == 0) {
			if (p_by_query) {
				p_query_timer.stop();
				if (p_hz > 0) {
					ROS_INFO_NAMED("StillImageClient", "create QUERY timer to get still image @%.2fHz from %s", p_hz, p_remote_addr.str().c_str());
					p_query_timer = p_nh.createTimer(ros::Duration(1.0 / p_hz), &StillImageClient_ReceiveFSM::pQueryCallback, this);
				}
			}
		}
		if (p_requested_sensors.size() == 0) {
			ROS_INFO_NAMED("StillImageClient", "create EVENT to get still image %d @%.2fHz from %s", id, p_hz, p_remote_addr.str().c_str());
		} else {
			ROS_INFO_NAMED("StillImageClient", "update EVENT to get still image %d @%.2fHz from %s", id, p_hz, p_remote_addr.str().c_str());
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
		ROS_INFO_NAMED("StillImageClient", "update query to remove still image %d @%.2fHz from %s", id, p_hz, p_remote_addr.str().c_str());
		if (qlist.getNumberOfElements() == 0) {
			p_query_timer.stop();
			if (!p_by_query) {
				ROS_INFO_NAMED("StillImageClient", "cancel EVENT for still image for %s", p_remote_addr.str().c_str());
				pEventsClient_ReceiveFSM->cancel_event(*this, p_remote_addr, p_query_image_data);
			}
		}
	}
}

};
