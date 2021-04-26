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


#ifndef STILLIMAGECLIENT_RECEIVEFSM_H
#define STILLIMAGECLIENT_RECEIVEFSM_H

#include "JausUtils.h"
#include "InternalEvents/InternalEventHandler.h"
#include "Transport/JausTransport.h"
#include "JTSStateMachine.h"
#include "urn_jaus_jss_environmentSensing_StillImageClient/Messages/MessageSet.h"
#include "urn_jaus_jss_environmentSensing_StillImageClient/InternalEvents/InternalEventsSet.h"

#include "InternalEvents/Receive.h"
#include "InternalEvents/Send.h"

#include "urn_jaus_jss_core_Transport/Transport_ReceiveFSM.h"
#include "urn_jaus_jss_core_EventsClient/EventsClient_ReceiveFSM.h"
#include "urn_jaus_jss_core_AccessControlClient/AccessControlClient_ReceiveFSM.h"
#include "urn_jaus_jss_environmentSensing_VisualSensorClient/VisualSensorClient_ReceiveFSM.h"

#include "StillImageClient_ReceiveFSM_sm.h"
#include <rclcpp/rclcpp.hpp>
#include <fkie_iop_component/iop_component.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <image_transport/image_transport.hpp>
#include <fkie_iop_ocu_slavelib/SlaveHandlerInterface.h>
#include <fkie_iop_events/EventHandlerInterface.h>

namespace urn_jaus_jss_environmentSensing_StillImageClient
{

class DllExport StillImageClient_ReceiveFSM : public JTS::StateMachine, public iop::ocu::SlaveHandlerInterface, public iop::EventHandlerInterface
{
public:
	StillImageClient_ReceiveFSM(std::shared_ptr<iop::Component> cmp, urn_jaus_jss_environmentSensing_VisualSensorClient::VisualSensorClient_ReceiveFSM* pVisualSensorClient_ReceiveFSM, urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM, urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM);
	virtual ~StillImageClient_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();
	virtual void setupIopConfiguration();

	/// Action Methods
	virtual void handleReportStillImageDataAction(ReportStillImageData msg, Receive::Body::ReceiveRec transportData);
	virtual void handleReportStillImageSensorCapabilitiesAction(ReportStillImageSensorCapabilities msg, Receive::Body::ReceiveRec transportData);
	virtual void handleReportStillImageSensorConfigurationAction(ReportStillImageSensorConfiguration msg, Receive::Body::ReceiveRec transportData);

	/// EventHandlerInterface Methods
	void event(JausAddress reporter, unsigned short query_msg_id, unsigned int reportlen, const unsigned char* reportdata);

	/// SlaveHandlerInterface Methods
	void register_events(JausAddress remote_addr, double hz);
	void unregister_events(JausAddress remote_addr);
	void send_query(JausAddress remote_addr);
	void stop_query(JausAddress remote_addr);
	/// Guard Methods



	StillImageClient_ReceiveFSMContext *context;

protected:

	/// References to parent FSMs
	urn_jaus_jss_environmentSensing_VisualSensorClient::VisualSensorClient_ReceiveFSM* pVisualSensorClient_ReceiveFSM;
	urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM;
	urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM;
	urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;

	std::shared_ptr<iop::Component> cmp;
	rclcpp::Logger logger;

	std::map<unsigned int, image_transport::CameraPublisher> p_publisher_map;  // sensor id, publisher
	std::map<std::string, unsigned int> p_topic_map;  // topic name, sensor id
	std::vector<unsigned int> p_requested_sensors;  // sensor id
	int p_query_state;
	double p_hz;
	bool p_lazy;
	bool p_use_id_for_topics;

	QueryStillImageSensorConfiguration p_query_cfg;
	QueryStillImageSensorCapabilities p_query_cap;
	QueryStillImageData p_query_image_data;

	std::string get_image_format(unsigned short format);

	void p_connect_to_service(unsigned int id);
	void p_disconnect_from_service(unsigned int id);
	void pConnectImageCallback(const image_transport::SingleSubscriberPublisher& pub);
	void pDisconnectImageCallback(const image_transport::SingleSubscriberPublisher& pub);
};

}

#endif // STILLIMAGECLIENT_RECEIVEFSM_H
