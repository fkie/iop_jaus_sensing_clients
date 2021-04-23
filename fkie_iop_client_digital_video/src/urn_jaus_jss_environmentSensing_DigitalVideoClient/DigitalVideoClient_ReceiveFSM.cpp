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


#include "urn_jaus_jss_environmentSensing_DigitalVideoClient/DigitalVideoClient_ReceiveFSM.h"
#include <fkie_iop_component/iop_config.hpp>
#include <fkie_iop_ocu_slavelib/Slave.h>




using namespace JTS;
using namespace iop;

namespace urn_jaus_jss_environmentSensing_DigitalVideoClient
{



DigitalVideoClient_ReceiveFSM::DigitalVideoClient_ReceiveFSM(std::shared_ptr<iop::Component> cmp, urn_jaus_jss_environmentSensing_VisualSensorClient::VisualSensorClient_ReceiveFSM* pVisualSensorClient_ReceiveFSM, urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM, urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM)
: logger(cmp->get_logger().get_child("DigitalVideoClient"))
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new DigitalVideoClient_ReceiveFSMContext(*this);

	this->pVisualSensorClient_ReceiveFSM = pVisualSensorClient_ReceiveFSM;
	this->pAccessControlClient_ReceiveFSM = pAccessControlClient_ReceiveFSM;
	this->pEventsClient_ReceiveFSM = pEventsClient_ReceiveFSM;
	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->cmp = cmp;
	p_current_resource_id = 65535;
	p_has_access = false;
}



DigitalVideoClient_ReceiveFSM::~DigitalVideoClient_ReceiveFSM()
{
	delete context;
}

void DigitalVideoClient_ReceiveFSM::setupNotifications()
{
	pVisualSensorClient_ReceiveFSM->registerNotification("Receiving_Ready", ieHandler, "InternalStateChange_To_DigitalVideoClient_ReceiveFSM_Receiving_Ready", "VisualSensorClient_ReceiveFSM");
	pVisualSensorClient_ReceiveFSM->registerNotification("Receiving", ieHandler, "InternalStateChange_To_DigitalVideoClient_ReceiveFSM_Receiving_Ready", "VisualSensorClient_ReceiveFSM");
	registerNotification("Receiving_Ready", pVisualSensorClient_ReceiveFSM->getHandler(), "InternalStateChange_To_VisualSensorClient_ReceiveFSM_Receiving_Ready", "DigitalVideoClient_ReceiveFSM");
	registerNotification("Receiving", pVisualSensorClient_ReceiveFSM->getHandler(), "InternalStateChange_To_VisualSensorClient_ReceiveFSM_Receiving", "DigitalVideoClient_ReceiveFSM");

}


void DigitalVideoClient_ReceiveFSM::setupIopConfiguration()
{
	iop::Config cfg(cmp, "DigitalVideoClient");
	p_sub_cur_dv_id = cfg.create_subscription<std_msgs::msg::UInt16>("dv_resource_id", 10, std::bind(&DigitalVideoClient_ReceiveFSM::p_dandle_current_ressource_id, this, std::placeholders::_1));
	auto slave = ocu::Slave::get_instance(cmp);
	slave->add_supported_service(*this, "urn:jaus:jss:environmentSensing:DigitalVideo", 1, 0);

}

void DigitalVideoClient_ReceiveFSM::control_allowed(std::string service_uri, JausAddress component, unsigned char authority)
{
	if (service_uri.compare("urn:jaus:jss:environmentSensing:DigitalVideo") == 0) {
		p_has_access = true;
		p_remote_addr = component;
		RCLCPP_INFO(logger, "access granted for %d.%d.%d",
				component.getSubsystemID(), component.getNodeID(), component.getComponentID());
	} else {
		RCLCPP_WARN(logger, "unexpected control allowed for %s received, ignored!", service_uri.c_str());
	}
}

void DigitalVideoClient_ReceiveFSM::enable_monitoring_only(std::string service_uri, JausAddress component)
{
	RCLCPP_INFO(logger, "monitor enabled for %d.%d.%d",
			component.getSubsystemID(), component.getNodeID(), component.getComponentID());
}

void DigitalVideoClient_ReceiveFSM::access_deactivated(std::string service_uri, JausAddress component)
{
	p_has_access = false;
	p_remote_addr = JausAddress(0);
	RCLCPP_INFO(logger, "access released for %d.%d.%d",
			component.getSubsystemID(), component.getNodeID(), component.getComponentID());
}

void DigitalVideoClient_ReceiveFSM::create_events(std::string service_uri, JausAddress component, bool by_query)
{
}

void DigitalVideoClient_ReceiveFSM::cancel_events(std::string service_uri, JausAddress component, bool by_query)
{
}


void DigitalVideoClient_ReceiveFSM::handleReportDigitalVideoSensorCapabilitiesAction(ReportDigitalVideoSensorCapabilities msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
	printf("[DigitalVideoClient] handleReportDigitalVideoSensorCapabilitiesAction not implemented\n");
}

void DigitalVideoClient_ReceiveFSM::handleReportDigitalVideoSensorConfigurationAction(ReportDigitalVideoSensorConfiguration msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
	printf("[DigitalVideoClient] handleReportDigitalVideoSensorConfigurationAction not implemented\n");
}

void DigitalVideoClient_ReceiveFSM::p_dandle_current_ressource_id(const std_msgs::msg::UInt16::SharedPtr msg)
{
	if (p_remote_addr.get() != 0) {
		if (msg->data == 65535 and p_current_resource_id != 65535) {
			ControlDigitalVideoSensorStream cmd;
			RCLCPP_INFO(logger, "forward STOP for resource id %d to %d.%d.%d",
					p_current_resource_id, p_remote_addr.getSubsystemID(), p_remote_addr.getNodeID(), p_remote_addr.getComponentID());
			cmd.getBody()->getControlDigitalVideoSensorStreamRec()->setSensorID(p_current_resource_id);
			cmd.getBody()->getControlDigitalVideoSensorStreamRec()->setStreamState(2);
			sendJausMessage(cmd, p_remote_addr);
		} else {
			ControlDigitalVideoSensorStream cmd;
			RCLCPP_INFO(logger, "forward PLAY for resource id %d to %d.%d.%d",
					msg->data, p_remote_addr.getSubsystemID(), p_remote_addr.getNodeID(), p_remote_addr.getComponentID());
			cmd.getBody()->getControlDigitalVideoSensorStreamRec()->setSensorID(msg->data);
			cmd.getBody()->getControlDigitalVideoSensorStreamRec()->setStreamState(0);
			sendJausMessage(cmd, p_remote_addr);
		}
	}
	p_current_resource_id = msg->data;
}




}
