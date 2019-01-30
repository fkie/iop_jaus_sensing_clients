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


#include <fkie_iop_ocu_slavelib/Slave.h>
#include <fkie_iop_component/iop_config.h>

#include "urn_jaus_jss_environmentSensing_DigitalVideoClient/DigitalVideoClient_ReceiveFSM.h"




using namespace JTS;
using namespace iop;

namespace urn_jaus_jss_environmentSensing_DigitalVideoClient
{



DigitalVideoClient_ReceiveFSM::DigitalVideoClient_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM, urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM, urn_jaus_jss_environmentSensing_VisualSensorClient::VisualSensorClient_ReceiveFSM* pVisualSensorClient_ReceiveFSM)
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new DigitalVideoClient_ReceiveFSMContext(*this);

	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->pEventsClient_ReceiveFSM = pEventsClient_ReceiveFSM;
	this->pAccessControlClient_ReceiveFSM = pAccessControlClient_ReceiveFSM;
	this->pVisualSensorClient_ReceiveFSM = pVisualSensorClient_ReceiveFSM;
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
	iop::Config cfg("~DigitalVideoClient");
	p_sub_cur_dv_id = cfg.subscribe("dv_resource_id", 10, &DigitalVideoClient_ReceiveFSM::p_dandle_current_ressource_id, this);
	ocu::Slave &slave = ocu::Slave::get_instance(*(jausRouter->getJausAddress()));
	slave.add_supported_service(*this, "urn:jaus:jss:environmentSensing:DigitalVideo", 1, 0);

}

void DigitalVideoClient_ReceiveFSM::control_allowed(std::string service_uri, JausAddress component, unsigned char authority)
{
	if (service_uri.compare("urn:jaus:jss:environmentSensing:DigitalVideo") == 0) {
		p_has_access = true;
		p_remote_addr = component;
		ROS_INFO_NAMED("DigitalVideoClient_ReceiveFSM", "access granted for %d.%d.%d",
				component.getSubsystemID(), component.getNodeID(), component.getComponentID());
	} else {
		ROS_WARN_STREAM("[DigitalVideoClient_ReceiveFSM] unexpected control allowed for " << service_uri << " received, ignored!");
	}
}

void DigitalVideoClient_ReceiveFSM::enable_monitoring_only(std::string service_uri, JausAddress component)
{
	ROS_INFO_NAMED("DigitalVideoClient_ReceiveFSM", "monitor enabled for %d.%d.%d",
			component.getSubsystemID(), component.getNodeID(), component.getComponentID());
}

void DigitalVideoClient_ReceiveFSM::access_deactivated(std::string service_uri, JausAddress component)
{
	p_has_access = false;
	p_remote_addr = JausAddress(0);
	ROS_INFO_NAMED("DigitalVideoClient_ReceiveFSM", "access released for %d.%d.%d",
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

void DigitalVideoClient_ReceiveFSM::p_dandle_current_ressource_id(const std_msgs::UInt16::ConstPtr msg)
{
	if (p_remote_addr.get() != 0) {
		if (msg->data == 65535 and p_current_resource_id != 65535) {
			ControlDigitalVideoSensorStream cmd;
			ROS_INFO_NAMED("DigitalVideoClient_ReceiveFSM", "forward STOP for resource id %d to %d.%d.%d",
					p_current_resource_id, p_remote_addr.getSubsystemID(), p_remote_addr.getNodeID(), p_remote_addr.getComponentID());
			cmd.getBody()->getControlDigitalVideoSensorStreamRec()->setSensorID(p_current_resource_id);
			cmd.getBody()->getControlDigitalVideoSensorStreamRec()->setStreamState(2);
			sendJausMessage(cmd, p_remote_addr);
		} else {
			ControlDigitalVideoSensorStream cmd;
			ROS_INFO_NAMED("DigitalVideoClient_ReceiveFSM", "forward PLAY for resource id %d to %d.%d.%d",
					msg->data, p_remote_addr.getSubsystemID(), p_remote_addr.getNodeID(), p_remote_addr.getComponentID());
			cmd.getBody()->getControlDigitalVideoSensorStreamRec()->setSensorID(msg->data);
			cmd.getBody()->getControlDigitalVideoSensorStreamRec()->setStreamState(0);
			sendJausMessage(cmd, p_remote_addr);
		}
	}
	p_current_resource_id = msg->data;
}




};
