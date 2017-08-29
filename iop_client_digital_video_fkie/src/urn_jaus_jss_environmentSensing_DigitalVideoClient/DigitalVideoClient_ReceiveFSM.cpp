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


#include <iop_ocu_slavelib_fkie/Slave.h>

#include "urn_jaus_jss_environmentSensing_DigitalVideoClient/DigitalVideoClient_ReceiveFSM.h"




using namespace JTS;
using namespace iop;

namespace urn_jaus_jss_environmentSensing_DigitalVideoClient
{



DigitalVideoClient_ReceiveFSM::DigitalVideoClient_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM, urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM)
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
	p_pnh = ros::NodeHandle("~");
	p_current_resource_id = 65535;
}



DigitalVideoClient_ReceiveFSM::~DigitalVideoClient_ReceiveFSM()
{
	delete context;
}

void DigitalVideoClient_ReceiveFSM::setupNotifications()
{
	pAccessControlClient_ReceiveFSM->registerNotification("Receiving_Ready", ieHandler, "InternalStateChange_To_DigitalVideoClient_ReceiveFSM_Receiving_Ready", "AccessControlClient_ReceiveFSM");
	pAccessControlClient_ReceiveFSM->registerNotification("Receiving", ieHandler, "InternalStateChange_To_DigitalVideoClient_ReceiveFSM_Receiving_Ready", "AccessControlClient_ReceiveFSM");
	registerNotification("Receiving_Ready", pAccessControlClient_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControlClient_ReceiveFSM_Receiving_Ready", "DigitalVideoClient_ReceiveFSM");
	registerNotification("Receiving", pAccessControlClient_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControlClient_ReceiveFSM_Receiving", "DigitalVideoClient_ReceiveFSM");
	p_sub_cur_dv_id = p_nh.subscribe("dv_resource_id", 10, &DigitalVideoClient_ReceiveFSM::p_dandle_current_ressource_id, this);
	ocu::Slave &slave = ocu::Slave::get_instance(*(jausRouter->getJausAddress()));
	slave.add_supported_service(*this, "urn:jaus:jss:environmentSensing:DigitalVideo", 1, 0);

}

void DigitalVideoClient_ReceiveFSM::control_allowed(std::string service_uri, JausAddress component, unsigned char authority)
{
	if (service_uri.compare("urn:jaus:jss:environmentSensing:DigitalVideo") == 0) {
		p_control_addr = component;
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
	p_control_addr = JausAddress(0);
	ROS_INFO_NAMED("DigitalVideoClient_ReceiveFSM", "access released for %d.%d.%d",
			component.getSubsystemID(), component.getNodeID(), component.getComponentID());
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
	if (p_control_addr.get() != 0) {
		if (msg->data == 65535 and p_current_resource_id != 65535) {
			ControlDigitalVideoSensorStream cmd;
			ROS_INFO_NAMED("DigitalVideoClient_ReceiveFSM", "forward STOP for resource id %d to %d.%d.%d",
					p_current_resource_id, p_control_addr.getSubsystemID(), p_control_addr.getNodeID(), p_control_addr.getComponentID());
			cmd.getBody()->getControlDigitalVideoSensorStreamRec()->setSensorID(p_current_resource_id);
			cmd.getBody()->getControlDigitalVideoSensorStreamRec()->setStreamState(2);
			sendJausMessage(cmd, p_control_addr);
		} else {
			ControlDigitalVideoSensorStream cmd;
			ROS_INFO_NAMED("DigitalVideoClient_ReceiveFSM", "forward PLAY for resource id %d to %d.%d.%d",
					msg->data, p_control_addr.getSubsystemID(), p_control_addr.getNodeID(), p_control_addr.getComponentID());
			cmd.getBody()->getControlDigitalVideoSensorStreamRec()->setSensorID(msg->data);
			cmd.getBody()->getControlDigitalVideoSensorStreamRec()->setStreamState(0);
			sendJausMessage(cmd, p_control_addr);
		}
	}
	p_current_resource_id = msg->data;
}




};
