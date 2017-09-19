
#include <iop_msgs_fkie/VisualSensorNames.h>

#include <iop_ocu_slavelib_fkie/Slave.h>

#include "urn_jaus_jss_environmentSensing_VisualSensorClient/VisualSensorClient_ReceiveFSM.h"




using namespace JTS;
using namespace iop;

namespace urn_jaus_jss_environmentSensing_VisualSensorClient
{



VisualSensorClient_ReceiveFSM::VisualSensorClient_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM, urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM)
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new VisualSensorClient_ReceiveFSMContext(*this);

	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->pEventsClient_ReceiveFSM = pEventsClient_ReceiveFSM;
	this->pAccessControlClient_ReceiveFSM = pAccessControlClient_ReceiveFSM;
	p_pnh = ros::NodeHandle("~");
	QueryVisualSensorCapabilities::Body::QueryVisualSensorCapabilitiesList::QueryVisualSensorCapabilitiesRec qc_rec;
	qc_rec.setSensorID(65535);  // request capabilities for all available sensors
	p_query_caps.getBody()->getQueryVisualSensorCapabilitiesList()->addElement(qc_rec);
}



VisualSensorClient_ReceiveFSM::~VisualSensorClient_ReceiveFSM()
{
	delete context;
}

void VisualSensorClient_ReceiveFSM::setupNotifications()
{
	pAccessControlClient_ReceiveFSM->registerNotification("Receiving_Ready", ieHandler, "InternalStateChange_To_VisualSensorClient_ReceiveFSM_Receiving_Ready", "AccessControlClient_ReceiveFSM");
	pAccessControlClient_ReceiveFSM->registerNotification("Receiving", ieHandler, "InternalStateChange_To_VisualSensorClient_ReceiveFSM_Receiving_Ready", "AccessControlClient_ReceiveFSM");
	registerNotification("Receiving_Ready", pAccessControlClient_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControlClient_ReceiveFSM_Receiving_Ready", "VisualSensorClient_ReceiveFSM");
	registerNotification("Receiving", pAccessControlClient_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControlClient_ReceiveFSM_Receiving", "VisualSensorClient_ReceiveFSM");

	p_pub_visual_sensor_names = p_nh.advertise<iop_msgs_fkie::VisualSensorNames>("visual_sensor_names", 10, true);
	ocu::Slave &slave = ocu::Slave::get_instance(*(jausRouter->getJausAddress()));
	slave.add_supported_service(*this, "urn:jaus:jss:environmentSensing:VisualSensor", 1, 0);
}

void VisualSensorClient_ReceiveFSM::control_allowed(std::string service_uri, JausAddress component, unsigned char authority)
{
	if (service_uri.compare("urn:jaus:jss:environmentSensing:VisualSensor") == 0) {
		p_control_addr = component;
		ROS_INFO_NAMED("VisualSensorClient_ReceiveFSM", "access granted for %d.%d.%d",
				component.getSubsystemID(), component.getNodeID(), component.getComponentID());
		sendJausMessage(p_query_caps, p_control_addr);
	} else {
		ROS_WARN_STREAM("[VisualSensorClient_ReceiveFSM] unexpected control allowed for " << service_uri << " received, ignored!");
	}
}

void VisualSensorClient_ReceiveFSM::enable_monitoring_only(std::string service_uri, JausAddress component)
{
	ROS_INFO_NAMED("VisualSensorClient_ReceiveFSM", "monitor enabled for %d.%d.%d",
			component.getSubsystemID(), component.getNodeID(), component.getComponentID());
	sendJausMessage(p_query_caps, component);
}

void VisualSensorClient_ReceiveFSM::access_deactivated(std::string service_uri, JausAddress component)
{
	p_control_addr = JausAddress(0);
	ROS_INFO_NAMED("VisualSensorClient_ReceiveFSM", "access released for %d.%d.%d",
			component.getSubsystemID(), component.getNodeID(), component.getComponentID());
	// send an empty message on release control or monitoring
	iop_msgs_fkie::VisualSensorNames ros_msg;
	p_pub_visual_sensor_names.publish(ros_msg);
}

void VisualSensorClient_ReceiveFSM::handleConfirmSensorConfigurationAction(ConfirmSensorConfiguration msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
}

void VisualSensorClient_ReceiveFSM::handleReportSensorGeometricPropertiesAction(ReportSensorGeometricProperties msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
}

void VisualSensorClient_ReceiveFSM::handleReportVisualSensorCapabilitiesAction(ReportVisualSensorCapabilities msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
	iop_msgs_fkie::VisualSensorNames ros_msg;
	ReportVisualSensorCapabilities::Body::VisualSensorCapabilitiesList *caplist = msg.getBody()->getVisualSensorCapabilitiesList();
	for (unsigned int i = 0; i < caplist->getNumberOfElements(); i++) {
		ReportVisualSensorCapabilities::Body::VisualSensorCapabilitiesList::VisualSensorCapabilitiesRec *entry = caplist->getElement(i);
		iop_msgs_fkie::VisualSensorName sn;
		sn.name = entry->getSensorName();
		sn.resource_id = entry->getSensorID();
		ros_msg.names.push_back(sn);
	}
	p_pub_visual_sensor_names.publish(ros_msg);
}

void VisualSensorClient_ReceiveFSM::handleReportVisualSensorConfigurationAction(ReportVisualSensorConfiguration msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
}





};
