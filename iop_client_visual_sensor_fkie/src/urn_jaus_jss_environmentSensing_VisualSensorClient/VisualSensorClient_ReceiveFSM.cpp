
#include <iop_msgs_fkie/VisualSensorNames.h>

#include <iop_ocu_slavelib_fkie/Slave.h>
#include <iop_component_fkie/iop_config.h>

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
	p_has_access = false;
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
	iop::Config cfg("~VisualSensorClient");
	p_pub_visual_sensor_names = cfg.advertise<iop_msgs_fkie::VisualSensorNames>("visual_sensor_names", 10, true);
	ocu::Slave &slave = ocu::Slave::get_instance(*(jausRouter->getJausAddress()));
	slave.add_supported_service(*this, "urn:jaus:jss:environmentSensing:VisualSensor", 1, 0);
}

void VisualSensorClient_ReceiveFSM::control_allowed(std::string service_uri, JausAddress component, unsigned char authority)
{
	if (service_uri.compare("urn:jaus:jss:environmentSensing:VisualSensor") == 0) {
		p_remote_addr = component;
		p_has_access = true;
	} else {
		ROS_WARN_STREAM_NAMED("VisualSensorClient", "unexpected control allowed for " << service_uri << " received, ignored!");
	}
}

void VisualSensorClient_ReceiveFSM::enable_monitoring_only(std::string service_uri, JausAddress component)
{
	p_remote_addr = component;
}

void VisualSensorClient_ReceiveFSM::access_deactivated(std::string service_uri, JausAddress component)
{
	p_has_access = false;
	p_remote_addr = JausAddress(0);
	// send an empty message on release control or monitoring
	iop_msgs_fkie::VisualSensorNames ros_msg;
	p_pub_visual_sensor_names.publish(ros_msg);
}

void VisualSensorClient_ReceiveFSM::create_events(std::string service_uri, JausAddress component, bool by_query)
{
	ROS_INFO_NAMED("VisualSensorClient", "create QUERY timer to update names for visual sensors from %d.%d.%d",
			component.getSubsystemID(), component.getNodeID(), component.getComponentID());
	p_query_timer = p_nh.createTimer(ros::Duration(3), &VisualSensorClient_ReceiveFSM::pQueryCallback, this);
	sendJausMessage(p_query_caps, p_remote_addr);
}

void VisualSensorClient_ReceiveFSM::cancel_events(std::string service_uri, JausAddress component, bool by_query)
{
	p_query_timer.stop();
}

void VisualSensorClient_ReceiveFSM::pQueryCallback(const ros::TimerEvent& event)
{
	if (p_remote_addr.get() != 0) {
		ROS_INFO_NAMED("VisualSensorClient", "... update names for visual sensors from %d.%d.%d",
				p_remote_addr.getSubsystemID(), p_remote_addr.getNodeID(), p_remote_addr.getComponentID());
		sendJausMessage(p_query_caps, p_remote_addr);
	}
}

std::string VisualSensorClient_ReceiveFSM::get_sensor_name(JausAddress &component, unsigned short id)
{
	std::map<unsigned int, std::map<unsigned short, std::string> >::iterator it = p_sensor_names.find(component.get());
	if (it != p_sensor_names.end()) {
		std::map<unsigned short, std::string>::iterator it2 = it->second.find(id);
		if (it2 != it->second.end()) {
			return it2->second;
		}
	}
	return "";
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
	p_query_timer.stop();
	iop_msgs_fkie::VisualSensorNames ros_msg;
	ReportVisualSensorCapabilities::Body::VisualSensorCapabilitiesList *caplist = msg.getBody()->getVisualSensorCapabilitiesList();
	JausAddress sender = transportData.getAddress();
	std::map<unsigned int, std::map<unsigned short, std::string> >::iterator it = p_sensor_names.find(sender.get());
	if (it != p_sensor_names.end()) {
		it->second.clear();
	}
	for (unsigned int i = 0; i < caplist->getNumberOfElements(); i++) {
		ReportVisualSensorCapabilities::Body::VisualSensorCapabilitiesList::VisualSensorCapabilitiesRec *entry = caplist->getElement(i);
		iop_msgs_fkie::VisualSensorName sn;
		sn.name = entry->getSensorName();
		sn.resource_id = entry->getSensorID();
		p_sensor_names[sender.get()][sn.resource_id] = sn.name;
		ros_msg.names.push_back(sn);
	}
	p_pub_visual_sensor_names.publish(ros_msg);
}

void VisualSensorClient_ReceiveFSM::handleReportVisualSensorConfigurationAction(ReportVisualSensorConfiguration msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
}





};
