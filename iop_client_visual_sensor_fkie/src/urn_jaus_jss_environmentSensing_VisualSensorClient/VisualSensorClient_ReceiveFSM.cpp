
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
	p_query_state = 0;
	p_by_query = false;
	p_hz = 0;
	p_request_id = 0;
	QueryVisualSensorCapabilities::Body::QueryVisualSensorCapabilitiesList::QueryVisualSensorCapabilitiesRec qc_rec;
	qc_rec.setSensorID(0);  // request capabilities for all available sensors
	p_query_caps.getBody()->getQueryVisualSensorCapabilitiesList()->addElement(qc_rec);
	QueryVisualSensorConfiguration::Body::QueryVisualSensorConfigurationList::QueryVisualSensorConfigurationRec qcf_rec;
	qcf_rec.setSensorID(0);
	p_query_cfgs.getBody()->getQueryVisualSensorConfigurationList()->addElement(qcf_rec);
	QuerySensorGeometricProperties::Body::SensorIdList::SensorIdRec sid_rec;
	sid_rec.setSensorID(0);
	p_query_geo.getBody()->getSensorIdList()->addElement(sid_rec);
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
	cfg.param("hz", p_hz, p_hz, false, false);
	p_pub_visual_sensor_names = cfg.advertise<iop_msgs_fkie::VisualSensorNames>("visual_sensor_names", 10, true);
	p_pub_diagnostic = cfg.advertise<diagnostic_msgs::DiagnosticStatus>(std::string("power_states"), 10, true);
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
	ROS_INFO_NAMED("VisualSensorClient", "create QUERY timer to update names for visual sensors from %s", component.str().c_str());
	p_query_timer = p_nh.createTimer(ros::Duration(1), &VisualSensorClient_ReceiveFSM::pQueryCallback, this);
	p_by_query = by_query;
	sendJausMessage(p_query_caps, p_remote_addr);
}

void VisualSensorClient_ReceiveFSM::cancel_events(std::string service_uri, JausAddress component, bool by_query)
{
	lock_type lock(p_mutex);
	p_query_timer.stop();
	if (!by_query) {
		ROS_INFO_NAMED("VisualSensorClient", "cancel EVENT for visual configuration by %s", component.str().c_str());
		pEventsClient_ReceiveFSM->cancel_event(*this, component, p_query_cfgs);
	}
	p_query_state = 0;
	p_sensors.clear();
}

void VisualSensorClient_ReceiveFSM::pQueryCallback(const ros::TimerEvent& event)
{
	if (p_remote_addr.get() != 0) {
		if (p_query_state == 0) {
			ROS_INFO_NAMED("VisualSensorClient", "... update names for visual sensors from %s", p_remote_addr.str().c_str());
			sendJausMessage(p_query_caps, p_remote_addr);
		} else if (p_query_state == 1) {
			ROS_DEBUG_NAMED("VisualSensorClient", "send query geometrics and coniguration for visual sensor to %s", p_remote_addr.str().c_str());
			sendJausMessage(p_query_geo, p_remote_addr);
			sendJausMessage(p_query_cfgs, p_remote_addr);
		} else {
			sendJausMessage(p_query_cfgs, p_remote_addr);
		}
	}
}

void VisualSensorClient_ReceiveFSM::event(JausAddress sender, unsigned short query_msg_id, unsigned int reportlen, const unsigned char* reportdata)
{
	ReportVisualSensorConfiguration report;
	report.decode(reportdata);
	Receive::Body::ReceiveRec transport_data;
	transport_data.setSrcSubsystemID(sender.getSubsystemID());
	transport_data.setSrcNodeID(sender.getNodeID());
	transport_data.setSrcComponentID(sender.getComponentID());
	handleReportVisualSensorConfigurationAction(report, transport_data);
}

std::string VisualSensorClient_ReceiveFSM::get_sensor_name(JausAddress &component, unsigned short id)
{
	lock_type lock(p_mutex);
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
	JausAddress sender = transportData.getAddress();
	ROS_DEBUG_NAMED("VisualSensorClient", "received confirm configuration from %s for request %d with %d variant inside, currently ignored!",
			p_remote_addr.str().c_str(), (int)msg.getBody()->getConfirmSensorConfigurationSequence()->getRequestIdRec()->getRequestID(),
			msg.getBody()->getConfirmSensorConfigurationSequence()->getConfirmSensorList()->getNumberOfElements());
}

void VisualSensorClient_ReceiveFSM::handleReportSensorGeometricPropertiesAction(ReportSensorGeometricProperties msg, Receive::Body::ReceiveRec transportData)
{
	lock_type lock(p_mutex);
	JausAddress sender = transportData.getAddress();
	ReportSensorGeometricProperties::Body::GeometricPropertiesList *clist = msg.getBody()->getGeometricPropertiesList();
	ROS_DEBUG_NAMED("VisualSensorClient", "apply visual geometric properties from %s with %d sensors", p_remote_addr.str().c_str(), clist->getNumberOfElements());
	for (unsigned int i = 0; i < clist->getNumberOfElements(); i++) {
		ReportSensorGeometricProperties::Body::GeometricPropertiesList::GeometricPropertiesSequence *entry = clist->getElement(i);
		std::map<jUnsignedShortInteger, boost::shared_ptr<iop::VisualSensorClient> >::iterator its = p_sensors.find(entry->getSensorIdRec()->getSensorID());
		if (its != p_sensors.end()) {
			its->second->apply_geometric(*entry);
		}
	}
}

void VisualSensorClient_ReceiveFSM::handleReportVisualSensorCapabilitiesAction(ReportVisualSensorCapabilities msg, Receive::Body::ReceiveRec transportData)
{
	lock_type lock(p_mutex);
	p_query_timer.stop();
	p_sensors.clear();
	JausAddress sender = transportData.getAddress();
	iop_msgs_fkie::VisualSensorNames ros_msg;
	ReportVisualSensorCapabilities::Body::VisualSensorCapabilitiesList *caplist = msg.getBody()->getVisualSensorCapabilitiesList();
	ROS_DEBUG_NAMED("VisualSensorClient", "apply visual capabilities from %s with %d sensors", p_remote_addr.str().c_str(), caplist->getNumberOfElements());
	std::map<unsigned int, std::map<unsigned short, std::string> >::iterator it = p_sensor_names.find(sender.get());
	if (it != p_sensor_names.end()) {
		it->second.clear();
	}
	for (unsigned int i = 0; i < caplist->getNumberOfElements(); i++) {
		ReportVisualSensorCapabilities::Body::VisualSensorCapabilitiesList::VisualSensorCapabilitiesRec *entry = caplist->getElement(i);
		iop_msgs_fkie::VisualSensorName sn;
		sn.name = entry->getSensorName();
		sn.resource_id = entry->getSensorID();
		ros_msg.names.push_back(sn);
		p_sensor_names[sender.get()][sn.resource_id] = sn.name;
		boost::shared_ptr<iop::VisualSensorClient> vs(boost::make_shared<iop::VisualSensorClient>(*entry));
		vs->set_state_callback(&VisualSensorClient_ReceiveFSM::p_state_changed, this);
		p_sensors[entry->getSensorID()] = vs;
	}
	p_pub_visual_sensor_names.publish(ros_msg);
	ROS_DEBUG_NAMED("VisualSensorClient", "send query geometrics and coniguration for visual sensor to %s", p_remote_addr.str().c_str());
	sendJausMessage(p_query_geo, p_remote_addr);
	sendJausMessage(p_query_cfgs, p_remote_addr);
	p_query_state = 2;
	// create event or timer for queries
	if (p_remote_addr.get() != 0) {
		if (p_by_query) {
			p_query_timer.stop();
			if (p_hz > 0) {
				ROS_INFO_NAMED("VisualSensorClient", "create QUERY timer to get visual configuration from %s", p_remote_addr.str().c_str());
				p_query_timer = p_nh.createTimer(ros::Duration(1.0 / p_hz), &VisualSensorClient_ReceiveFSM::pQueryCallback, this);
			} else {
				ROS_WARN_NAMED("VisualSensorClient", "invalid hz %.2f for QUERY timer to get visual configuration from %s", p_hz, p_remote_addr.str().c_str());
			}
		} else {
			ROS_INFO_NAMED("VisualSensorClient", "create EVENT to get visual configuration from %s", p_remote_addr.str().c_str());
			pEventsClient_ReceiveFSM->create_event(*this, p_remote_addr, p_query_cfgs, p_hz);
		}
	}
	p_pub_power_states();
}

void VisualSensorClient_ReceiveFSM::handleReportVisualSensorConfigurationAction(ReportVisualSensorConfiguration msg, Receive::Body::ReceiveRec transportData)
{
	lock_type lock(p_mutex);
	JausAddress sender = transportData.getAddress();
	ReportVisualSensorConfiguration::Body::VisualSensorConfigurationList *clist = msg.getBody()->getVisualSensorConfigurationList();
	ROS_DEBUG_NAMED("VisualSensorClient", "apply visual configuration from %s with %d sensors", p_remote_addr.str().c_str(), clist->getNumberOfElements());
	for (unsigned int i = 0; i < clist->getNumberOfElements(); i++) {
		ReportVisualSensorConfiguration::Body::VisualSensorConfigurationList::VisualSensorConfigurationRec *entry = clist->getElement(i);
		std::map<jUnsignedShortInteger, boost::shared_ptr<iop::VisualSensorClient> >::iterator its = p_sensors.find(entry->getSensorID());
		ROS_DEBUG_NAMED("VisualSensorClient", "  apply sensor %d: valid state: %d, state: %d", entry->getSensorID(), (int)entry->isSensorStateValid(), (int)entry->getSensorState());
		if (its != p_sensors.end()) {
			its->second->apply_configuration(*entry);
		}
	}
	p_pub_power_states();
}

void VisualSensorClient_ReceiveFSM::p_state_changed(jUnsignedShortInteger id, SetConfigurationRec cfg)
{
	lock_type lock(p_mutex);
	if (p_remote_addr.get() != 0) {
		ROS_DEBUG_NAMED("VisualSensorClient", "send new configuration to %s for sensor id %d with request id %d", p_remote_addr.str().c_str(), (int)id , (int)p_request_id);
		SetVisualSensorConfiguration response;
		response.getBody()->getVisualSensorConfigurationSequence()->getRequestIdRec()->setRequestID(p_request_id);
		p_request_id++;
		if (p_request_id == 255) p_request_id = 0;
		response.getBody()->getVisualSensorConfigurationSequence()->getVisualSensorConfigurationList()->addElement(cfg);
		sendJausMessage(response, p_remote_addr);
	}
}

void VisualSensorClient_ReceiveFSM::p_pub_power_states()
{
	lock_type lock(p_mutex);
	diagnostic_msgs::DiagnosticStatus ros_msg;
	ros_msg.level = diagnostic_msgs::DiagnosticStatus::OK;
	std::map<jUnsignedShortInteger, boost::shared_ptr<iop::VisualSensorClient> >::iterator it;
	for (it = p_sensors.begin(); it != p_sensors.end(); ++it) {
		if (it->second->is_switchable()) {
			diagnostic_msgs::KeyValue entry;
			entry.key = it->second->get_name();
			if (it->second->get_switch_state()) {
				entry.value = "ON";
			} else {
				entry.value = "OFF";
			}
			ros_msg.values.push_back(entry);
		}
	}
	p_pub_diagnostic.publish(ros_msg);
}

};
