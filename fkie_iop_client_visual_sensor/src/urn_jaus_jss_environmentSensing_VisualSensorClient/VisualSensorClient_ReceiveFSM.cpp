
#include "urn_jaus_jss_environmentSensing_VisualSensorClient/VisualSensorClient_ReceiveFSM.h"
#include <fkie_iop_component/iop_config.hpp>
#include <fkie_iop_ocu_slavelib/Slave.h>



using namespace JTS;

namespace urn_jaus_jss_environmentSensing_VisualSensorClient
{



VisualSensorClient_ReceiveFSM::VisualSensorClient_ReceiveFSM(std::shared_ptr<iop::Component> cmp, urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM, urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM)
: SlaveHandlerInterface(cmp, "VisualSensorClient", 10.0),
  logger(cmp->get_logger().get_child("VisualSensorClient"))
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new VisualSensorClient_ReceiveFSMContext(*this);

	this->pAccessControlClient_ReceiveFSM = pAccessControlClient_ReceiveFSM;
	this->pEventsClient_ReceiveFSM = pEventsClient_ReceiveFSM;
	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->cmp = cmp;
	p_query_state = 0;
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

}


void VisualSensorClient_ReceiveFSM::setupIopConfiguration()
{
	iop::Config cfg(cmp, "VisualSensorClient");
	cfg.declare_param<double>("hz", p_hz, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE,
		"Sets how often the reports are requested. If use_queries is True hz must be greather then 0. In this case each time a Query message is sent to get a report. If use_queries is False an event is created to get Reports. In this case 0 disables the rate and an event of type on_change will be created.",
		"Default: 0.0");
	cfg.param("hz", p_hz, p_hz, false);
	p_pub_visual_sensor_names = cfg.create_publisher<fkie_iop_msgs::msg::VisualSensorNames>("visual_sensor_names", 10);
	p_pub_diagnostic = cfg.create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(std::string("power_states"), 10);
	// initialize the control layer, which handles the access control staff
	this->set_rate(p_hz);
	this->set_supported_service(*this, "urn:jaus:jss:environmentSensing:VisualSensor", 1, 0);
	this->set_event_name("update names for visual sensors");
	this->set_query_before_event(true, 1.0);
}

void VisualSensorClient_ReceiveFSM::register_events(JausAddress remote_addr, double hz)
{
	pEventsClient_ReceiveFSM->create_event(*this, remote_addr, p_query_cfgs, p_hz);
}

void VisualSensorClient_ReceiveFSM::unregister_events(JausAddress remote_addr)
{
	pEventsClient_ReceiveFSM->cancel_event(*this, remote_addr, p_query_cfgs);
	stop_query(remote_addr);
}

void VisualSensorClient_ReceiveFSM::send_query(JausAddress remote_addr)
{
	if (p_query_state == 0) {
		RCLCPP_INFO(logger, "... update names for visual sensors from %s", remote_addr.str().c_str());
		sendJausMessage(p_query_caps, remote_addr);
	} else if (p_query_state == 1) {
		RCLCPP_DEBUG(logger, "send query geometrics and coniguration for visual sensor to %s", remote_addr.str().c_str());
		sendJausMessage(p_query_geo, remote_addr);
		sendJausMessage(p_query_cfgs, remote_addr);
		// force event for request sensor data
		this->set_event_name("visual configuration");
		this->set_query_before_event(false);
	} else {
		sendJausMessage(p_query_cfgs, remote_addr);
	}
}

void VisualSensorClient_ReceiveFSM::stop_query(JausAddress remote_addr)
{
	// send an empty message on release control or monitoring
	auto ros_msg = fkie_iop_msgs::msg::VisualSensorNames();
	p_pub_visual_sensor_names->publish(ros_msg);
	p_query_state = 0;
	lock_type lock(p_mutex);
	p_sensors.clear();
	this->set_event_name("update names for visual sensors");
	this->set_query_before_event(true, 1.0);
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
	RCLCPP_DEBUG(logger, "received confirm configuration from %s for request %d with %d variant inside, currently ignored!",
			sender.str().c_str(), (int)msg.getBody()->getConfirmSensorConfigurationSequence()->getRequestIdRec()->getRequestID(),
			msg.getBody()->getConfirmSensorConfigurationSequence()->getConfirmSensorList()->getNumberOfElements());
}

void VisualSensorClient_ReceiveFSM::handleReportSensorGeometricPropertiesAction(ReportSensorGeometricProperties msg, Receive::Body::ReceiveRec transportData)
{
	lock_type lock(p_mutex);
	JausAddress sender = transportData.getAddress();
	ReportSensorGeometricProperties::Body::GeometricPropertiesList *clist = msg.getBody()->getGeometricPropertiesList();
	RCLCPP_DEBUG(logger, "apply visual geometric properties from %s with %d sensors", sender.str().c_str(), clist->getNumberOfElements());
	for (unsigned int i = 0; i < clist->getNumberOfElements(); i++) {
		ReportSensorGeometricProperties::Body::GeometricPropertiesList::GeometricPropertiesSequence *entry = clist->getElement(i);
		std::map<jUnsignedShortInteger, std::shared_ptr<iop::VisualSensorClient> >::iterator its = p_sensors.find(entry->getSensorIdRec()->getSensorID());
		if (its != p_sensors.end()) {
			its->second->apply_geometric(*entry);
		}
	}
}

void VisualSensorClient_ReceiveFSM::handleReportVisualSensorCapabilitiesAction(ReportVisualSensorCapabilities msg, Receive::Body::ReceiveRec transportData)
{
	lock_type lock(p_mutex);
	p_sensors.clear();
	JausAddress sender = transportData.getAddress();
	auto ros_msg = fkie_iop_msgs::msg::VisualSensorNames();
	ReportVisualSensorCapabilities::Body::VisualSensorCapabilitiesList *caplist = msg.getBody()->getVisualSensorCapabilitiesList();
	RCLCPP_DEBUG(logger, "apply visual capabilities from %s with %d sensors", sender.str().c_str(), caplist->getNumberOfElements());
	std::map<unsigned int, std::map<unsigned short, std::string> >::iterator it = p_sensor_names.find(sender.get());
	if (it != p_sensor_names.end()) {
		it->second.clear();
	}
	for (unsigned int i = 0; i < caplist->getNumberOfElements(); i++) {
		ReportVisualSensorCapabilities::Body::VisualSensorCapabilitiesList::VisualSensorCapabilitiesRec *entry = caplist->getElement(i);
		auto sn = fkie_iop_msgs::msg::VisualSensorName();
		sn.name = entry->getSensorName();
		sn.resource_id = entry->getSensorID();
		ros_msg.names.push_back(sn);
		p_sensor_names[sender.get()][sn.resource_id] = sn.name;
		std::shared_ptr<iop::VisualSensorClient> vs(std::make_shared<iop::VisualSensorClient>(cmp, *entry));
		vs->set_state_callback(&VisualSensorClient_ReceiveFSM::p_state_changed, this);
		p_sensors[entry->getSensorID()] = vs;
	}
	p_pub_visual_sensor_names->publish(ros_msg);
	RCLCPP_DEBUG(logger, "send query geometrics and coniguration for visual sensor to %s", sender.str().c_str());
	sendJausMessage(p_query_geo, sender);
	sendJausMessage(p_query_cfgs, sender);
	p_query_state = 1;
	p_pub_power_states();
}

void VisualSensorClient_ReceiveFSM::handleReportVisualSensorConfigurationAction(ReportVisualSensorConfiguration msg, Receive::Body::ReceiveRec transportData)
{
	lock_type lock(p_mutex);
	JausAddress sender = transportData.getAddress();
	ReportVisualSensorConfiguration::Body::VisualSensorConfigurationList *clist = msg.getBody()->getVisualSensorConfigurationList();
	RCLCPP_DEBUG(logger, "apply visual configuration from %s with %d sensors", sender.str().c_str(), clist->getNumberOfElements());
	for (unsigned int i = 0; i < clist->getNumberOfElements(); i++) {
		ReportVisualSensorConfiguration::Body::VisualSensorConfigurationList::VisualSensorConfigurationRec *entry = clist->getElement(i);
		std::map<jUnsignedShortInteger, std::shared_ptr<iop::VisualSensorClient> >::iterator its = p_sensors.find(entry->getSensorID());
		RCLCPP_DEBUG(logger, "  apply sensor %d: valid state: %d, state: %d", entry->getSensorID(), (int)entry->isSensorStateValid(), (int)entry->getSensorState());
		if (its != p_sensors.end()) {
			its->second->apply_configuration(*entry);
		}
	}
	p_pub_power_states();
}

void VisualSensorClient_ReceiveFSM::p_state_changed(jUnsignedShortInteger id, SetConfigurationRec cfg)
{
	lock_type lock(p_mutex);
	if (has_remote_addr()) {
		RCLCPP_DEBUG(logger, "send new configuration to %s for sensor id %d with request id %d", p_remote_addr.str().c_str(), (int)id , (int)p_request_id);
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
	auto ros_msg = diagnostic_msgs::msg::DiagnosticStatus();
	ros_msg.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
	std::map<jUnsignedShortInteger, std::shared_ptr<iop::VisualSensorClient> >::iterator it;
	for (it = p_sensors.begin(); it != p_sensors.end(); ++it) {
		if (it->second->is_switchable()) {
			auto entry = diagnostic_msgs::msg::KeyValue();
			entry.key = it->second->get_name();
			if (it->second->get_switch_state()) {
				entry.value = "ON";
			} else {
				entry.value = "OFF";
			}
			ros_msg.values.push_back(entry);
		}
	}
	p_pub_diagnostic->publish(ros_msg);
}

}
