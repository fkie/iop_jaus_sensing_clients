

#ifndef VISUALSENSORCLIENT_RECEIVEFSM_H
#define VISUALSENSORCLIENT_RECEIVEFSM_H

#include "JausUtils.h"
#include "InternalEvents/InternalEventHandler.h"
#include "Transport/JausTransport.h"
#include "JTSStateMachine.h"
#include "urn_jaus_jss_environmentSensing_VisualSensorClient/Messages/MessageSet.h"
#include "urn_jaus_jss_environmentSensing_VisualSensorClient/InternalEvents/InternalEventsSet.h"

#include "InternalEvents/Receive.h"
#include "InternalEvents/Send.h"

#include "urn_jaus_jss_core_Transport/Transport_ReceiveFSM.h"
#include "urn_jaus_jss_core_EventsClient/EventsClient_ReceiveFSM.h"
#include "urn_jaus_jss_core_AccessControlClient/AccessControlClient_ReceiveFSM.h"


#include "VisualSensorClient_ReceiveFSM_sm.h"
#include <rclcpp/rclcpp.hpp>
#include <fkie_iop_component/iop_component.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <fkie_iop_ocu_slavelib/SlaveHandlerInterface.h>
#include <fkie_iop_events/EventHandlerInterface.h>
#include <fkie_iop_client_visual_sensor/VisualSensorClient.h>
#include <fkie_iop_msgs/msg/visual_sensor_names.hpp>


namespace urn_jaus_jss_environmentSensing_VisualSensorClient
{

class DllExport VisualSensorClient_ReceiveFSM : public JTS::StateMachine, public iop::ocu::SlaveHandlerInterface, public iop::EventHandlerInterface
{
public:
	VisualSensorClient_ReceiveFSM(std::shared_ptr<iop::Component> cmp, urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM, urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM);
	virtual ~VisualSensorClient_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();
	virtual void setupIopConfiguration();

	/// Action Methods
	virtual void handleConfirmSensorConfigurationAction(ConfirmSensorConfiguration msg, Receive::Body::ReceiveRec transportData);
	virtual void handleReportSensorGeometricPropertiesAction(ReportSensorGeometricProperties msg, Receive::Body::ReceiveRec transportData);
	virtual void handleReportVisualSensorCapabilitiesAction(ReportVisualSensorCapabilities msg, Receive::Body::ReceiveRec transportData);
	virtual void handleReportVisualSensorConfigurationAction(ReportVisualSensorConfiguration msg, Receive::Body::ReceiveRec transportData);


	/// SlaveHandlerInterface Methods
	void control_allowed(std::string service_uri, JausAddress component, unsigned char authority);
	void enable_monitoring_only(std::string service_uri, JausAddress component);
	void access_deactivated(std::string service_uri, JausAddress component);
	void create_events(std::string service_uri, JausAddress component, bool by_query=false);
	void cancel_events(std::string service_uri, JausAddress component, bool by_query=false);
	/// Guard Methods

	/// EventHandlerInterface Methods
	void event(JausAddress reporter, unsigned short query_msg_id, unsigned int reportlen, const unsigned char* reportdata);
	std::string get_sensor_name(JausAddress &component, unsigned short id);


	VisualSensorClient_ReceiveFSMContext *context;

protected:

	/// References to parent FSMs
	urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM;
	urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM;
	urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;

	std::shared_ptr<iop::Component> cmp;
	rclcpp::Logger logger;

	typedef std::recursive_mutex mutex_type;
	typedef std::unique_lock<mutex_type> lock_type;
	mutable mutex_type p_mutex;

	JausAddress p_remote_addr;
	bool p_has_access;
	int p_query_state;
	bool p_by_query;
	double p_hz;
	uint8_t p_request_id;
	iop::Timer p_query_timer;
	rclcpp::Publisher<fkie_iop_msgs::msg::VisualSensorNames>::SharedPtr p_pub_visual_sensor_names;
	rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr p_pub_diagnostic;
	QueryVisualSensorCapabilities p_query_caps;
	QuerySensorGeometricProperties p_query_geo;
	QueryVisualSensorConfiguration p_query_cfgs;
	std::map<unsigned int, std::map<uint16_t, std::string> > p_sensor_names;  // JausAddress, sensor ID, name
	std::map<uint16_t, std::shared_ptr<iop::VisualSensorClient> > p_sensors;

	void pQueryCallback();
	void p_state_changed(uint16_t id, SetConfigurationRec cfg);
        void p_pub_power_states();

};

}

#endif // VISUALSENSORCLIENT_RECEIVEFSM_H
