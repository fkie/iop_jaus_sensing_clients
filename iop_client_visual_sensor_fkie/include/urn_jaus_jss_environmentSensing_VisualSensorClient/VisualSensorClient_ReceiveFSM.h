

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

#include <boost/thread/recursive_mutex.hpp>
#include <ros/ros.h>
#include <std_msgs/UInt16.h>
#include <iop_ocu_slavelib_fkie/SlaveHandlerInterface.h>
#include <iop_events_fkie/EventHandlerInterface.h>
#include <iop_client_visual_sensor_fkie/VisualSensorClient.h>

namespace urn_jaus_jss_environmentSensing_VisualSensorClient
{

class DllExport VisualSensorClient_ReceiveFSM : public JTS::StateMachine, public iop::ocu::SlaveHandlerInterface, public iop::EventHandlerInterface
{
public:
	VisualSensorClient_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM, urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM);
	virtual ~VisualSensorClient_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();

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
	urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;
	urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM;
	urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM;

	typedef boost::recursive_mutex mutex_type;
	typedef boost::unique_lock<mutex_type> lock_type;
	mutable mutex_type p_mutex;

	JausAddress p_remote_addr;
	ros::NodeHandle p_nh;
	bool p_has_access;
	int p_query_state;
	bool p_by_query;
	double p_hz;
	unsigned char p_request_id;
	ros::Timer p_query_timer;
	ros::Publisher p_pub_visual_sensor_names;
	QueryVisualSensorCapabilities p_query_caps;
	QuerySensorGeometricProperties p_query_geo;
	QueryVisualSensorConfiguration p_query_cfgs;
	std::map<unsigned int, std::map<unsigned short, std::string> > p_sensor_names;  // JausAddress, sensor ID, name
	std::map<jUnsignedShortInteger, boost::shared_ptr<iop::VisualSensorClient> > p_sensors;

	void pQueryCallback(const ros::TimerEvent& event);
	void p_state_changed(jUnsignedShortInteger id, SetConfigurationRec cfg);

};

};

#endif // VISUALSENSORCLIENT_RECEIVEFSM_H
