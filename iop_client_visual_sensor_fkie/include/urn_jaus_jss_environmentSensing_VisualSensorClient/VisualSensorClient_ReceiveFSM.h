

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

#include <ros/ros.h>
#include <std_msgs/UInt16.h>
#include <iop_ocu_slavelib_fkie/SlaveHandlerInterface.h>


namespace urn_jaus_jss_environmentSensing_VisualSensorClient
{

class DllExport VisualSensorClient_ReceiveFSM : public JTS::StateMachine, public iop::ocu::SlaveHandlerInterface
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
	/// Guard Methods



	VisualSensorClient_ReceiveFSMContext *context;

protected:

    /// References to parent FSMs
	urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;
	urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM;
	urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM;

	JausAddress p_control_addr;
	ros::NodeHandle p_nh;
	ros::NodeHandle p_pnh;
	ros::Publisher p_pub_visual_sensor_names;
	QueryVisualSensorCapabilities p_query_caps;

};

};

#endif // VISUALSENSORCLIENT_RECEIVEFSM_H
