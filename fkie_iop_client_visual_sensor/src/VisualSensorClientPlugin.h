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


#ifndef VISUALSENSORCLIENTPLUGIN_H
#define VISUALSENSORCLIENTPLUGIN_H

#include "urn_jaus_jss_environmentSensing_VisualSensorClient/VisualSensorClientService.h"
#include "urn_jaus_jss_core_AccessControlClient/AccessControlClientService.h"
#include "urn_jaus_jss_core_EventsClient/EventsClientService.h"
#include "urn_jaus_jss_core_Transport/TransportService.h"

#include <fkie_iop_component/iop_plugin_interface.h>

namespace iop
{

class DllExport VisualSensorClientPlugin : public PluginInterface
{
public:
	VisualSensorClientPlugin();

	JTS::Service* get_service();
	void create_service(JTS::JausRouter* jaus_router);

protected:
	urn_jaus_jss_environmentSensing_VisualSensorClient::VisualSensorClientService *p_my_service;
	urn_jaus_jss_core_AccessControlClient::AccessControlClientService *p_base_service;
	urn_jaus_jss_core_EventsClient::EventsClientService *p_events_service;
	urn_jaus_jss_core_Transport::TransportService *p_transport_service;

};

};

#endif
