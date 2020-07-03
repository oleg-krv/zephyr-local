/*
 * Copyright (c) 2020 Lemonbeat GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <net/ppp.h>

void ppp_mgmt_raise_carrier_on_event(struct net_if *iface)
{
	net_mgmt_event_notify(NET_EVENT_PPP_CARRIER_ON, iface);
}

void ppp_mgmt_raise_carrier_off_event(struct net_if *iface)
{
	net_mgmt_event_notify(NET_EVENT_PPP_CARRIER_OFF, iface);
}

void ppp_mgmt_raise_link_dead_event(struct net_if *iface)
{
    net_mgmt_event_notify(NET_EVENT_PPP_LINK_DEAD, iface);
}

void ppp_mgmt_raise_link_down_event(struct net_if *iface)
{
    net_mgmt_event_notify(NET_EVENT_PPP_LINK_DOWN, iface);
}

#ifdef CONFIG_MODEM_GSM_SIMCOM_EXT

void ppp_mgmt_raise_modem_operator_connect_event(struct net_if *iface)
{
	net_mgmt_event_notify(NET_EVENT_MODEM_OPERATOR_CONNECT, iface);
}

void ppp_mgmt_raise_modem_operator_disconnect_event(struct net_if *iface)
{
	net_mgmt_event_notify(NET_EVENT_MODEM_OPERATOR_DISCONNECT, iface);
}

void ppp_mgmt_raise_modem_sim_error_event(struct net_if *iface)
{
    net_mgmt_event_notify(NET_EVENT_MODEM_SIM_ERROR, iface);
}

void ppp_mgmt_raise_modem_reset_event(struct net_if *iface)
{
    net_mgmt_event_notify(NET_EVENT_MODEM_RESET, iface);
}

void ppp_mgmt_raise_modem_power_on_event(struct net_if *iface)
{
    net_mgmt_event_notify(NET_EVENT_MODEM_POWER_ON, iface);
}

void ppp_mgmt_raise_modem_power_off_event(struct net_if *iface)
{
    net_mgmt_event_notify(NET_EVENT_MODEM_POWER_OFF, iface);
}
#endif
