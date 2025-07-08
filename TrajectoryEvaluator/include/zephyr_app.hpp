// Copyright (c) 2022-2023, Arm Limited.
// SPDX-License-Identifier: Apache-2.0

#ifndef ZEPHYR_APP_HPP_
#define ZEPHYR_APP_HPP_

#include <dds/ddsi/ddsi_config.h>

#define ACTUATION_STACK_SIZE 16 * 1024

// Domain ID used for Safety Island communication.
#define DDS_DOMAIN_ACTUATION 2

#define ACTUATION_SERVICE_PORT 49152
#define PACKET_ANALYZER_FIN "packet_analyzer_fin"
#define PACKET_ANALYZER_FIN_ACK "packet_analyzer_fin_ack"

#if defined(CONFIG_NET_CONFIG_PEER_IPV4_ADDR)
static struct ddsi_config_peer_listelem cfg_peer
{
  nullptr,
  const_cast<char *>(CONFIG_NET_CONFIG_PEER_IPV4_ADDR)
};
#endif

#if defined(CONFIG_DDS_NETWORK_INTERFACE)
static struct ddsi_config_network_interface_listelem cfg_iface
{
  nullptr,
  {
    0,
    const_cast<char *>(CONFIG_DDS_NETWORK_INTERFACE),
    nullptr,
    0,
    1,
    DDSI_BOOLDEF_DEFAULT,
    {1, 0}
  }
};
#endif

/// @brief Initialize a given DDS configuration structure.
/// @param[out] cfg Configuration structure that will be filled.
inline static void init_config(struct ddsi_config & cfg)
{
  ddsi_config_init_default(&cfg);

  // Buffers
  cfg.rbuf_size = 16 * 1024;
  cfg.rmsg_chunk_size = 2 * 1204;
  cfg.max_msg_size = 1456;

  // Discovery
  cfg.participantIndex = DDSI_PARTICIPANT_INDEX_AUTO;
  cfg.maxAutoParticipantIndex = 60;
  cfg.allowMulticast = DDSI_AMC_SPDP;

  // Trace
  cfg.tracefp = NULL;
  cfg.tracemask = DDS_LC_FATAL | DDS_LC_ERROR;
  cfg.tracefile = const_cast<char *>("stderr");
  cfg.tracefp = NULL;

#if defined(CONFIG_NET_CONFIG_PEER_IPV4_ADDR)
  if (sizeof(CONFIG_NET_CONFIG_PEER_IPV4_ADDR) > 1) {
    cfg.peers = &cfg_peer;
  }
#endif

#if defined(CONFIG_DDS_NETWORK_INTERFACE)
  if (sizeof(CONFIG_DDS_NETWORK_INTERFACE) > 1) {
    cfg.network_interfaces = &cfg_iface;
  }
#endif
}

#endif  // ZEPHYR_APP_HPP_
