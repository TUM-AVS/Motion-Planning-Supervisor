#include "dds/dds.h"
#include <stdio.h>
#include <iostream>
#include <memory>
#include <vector>
#include <stdlib.h>
#include <zephyr/kernel.h>
#include <pthread.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/ethernet.h>
#include <Eigen/Core>
#include "TrajectorySample.hpp"
#include "CartesianSample.hpp"
#include "CurvilinearSample.hpp"
#include "TrajectorySampleData.h"
#include "CartesianSampleData.h"
#include "CurvilinearSampleData.h"
#include "ResultData.h"
#include "TrajectoryEvaluator.hpp"
#include <dds/ddsi/ddsi_config.h>
#include "zephyr_app.hpp"


#define ACTUATION_STACK_SIZE 16 * 1024
#define DDS_DOMAIN_ACTUATION 2
static K_THREAD_STACK_DEFINE(main_stack_area, ACTUATION_STACK_SIZE);

#if defined(CONFIG_NET_DHCPV4)
static K_SEM_DEFINE(got_address, 0, 1);

static struct net_mgmt_event_callback mgmt_cb;
#endif  // CONFIG_NET_DHCPV4

const char* const TRAJECTORY_SAMPLE_MSG = "trajectory_sample_msg";
const char* const RESULT_DATA_MSG = "result_data_msg";

using TrajectorySampleData = custom_trajectory_msgs_msg_TrajectorySampleData;
dds_entity_t result_writer;

static bool get_msg(dds_entity_t rd, void * sample)
{
  dds_sample_info_t info;
  dds_return_t rc = dds_take(rd, &sample, &info, 1, 1);
  if (rc < 0) {
    dds_log(DDS_LC_WARNING, __FILE__, __LINE__, DDS_FUNCTION, "can't take msg\n");
  }
  if (rc > 0 && info.valid_data) {
    return true;
  }
  return false;
}

template<typename T>
static void on_msg_dds(dds_entity_t rd, void * arg)
{
  if (!arg) {
    return;
  }
  auto process = *reinterpret_cast<std::function<void(const T &)> *>(arg);

  // Rely on CycloneDDS for the memory management of the message's sequence buffers.
  // A static message ensures the buffers allocated dynamically on the previous calls are not lost.
  static T msg;
  if (get_msg(rd, reinterpret_cast<void *>(&msg))) {
    process(msg);
  }
}


/*
Creates a DDS reader.

m_participant: The DDS participant entity.
desc: Pointer to the DDS topic descriptor for the message type.
name: Name of the topic to read from.
qos: Quality of Service settings for the reader.
callback: Callback function that is called upon data receival.
arg: User-defined data pointer, passed by the listener to the callback function.
*/
void create_reader(
  dds_entity_t m_participant,
  const dds_topic_descriptor_t * desc,
  const char * name, const dds_qos_t * qos, 
  dds_on_data_available_fn callback, void * arg)
{
  dds_entity_t topic = dds_create_topic(m_participant, desc, name, NULL, NULL);
  if (topic < 0) {
    printf("dds_create_topic (%s): %s\n", name, dds_strretcode(-topic));
    std::exit(EXIT_FAILURE);
  }

  dds_listener_t * listener = dds_create_listener(arg);
  dds_lset_data_available(listener, callback);

  dds_entity_t reader = dds_create_reader(m_participant, topic, qos, listener);
  if (reader < 0) {
    printf("dds_create_reader (%s): %s\n", name, dds_strretcode(-reader));
    std::exit(EXIT_FAILURE);
  }

  std::cout << "waiting for writer to be discovered" << std::endl;

  uint32_t status = 0;

  dds_return_t rc = dds_set_status_mask(reader, DDS_SUBSCRIPTION_MATCHED_STATUS);
  if (rc != DDS_RETCODE_OK)
    DDS_FATAL("dds_set_status_mask: %s\n", dds_strretcode(-rc));

  while(!(status & DDS_SUBSCRIPTION_MATCHED_STATUS))
  {
    rc = dds_get_status_changes (reader, &status);
    if (rc != DDS_RETCODE_OK)
      DDS_FATAL("dds_get_status_changes: %s\n", dds_strretcode(-rc));

    /* Polling sleep. */
    dds_sleepfor (DDS_MSECS (20));
  }

  dds_delete_listener(listener);
}

/*
@brief Creates a DDS writer.

@param m_participant The DDS participant entity.
@param desc Pointer to the DDS topic descriptor for the message type.
@param name Name of the topic to read from.
@param qos Quality of Service settings for the reader.
*/
dds_entity_t create_writer(
  dds_entity_t m_participant,
  const dds_topic_descriptor_t * desc,
  const char * name, const dds_qos_t * qos)
{
  dds_entity_t topic = dds_create_topic(m_participant, desc, name, NULL, NULL);
  if (topic < 0) {
    printf("dds_create_topic (%s): %s\n", name, dds_strretcode(-topic));
    std::exit(EXIT_FAILURE);
  }

  dds_entity_t writer = dds_create_writer(m_participant, topic, qos, NULL);
  if (writer < 0) {
    printf("dds_create_writer (%s): %s\n", name, dds_strretcode(-writer));
    std::exit(EXIT_FAILURE);
  }

  return writer;
}

/*
@brief Creates an Eigen::Map to view a DDS buffer as an Eigen::VectorXd.

@param seq Data buffer
*/
inline Eigen::Map< Eigen::VectorXd> ddsSequenceToVector(const dds_sequence_double& seq)
{
    return Eigen::Map< Eigen::VectorXd>(
        seq._buffer, 
        static_cast<Eigen::Index>(seq._length)
    );
}


/*
@brief Converts a TrajectorySampleData message to a TrajectorySample and calcualtes cost and feasibility on it.

@param msg TrajectorySampleData message
*/
void on_msg(const TrajectorySampleData& msg) {
  size_t m_size = msg.size;
  size_t m_actualSize = msg.m_actual_size;
  double m_dT = msg.m_d_t;

  auto m_curvilinearSample_s = ddsSequenceToVector(msg.m_curvilinear_sample.s);
  auto m_curvilinearSample_d = ddsSequenceToVector(msg.m_curvilinear_sample.d);
  auto m_curvilinearSample_theta = ddsSequenceToVector(msg.m_curvilinear_sample.theta);
  auto m_curvilinearSample_dd = ddsSequenceToVector(msg.m_curvilinear_sample.dd);
  auto m_curvilinearSample_ddd = ddsSequenceToVector(msg.m_curvilinear_sample.ddd);
  auto m_curvilinearSample_ss = ddsSequenceToVector(msg.m_curvilinear_sample.ss);
  auto m_curvilinearSample_sss = ddsSequenceToVector(msg.m_curvilinear_sample.sss);

  auto m_cartesianSample_x = ddsSequenceToVector(msg.m_cartesian_sample.x);
  auto m_cartesianSample_y = ddsSequenceToVector(msg.m_cartesian_sample.y);
  auto m_cartesianSample_theta = ddsSequenceToVector(msg.m_cartesian_sample.theta);
  auto m_cartesianSample_velocity = ddsSequenceToVector(msg.m_cartesian_sample.velocity);
  auto m_cartesianSample_acceleration = ddsSequenceToVector(msg.m_cartesian_sample.acceleration);
  auto m_cartesianSample_kappa = ddsSequenceToVector(msg.m_cartesian_sample.kappa);
  auto m_cartesianSample_kappaDot = ddsSequenceToVector(msg.m_cartesian_sample.kappa_dot);

  CartesianSample cartesianSample(
      m_cartesianSample_x,
      m_cartesianSample_y,
      m_cartesianSample_theta,
      m_cartesianSample_velocity,
      m_cartesianSample_acceleration,
      m_cartesianSample_kappa,
      m_cartesianSample_kappaDot
  );

  CurviLinearSample curvilinearSample(
      m_curvilinearSample_s,
      m_curvilinearSample_d,
      m_curvilinearSample_theta,
      m_curvilinearSample_dd,
      m_curvilinearSample_ddd,
      m_curvilinearSample_ss,
      m_curvilinearSample_sss
  );

  TrajectorySample trajectory_sample(
      cartesianSample,
      curvilinearSample,
      m_size,
      m_actualSize,
      m_dT
  );

  TrajectoryEvaluator trajectory_evaluator;

#if defined(ENABLE_TIMING)
  uint32_t start = k_cycle_get_32();
#endif
  trajectory_evaluator.evaluateTrajectory(trajectory_sample);
#if defined(ENABLE_TIMING)
  uint32_t end = k_cycle_get_32();
#endif

  custom_trajectory_msgs_msg_ResultData result_msg;
  result_msg.m_cost = trajectory_sample.m_cost;
  result_msg.m_feasible = trajectory_sample.m_feasible;
  
#if defined(ENABLE_TIMING)
  uint64_t ns = k_cyc_to_ns_floor64(end - start);
  std::cout << "Evaluation of trajectory took " << ns << " ns." << std::endl;
#endif

  dds_write(result_writer, &result_msg);
}

/*
@brief Main function, configures network settings and DDS participant.
*/
static void * main_thread(void * arg)
{
    std::cout << "entered main_thread" << std::endl;
    (void)arg;
    dds_entity_t participant;
    dds_qos_t *qos;

    struct ddsi_config dds_cfg;
    init_config(dds_cfg);

    dds_entity_t domain = dds_create_domain_with_rawconfig(DDS_DOMAIN_ACTUATION, &dds_cfg);

    if (domain < 0 && domain != DDS_RETCODE_PRECONDITION_NOT_MET) {
      printf("dds_create_domain_with_rawconfig: %s\n", dds_strretcode(-domain));
      std::exit(EXIT_FAILURE);
    }

    participant = dds_create_participant (DDS_DOMAIN_ACTUATION, NULL, NULL);
    if (participant < 0)
      DDS_FATAL("dds_create_participant: %s\n", dds_strretcode(-participant));

    qos = dds_create_qos ();
    dds_qset_reliability (qos, DDS_RELIABILITY_RELIABLE, DDS_MSECS(30));  

    std::function<void(const TrajectorySampleData&)> trajectory_callback = [&](const TrajectorySampleData & msg) {on_msg(msg);};

    std::cout << "creating reader" << std::endl;
    create_reader(
      participant,
      &custom_trajectory_msgs_msg_TrajectorySampleData_desc,
      TRAJECTORY_SAMPLE_MSG,
      qos,
      on_msg_dds<TrajectorySampleData>,
      reinterpret_cast<void*>(&trajectory_callback)
    );
    
    std::cout << "creating writer" << std::endl;
    result_writer = create_writer(
      participant,
      &custom_trajectory_msgs_msg_ResultData_desc,
      RESULT_DATA_MSG,
      qos
    );
    dds_delete_qos(qos);
    
    
    return NULL;
}

static void net_if_cb(struct net_if * iface, void * user_data)
{
  auto ifs = reinterpret_cast<std::vector<struct net_if *> *>(user_data);
  ifs->push_back(iface);
}

static int setup_iface(
  struct net_if * iface, const char * addr, const char * gw, const char * netmask, uint16_t tag)
{
  struct in_addr inaddr;

  if (net_addr_pton(AF_INET, addr, &inaddr)) {
    std::printf("Invalid address: %s", addr);
    return 1;
  }
  if (!net_if_ipv4_addr_add(iface, &inaddr, NET_ADDR_MANUAL, 0)) {
    std::printf("Cannot add %s to interface %p", addr, iface);
    return 1;
  }

  if (net_addr_pton(AF_INET, gw, &inaddr)) {
    std::printf("Invalid address: %s", gw);
    return 1;
  }
  net_if_ipv4_set_gw(iface, &inaddr);

  if (net_addr_pton(AF_INET, netmask, &inaddr)) {
    std::printf("Invalid address: %s", netmask);
    return 1;
  }
  net_if_ipv4_set_netmask(iface, &inaddr);

#if defined(CONFIG_NET_VLAN)
  if (tag > 0) {
    int ret = net_eth_vlan_enable(iface, tag);
    if (ret < 0) {
      std::printf("Cannot set VLAN tag %d to interface %p", tag, iface);
      return 1;
    }
  }
#endif

  return 0;
}

#if defined(CONFIG_NET_DHCPV4)
static void handler(
  struct net_mgmt_event_callback *cb, uint32_t mgmt_event, struct net_if *iface)
{
  int i = 0;

  if (mgmt_event != NET_EVENT_IPV4_ADDR_ADD) {
    return;
  }

  for (i = 0; i < NET_IF_MAX_IPV4_ADDR; i++) {
    char buf[NET_IPV4_ADDR_LEN];

    if (iface->config.ip.ipv4->unicast[i].addr_type != NET_ADDR_DHCP) {
      continue;
    }

    printf("  IP address: %s\n",
      net_addr_ntop(AF_INET,
          &iface->config.ip.ipv4->unicast[i].address.in_addr,
              buf, sizeof(buf)));
    printf("  Lease time: %u seconds\n",
       iface->config.dhcpv4.lease_time);
    printf("  Netmask:    %s\n",
      net_addr_ntop(AF_INET,
               &iface->config.ip.ipv4->netmask,
               buf, sizeof(buf)));
    printf("  Gateway:    %s\n",
      net_addr_ntop(AF_INET,
             &iface->config.ip.ipv4->gw,
             buf, sizeof(buf)));

    k_sem_give(&got_address);
  }
}
#endif  // CONFIG_NET_DHCPV4

int main(void)
{
  // Initialize network interfaces
  std::vector<struct net_if *> ifs {};
  net_if_foreach(net_if_cb, &ifs);
  if (ifs.size() >= 1 && sizeof(CONFIG_NET_IFACE1_ADDR) > 1) {
    int ret = setup_iface(
      ifs[0],
      CONFIG_NET_IFACE1_ADDR,
      CONFIG_NET_IFACE1_GW,
      CONFIG_NET_IFACE1_NETMASK,
      CONFIG_NET_IFACE1_VLAN);
    if (ret) {
      return 1;
    }
  }
  if (ifs.size() >= 2 && sizeof(CONFIG_NET_IFACE2_ADDR) > 1) {
    int ret = setup_iface(
      ifs[1],
      CONFIG_NET_IFACE2_ADDR,
      CONFIG_NET_IFACE2_GW,
      CONFIG_NET_IFACE2_NETMASK,
      CONFIG_NET_IFACE2_VLAN);
    if (ret) {
      return 1;
    }
  }

#if defined(CONFIG_NET_DHCPV4)
  if(ifs.size() >= 1) {
    std::printf("Requesting a DHCP lease...\n");
    net_mgmt_init_event_callback(&mgmt_cb, handler, NET_EVENT_IPV4_ADDR_ADD);
    net_mgmt_add_event_callback(&mgmt_cb);
    net_dhcpv4_start(ifs[0]);

    /* Wait for a lease. */
    if (k_sem_take(&got_address, K_SECONDS(10)) != 0) {
      std::printf("Did not get a DHCP lease\n");
    }
  }
#endif  // CONFIG_NET_DHCPV4

  pthread_attr_t main_attr {};
  pthread_t pthread_main {};
  void * retval = NULL;
  size_t main_stacksize = K_THREAD_STACK_SIZEOF(main_stack_area);

  pthread_attr_init(&main_attr);
  pthread_attr_setstack(&main_attr, &main_stack_area, main_stacksize);
  pthread_create(&pthread_main, &main_attr, &main_thread, NULL);
  pthread_join(pthread_main, &retval);

  return 0;
}
