#include "dds/dds.h"
#include "CurvilinearSampleData.h"
#include "CartesianSampleData.h"
#include "TrajectorySampleData.h"
#include "ResultData.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "csv_parser.hpp"
#include <cstring>
#include <chrono>
#include <pthread.h>
#include "zephyr_app.hpp"

#define DDS_DOMAIN_ACTUATION 2
size_t counter = 1;

std::vector<std::map<std::string, std::vector<double>>> csv_data; 
const uint32_t array_size = 31; 
dds_entity_t writer;

std::chrono::time_point<std::chrono::high_resolution_clock> start;

using TrajectorySampleData = custom_trajectory_msgs_msg_TrajectorySampleData;
using CurvilinearSampleData = custom_trajectory_msgs_msg_CurvilinearSampleData;
using CartesianSampleData = custom_trajectory_msgs_msg_CartesianSampleData;
using ResultData = custom_trajectory_msgs_msg_ResultData;

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

  auto end = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
  std::cout << "Round-trip time for message: " << duration.count() << " milliseconds" << std::endl;

  auto process = *reinterpret_cast<std::function<void(const T &)> *>(arg);

  static T msg;
  if (get_msg(rd, reinterpret_cast<void *>(&msg))) {
    process(msg);
  }
}


void create_reader(
  dds_entity_t participant,
  const dds_topic_descriptor_t * desc,
  const char * name, const dds_qos_t * qos,
  dds_on_data_available_fn callback, void * arg)
{
  dds_entity_t topic = dds_create_topic(participant, desc, name, NULL, NULL);
  if (topic < 0) {
    std::cout << "dds_create_topic (" << name << "): " << dds_strretcode(-topic) << std::endl;
    std::exit(EXIT_FAILURE);
  }

  dds_listener_t * listener = dds_create_listener(arg);
  dds_lset_data_available(listener, callback);

  dds_entity_t reader = dds_create_reader(participant, topic, NULL, listener);
  if (reader < 0) {
    std::cout << "dds_create_reader (" << name << "): " << dds_strretcode(-reader) << std::endl;
    std::exit(EXIT_FAILURE);
  }

  dds_delete_listener(listener);
}

dds_entity_t create_writer(
  dds_entity_t participant,
  const dds_topic_descriptor_t * desc,
  const char * name, const dds_qos_t * qos)
  {
  dds_return_t rc;
  uint32_t status = 0;

  /* Create a Topic. */
  dds_entity_t topic = dds_create_topic (
    participant, desc, name, NULL, NULL);
  if (topic < 0)
    DDS_FATAL("dds_create_topic: %s\n", dds_strretcode(-topic));
    
  /* Create a Writer. */
  dds_entity_t writer = dds_create_writer (participant, topic, NULL, NULL);
  if (writer < 0)
    DDS_FATAL("dds_create_writer: %s\n", dds_strretcode(-writer));

  std::cout << "=== [Publisher]  Waiting for a reader to be discovered ..." << std::endl;

  rc = dds_set_status_mask(writer, DDS_PUBLICATION_MATCHED_STATUS);
  if (rc != DDS_RETCODE_OK)
    DDS_FATAL("dds_set_status_mask: %s\n", dds_strretcode(-rc));

  while(!(status & DDS_PUBLICATION_MATCHED_STATUS))
  {
    rc = dds_get_status_changes (writer, &status);
    if (rc != DDS_RETCODE_OK)
      DDS_FATAL("dds_get_status_changes: %s\n", dds_strretcode(-rc));

    /* Polling sleep. */
    dds_sleepfor (DDS_MSECS (20));
  }

  return writer;
}

/*
@brief Fills the data buffer of a dds message type.

@param seq DDS buffer
@param data Data to fill the buffer with
@param length Length of data
*/
static void fill_sequence(dds_sequence_double &seq, std::vector<double> &data, uint32_t length)
{
  seq._buffer  = (double*) malloc(length * sizeof(double));
  seq._length  = length;
  seq._maximum = length;
  seq._release = true;

  memcpy(seq._buffer, data.data(), length * sizeof(double));
}

/*
@brief Reads a trajectory from the csv data and sends it via CycloneDDS.
*/
void send_trajectory(){
  std::map<std::string, std::vector<double>> trajectory = csv_data.at(counter);
  counter++;

  TrajectorySampleData trajectory_data;
  trajectory_data.size = array_size;
  trajectory_data.m_actual_size = array_size;
  trajectory_data.m_d_t = 0.1;
  
  // Cartesian
  fill_sequence(trajectory_data.m_cartesian_sample.x, trajectory["x"], array_size);
  fill_sequence(trajectory_data.m_cartesian_sample.y, trajectory["y"], array_size);
  fill_sequence(trajectory_data.m_cartesian_sample.theta, trajectory["theta"], array_size);
  fill_sequence(trajectory_data.m_cartesian_sample.velocity, trajectory["v"], array_size);
  fill_sequence(trajectory_data.m_cartesian_sample.acceleration, trajectory["a"], array_size);
  fill_sequence(trajectory_data.m_cartesian_sample.kappa, trajectory["kappa"], array_size);
  fill_sequence(trajectory_data.m_cartesian_sample.kappa_dot, trajectory["kappa_dot"], array_size);

  // Curvilinear
  fill_sequence(trajectory_data.m_curvilinear_sample.s, trajectory["s"], array_size);
  fill_sequence(trajectory_data.m_curvilinear_sample.d, trajectory["d"], array_size);
  fill_sequence(trajectory_data.m_curvilinear_sample.theta, trajectory["theta_curv"], array_size);

  fill_sequence(trajectory_data.m_curvilinear_sample.dd, trajectory["d_dot"], array_size);
  fill_sequence(trajectory_data.m_curvilinear_sample.ddd, trajectory["d_ddot"], array_size);
  fill_sequence(trajectory_data.m_curvilinear_sample.ss, trajectory["s_dot"], array_size);
  fill_sequence(trajectory_data.m_curvilinear_sample.sss, trajectory["s_ddot"], array_size);

  std::cout << "=== [Publisher]  Writing..." << std::endl;
  dds_return_t rc = dds_write (writer, &trajectory_data);
  start = std::chrono::high_resolution_clock::now();

  if (rc != DDS_RETCODE_OK)
    DDS_FATAL("dds_write: %s\n", dds_strretcode(-rc));
}

/*
@brief Callback for ResultData messages.

@param msg Message
*/
void on_msg(const ResultData & msg)
{
   
    std::cout << "=== [Reader]  Received message..." << std::endl;
    std::cout << counter << std::endl;
    counter++;
    std::cout << "feasible: " << msg.m_feasible << std::endl;
    std::cout << "cost: " << msg.m_cost << std::endl;
    std::cout << std::endl;

    if(counter >= csv_data.size()){
      std::cout << "All trajectories sent" << std::endl;
      return;
    }
    
    send_trajectory();
}

/*
@brief Main function configures CycloneDDS and initializes message exchange.
*/
int main (void)
{
  dds_sequence_double seq;
  dds_entity_t participant;
  dds_qos_t *qos;


  csv_data = get_data();
  
  /* Create a Participant. */
  participant = dds_create_participant (DDS_DOMAIN_ACTUATION, NULL, NULL);
  if (participant < 0)
    DDS_FATAL("dds_create_participant: %s\n", dds_strretcode(-participant));


  qos = dds_create_qos();
  dds_qset_reliability(qos, DDS_RELIABILITY_RELIABLE, DDS_MSECS(30)); 

  /* Create a Writer */
  writer = create_writer(
    participant, 
    &custom_trajectory_msgs_msg_TrajectorySampleData_desc,
    "trajectory_sample_msg",
    qos
  );

  std::function<void(const ResultData &)> m_state_process = [&](const ResultData & msg) {on_msg(msg);};

  create_reader(
    participant,
    &custom_trajectory_msgs_msg_ResultData_desc,
    "result_data_msg",
    qos,
    on_msg_dds<ResultData>,
    reinterpret_cast<void *>(&m_state_process)
  );

  /* Writing first message so i get a response */
  send_trajectory();
  dds_delete_qos(qos);

  while(true){
    pause();
  }

  dds_return_t rc = dds_delete(participant);
  if (rc != DDS_RETCODE_OK)
    DDS_FATAL("dds_delete: %s\n", dds_strretcode(-rc));

  return EXIT_SUCCESS;
}
