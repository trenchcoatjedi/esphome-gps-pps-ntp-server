#include "ntp_server.h"
#include "esphome/core/log.h"
#include "esphome/components/gps_pps_time/gps_pps_time.h"

#include <sys/time.h>
#include <cerrno>
#include <cstring>

#ifdef USE_ESP_IDF
#include <fcntl.h>
#include <unistd.h>
#endif

namespace esphome {
namespace ntp_server {

static const char *const TAG = "ntp_server";

/// Offset between Unix epoch (1970) and NTP epoch (1900) in seconds
static const uint32_t NTP_UNIX_OFFSET = 2208988800UL;

/// NTP packet size
static const int NTP_PACKET_SIZE = 48;

// ---- Platform-specific setup / loop ----

#ifdef USE_ESP_IDF

void NTPServer::setup() {
  ESP_LOGI(TAG, "Setting up NTP server on port %u...", this->port_);

  this->socket_fd_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (this->socket_fd_ < 0) {
    ESP_LOGE(TAG, "Failed to create UDP socket: %d", errno);
    this->mark_failed();
    return;
  }

  // Set non-blocking
  int flags = fcntl(this->socket_fd_, F_GETFL, 0);
  fcntl(this->socket_fd_, F_SETFL, flags | O_NONBLOCK);

  struct sockaddr_in server_addr {};
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(this->port_);
  server_addr.sin_addr.s_addr = htonl(INADDR_ANY);

  if (bind(this->socket_fd_, (struct sockaddr *) &server_addr, sizeof(server_addr)) < 0) {
    ESP_LOGE(TAG, "Failed to bind UDP socket to port %u: %d", this->port_, errno);
    close(this->socket_fd_);
    this->socket_fd_ = -1;
    this->mark_failed();
    return;
  }

  ESP_LOGI(TAG, "NTP server listening on port %u", this->port_);
}

void NTPServer::loop() {
  if (this->socket_fd_ < 0)
    return;

  uint8_t buffer[NTP_PACKET_SIZE];
  struct sockaddr_in client_addr {};
  socklen_t client_len = sizeof(client_addr);

  int received = recvfrom(this->socket_fd_, buffer, sizeof(buffer), 0,
                          (struct sockaddr *) &client_addr, &client_len);

  if (received < NTP_PACKET_SIZE)
    return;

  if (!this->is_time_synchronized_()) {
    ESP_LOGD(TAG, "NTP request dropped (time not synchronized)");
    return;
  }

  uint8_t response[NTP_PACKET_SIZE];
  this->build_ntp_response_(buffer, response);

  sendto(this->socket_fd_, response, NTP_PACKET_SIZE, 0,
         (struct sockaddr *) &client_addr, client_len);

  ESP_LOGD(TAG, "NTP response sent");
}

#else  // Arduino platforms (RP2040, ESP32-Arduino, etc.)

void NTPServer::setup() {
  ESP_LOGI(TAG, "Setting up NTP server on port %u...", this->port_);

  if (!this->udp_.begin(this->port_)) {
    ESP_LOGE(TAG, "Failed to bind UDP to port %u", this->port_);
    this->mark_failed();
    return;
  }

  ESP_LOGI(TAG, "NTP server listening on port %u", this->port_);
}

void NTPServer::loop() {
  int packet_size = this->udp_.parsePacket();
  if (packet_size < NTP_PACKET_SIZE)
    return;

  uint8_t buffer[NTP_PACKET_SIZE];
  this->udp_.read(buffer, NTP_PACKET_SIZE);

  uint8_t response[NTP_PACKET_SIZE];
  this->build_ntp_response_(buffer, response);

  this->udp_.beginPacket(this->udp_.remoteIP(), this->udp_.remotePort());
  this->udp_.write(response, NTP_PACKET_SIZE);
  this->udp_.endPacket();

  ESP_LOGD(TAG, "NTP response sent");
}

#endif  // USE_ESP_IDF

// ---- Shared NTP logic ----

void NTPServer::build_ntp_response_(const uint8_t *request, uint8_t *response) {
  NTPTimestamp receive_ts = this->get_ntp_timestamp_();

  memset(response, 0, NTP_PACKET_SIZE);

  bool synced = this->is_time_synchronized_();
  if (synced) {
    // LI=0 (no warning), VN=4, Mode=4 (server)
    response[0] = 0x24;
    // Stratum 1 (primary reference)
    response[1] = 1;
  } else {
    // LI=3 (clock unsynchronized), VN=4, Mode=4 (server)
    response[0] = 0xE4;
    // Stratum 16 (unsynchronized)
    response[1] = 16;
  }
  // Poll interval (copy from request)
  response[2] = request[2];
  // Precision: ~1 microsecond = 2^-20 seconds
  response[3] = static_cast<uint8_t>(-20 & 0xFF);

  // Reference ID: "GPS\0"
  response[12] = 'G';
  response[13] = 'P';
  response[14] = 'S';
  response[15] = 0;

  // Reference timestamp (last sync time) = current time
  response[16] = (receive_ts.seconds >> 24) & 0xFF;
  response[17] = (receive_ts.seconds >> 16) & 0xFF;
  response[18] = (receive_ts.seconds >> 8) & 0xFF;
  response[19] = receive_ts.seconds & 0xFF;
  response[20] = (receive_ts.fraction >> 24) & 0xFF;
  response[21] = (receive_ts.fraction >> 16) & 0xFF;
  response[22] = (receive_ts.fraction >> 8) & 0xFF;
  response[23] = receive_ts.fraction & 0xFF;

  // Origin timestamp (copy client's transmit timestamp)
  memcpy(&response[24], &request[40], 8);

  // Receive timestamp
  response[32] = (receive_ts.seconds >> 24) & 0xFF;
  response[33] = (receive_ts.seconds >> 16) & 0xFF;
  response[34] = (receive_ts.seconds >> 8) & 0xFF;
  response[35] = receive_ts.seconds & 0xFF;
  response[36] = (receive_ts.fraction >> 24) & 0xFF;
  response[37] = (receive_ts.fraction >> 16) & 0xFF;
  response[38] = (receive_ts.fraction >> 8) & 0xFF;
  response[39] = receive_ts.fraction & 0xFF;

  // Transmit timestamp
  NTPTimestamp transmit_ts = this->get_ntp_timestamp_();
  response[40] = (transmit_ts.seconds >> 24) & 0xFF;
  response[41] = (transmit_ts.seconds >> 16) & 0xFF;
  response[42] = (transmit_ts.seconds >> 8) & 0xFF;
  response[43] = transmit_ts.seconds & 0xFF;
  response[44] = (transmit_ts.fraction >> 24) & 0xFF;
  response[45] = (transmit_ts.fraction >> 16) & 0xFF;
  response[46] = (transmit_ts.fraction >> 8) & 0xFF;
  response[47] = transmit_ts.fraction & 0xFF;
}

NTPTimestamp NTPServer::get_ntp_timestamp_() {
  struct timeval tv;
  gettimeofday(&tv, nullptr);

  NTPTimestamp ts;
  ts.seconds = static_cast<uint32_t>(tv.tv_sec) + NTP_UNIX_OFFSET;
  // Convert microseconds to NTP fraction: usec * 2^32 / 1000000
  ts.fraction = static_cast<uint32_t>((static_cast<uint64_t>(tv.tv_usec) << 32) / 1000000ULL);
  return ts;
}

bool NTPServer::is_time_synchronized_() {
  if (this->time_source_ == nullptr)
    return true;  // No time source configured, assume synchronized
  return this->time_source_->is_synchronized();
}

void NTPServer::dump_config() {
  ESP_LOGCONFIG(TAG, "NTP Server:");
  ESP_LOGCONFIG(TAG, "  Port: %u", this->port_);
  ESP_LOGCONFIG(TAG, "  Time source: %s", this->time_source_ != nullptr ? "configured" : "none");
}

}  // namespace ntp_server
}  // namespace esphome
