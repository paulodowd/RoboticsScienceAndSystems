#include "RobotWifiAP.h"

RobotWifiAP_c::RobotWifiAP_c(const char* ssid, const char* password, uint16_t port)
  : _ssid(ssid), _password(password), _port(port), _server(port) {
}

bool RobotWifiAP_c::begin() {
  WiFi.mode(WIFI_AP);

  if (!WiFi.softAP(_ssid, _password)) {
    return false;
  }

  _server.begin();
  _server.setNoDelay(true);
  _started = true;
  return true;
}

void RobotWifiAP_c::update() {
  if (!_started) {
    return; // not ready
  }

  if (_client && !_client.connected()) {
    _client.stop();
  }

  if ((!_client || !_client.connected()) && shouldCheckAccept()) {
    _lastAcceptCheckMs = millis();

    WiFiClient newClient = _server.accept();
    if (newClient) {
      _client = newClient;
      _client.setNoDelay(true);
    }
  }

}

bool RobotWifiAP_c::clientConnected() {
  return _client && _client.connected();
}

size_t RobotWifiAP_c::available() {
  if (!clientConnected()) {
    return 0;
  }
  return _client.available();
}

int RobotWifiAP_c::read() {
  if (!clientConnected() || !_client.available()) {
    return -1;
  }
  return _client.read();
}

size_t RobotWifiAP_c::readBytes(char* buffer, size_t maxLen) {
  if (!clientConnected() || buffer == nullptr || maxLen == 0) {
    return 0;
  }

  size_t count = 0;
  while (count < maxLen && _client.available()) {
    int c = _client.read();
    if (c < 0) {
      break;
    }
    buffer[count++] = (char)c;
  }
  return count;
}

bool RobotWifiAP_c::send(const char* msg) {
  if (!clientConnected() || msg == nullptr) {
    return false;
  }

  size_t written = _client.print(msg);
  return written > 0;
}

bool RobotWifiAP_c::sendLine(const char* msg) {
  if (!clientConnected() || msg == nullptr) {
    return false;
  }

  size_t written = _client.print(msg);
  written += _client.print("\n");
  return written > 0;
}

bool RobotWifiAP_c::sendBuffer(const uint8_t* data, size_t len) {
  if (!clientConnected() || data == nullptr || len == 0) {
    return false;
  }

  size_t written = _client.write(data, len);
  return written == len;
}

void RobotWifiAP_c::disconnectClient() {
  if (_client) {
    _client.stop();
  }
}

IPAddress RobotWifiAP_c::ip() const {
  return WiFi.softAPIP();
}

int RobotWifiAP_c::stationCount() const {
  return WiFi.softAPgetStationNum();
}

void RobotWifiAP_c::setAcceptIntervalMs(unsigned long intervalMs) {
  _acceptIntervalMs = intervalMs;
}

bool RobotWifiAP_c::printf(const char* format, ...) {
  if (!clientConnected() || format == nullptr) {
    return false;
  }

  char buffer[256];
  va_list args;
  va_start(args, format);
  int len = vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);

  if (len <= 0) {
    return false;
  }
  if ((size_t)len >= sizeof(buffer)) {
    len = sizeof(buffer) - 1;
  }

  size_t written = _client.write((const uint8_t*)buffer, len);
  return written == (size_t)len;
}

bool RobotWifiAP_c::printlnf(const char* format, ...) {
  if (!clientConnected() || format == nullptr) {
    return false;
  }

  char buffer[256];
  va_list args;
  va_start(args, format);
  int len = vsnprintf(buffer, sizeof(buffer) - 2, format, args);
  va_end(args);

  if (len <= 0) {
    return false;
  }
  if (len > (int)(sizeof(buffer) - 2)) {
    len = sizeof(buffer) - 2;
  }

  buffer[len++] = '\n';
  buffer[len] = '\0';

  size_t written = _client.write((const uint8_t*)buffer, len);
  return written == (size_t)len;
}

bool RobotWifiAP_c::shouldCheckAccept() const {
  return (millis() - _lastAcceptCheckMs) >= _acceptIntervalMs;
}
