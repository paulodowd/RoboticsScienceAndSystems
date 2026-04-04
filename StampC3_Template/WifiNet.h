
#ifndef NETWORK_H
#define NETWORK_H
#include <WiFi.h>

class WifiAccessPointServer {
  public:
    WifiAccessPointServer(const char* ssid,
                          const char* password,
                          uint16_t port)
      : _ssid(ssid),
        _password(password),
        _port(port),
        _server(port) {
    }

    bool begin() {
      WiFi.mode(WIFI_AP);

      if (!WiFi.softAP(_ssid, _password)) {
        return false;
      }

      _server.begin();
      _server.setNoDelay(true);
      _started = true;
      return true;
    }

    void update() {
      if (!_started) {
        return;
      }

      // If current client has disconnected, close it
      if (_client && !_client.connected()) {
        _client.stop();
      }

      // Accept a new client only if there is no active one
      if ((!_client || !_client.connected()) && shouldCheckAccept()) {
        _lastAcceptCheckMs = millis();

        WiFiClient newClient = _server.accept();
        if (newClient) {
          _client = newClient;
          _client.setNoDelay(true);
        }
      }
    }

    bool clientConnected() {
      return _client && _client.connected();
    }

    size_t available() {
      if (!clientConnected()) {
        return 0;
      }
      return _client.available();
    }

    int read() {
      if (!clientConnected()) {
        return -1;
      }
      if (!_client.available()) {
        return -1;
      }
      return _client.read();
    }

    size_t readBytes(char* buffer, size_t maxLen) {
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

    bool send(const char* msg) {
      if (!clientConnected() || msg == nullptr) {
        return false;
      }

      size_t written = _client.print(msg);
      return written > 0;
    }

    bool sendLine(const char* msg) {
      if (!clientConnected() || msg == nullptr) {
        return false;
      }

      size_t written = _client.print(msg);
      written += _client.print("\n");
      return written > 0;
    }

    bool sendBuffer(const uint8_t* data, size_t len) {
      if (!clientConnected() || data == nullptr || len == 0) {
        return false;
      }

      size_t written = _client.write(data, len);
      return written == len;
    }

    void disconnectClient() {
      if (_client) {
        _client.stop();
      }
    }

    IPAddress ip() const {
      return WiFi.softAPIP();
    }

    int stationCount() const {
      return WiFi.softAPgetStationNum();
    }

    void setAcceptIntervalMs(unsigned long intervalMs) {
      _acceptIntervalMs = intervalMs;
    }

  private:
    bool shouldCheckAccept() const {
      return (millis() - _lastAcceptCheckMs) >= _acceptIntervalMs;
    }

    const char* _ssid;
    const char* _password;
    uint16_t _port;

    WiFiServer _server;
    WiFiClient _client;

    bool _started = false;
    unsigned long _lastAcceptCheckMs = 0;
    unsigned long _acceptIntervalMs = 100;
};
#endif
