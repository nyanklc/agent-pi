#include <iostream>
#include <string>
#include <thread>
#include <mutex>
#include <chrono>
#include <cstring>
#include <cstdlib>
#include <cstdio>

#include <wiringSerial.h>

class SerialCommunication {
public:
    SerialCommunication(const std::string& port, int baudRate) :
        m_port(port),
        m_baudRate(baudRate),
        m_running(true),
        m_messageToSend(""),
        m_latestReceived("")
    {
        m_serialPort = serialOpen(port.c_str(), baudRate);
        if (m_serialPort < 0) {
            std::cerr << "Error opening " << port << std::endl;
            exit(1);
        }
    }

    ~SerialCommunication() {
        m_running = false;
        if (m_communicationThread.joinable()) {
            m_communicationThread.join();
        }
        serialClose(m_serialPort);
    }

    void setMessageToBeSent(const std::string& msg) {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_messageToSend = msg;
    }

    std::string getLatestReceived() {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_latestReceived;
    }

    void startCommunication() {
        m_communicationThread = std::thread(&SerialCommunication::communicationLoop, this);
    }

private:
    void communicationLoop() {
        while (m_running) {
            // Send message
            {
                std::lock_guard<std::mutex> lock(m_mutex);
                if (!m_messageToSend.empty()) {
                    serialPrintf(m_serialPort, "%s\n", m_messageToSend.c_str());
                    m_messageToSend = "";
                }
            }

            // Receive message
            while (serialDataAvail(m_serialPort) > 0) {
                char c = serialGetchar(m_serialPort);
                if (c == '\n') {
                    std::lock_guard<std::mutex> lock(m_mutex);
                    m_latestReceived = m_currentReceived;
                    m_currentReceived = "";
                }
                else {
                    m_currentReceived += c;
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    std::string m_port;
    int m_baudRate;
    int m_serialPort;
    std::thread m_communicationThread;
    std::mutex m_mutex;
    bool m_running;
    std::string m_messageToSend;
    std::string m_latestReceived;
    std::string m_currentReceived;
};
