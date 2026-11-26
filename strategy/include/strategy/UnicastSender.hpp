#pragma once

#include <cstddef>
#include <string_view>
#include <asio.hpp>
#include "common/protocol.h"

class UnicastSender {
public:
    UnicastSender(std::string_view ip_address, int port);
    void send(const TwistCmd& cmd);
    void send(const MotorPwmCmd& cmd);

private:
    void send_bytes(const void* data, std::size_t size);

    asio::io_context m_io_context;
    asio::ip::udp::socket m_socket;
    asio::ip::udp::endpoint m_endpoint;
};
