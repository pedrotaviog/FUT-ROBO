#include "strategy/UnicastSender.hpp"

#include <print>

using asio::ip::udp;

UnicastSender::UnicastSender(std::string_view ip_address, int port)
    : m_io_context(), m_socket(m_io_context, udp::v4())
{
    auto address = asio::ip::make_address(ip_address);
    m_endpoint = udp::endpoint(address, port);
}

void UnicastSender::send(const TwistCmd& cmd) {
    send_bytes(&cmd, sizeof(TwistCmd));
}

void UnicastSender::send(const MotorPwmCmd& cmd) {
    send_bytes(&cmd, sizeof(MotorPwmCmd));
}

void UnicastSender::send_bytes(const void* data, std::size_t size) {
    asio::error_code ec;

    m_socket.send_to(asio::buffer(data, size), m_endpoint, 0, ec);

    if (ec) {
        std::println("Couldn't send command: {}", ec.message());
    }
}
