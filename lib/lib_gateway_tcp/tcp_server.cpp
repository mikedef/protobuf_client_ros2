// Copyright 2020:
//   GobySoft, LLC (2017-)
//   Massachusetts Institute of Technology (2017-)
// File authors:
//   Toby Schneider <toby@gobysoft.org>
//
//
// This file was adopted from GobySoft's NETSIM Libraries and modified by Supun Randeni.

#include "tcp_server.h"

// server
gateway::tcp_server::tcp_server(boost::asio::io_service& io_service, short port)
    : acceptor_(io_service, boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), port)),
      socket_(io_service)
{
    accept();
}

void gateway::tcp_server::write(const google::protobuf::Message& message,
                               const boost::asio::ip::tcp::endpoint& ep)
{
    auto it = sessions_.find(ep);
    if (it != sessions_.end())
    {
        if (auto session = it->second.lock())
            session->write(message);
        else
            sessions_.erase(it);
    }
}

void gateway::tcp_server::accept()
{
    acceptor_.async_accept(socket_, [this](boost::system::error_code ec) {
        if (!ec)
        {
            std::unique_ptr<boost::asio::ip::tcp::socket> socket(
                new boost::asio::ip::tcp::socket(std::move(socket_)));
            auto session = std::make_shared<tcp_session>(std::move(socket));
            sessions_.insert(std::make_pair(session->remote_endpoint(), session));

            session->read_callback([this](const std::string& pb_name, const std::string& bytes,
                                          const boost::asio::ip::tcp::endpoint& ep) {
                auto rx_range = rx_callbacks_.equal_range(pb_name);
                for (auto it = rx_range.first; it != rx_range.second; ++it)
                    it->second->post(pb_name, bytes, ep);
            });

            session->start();
        }
        accept();
    });
}
