// Copyright 2020:
//   GobySoft, LLC (2017-)
//   Massachusetts Institute of Technology (2017-)
// File authors:
//   Toby Schneider <toby@gobysoft.org>
//
//
// This file was adopted from GobySoft's NETSIM Libraries and modified by Supun Randeni.

#ifndef TCP_SERVER_VECTORS_H
#define TCP_SERVER_VECTORS_H

#include <list>

#include "tcp_session.h"

namespace gateway
{
class tcp_server
{
  public:
    tcp_server(boost::asio::io_service& io_service, short port);
    void write(const google::protobuf::Message& message, const boost::asio::ip::tcp::endpoint& ep);

    template <typename ProtobufMessage>
    void read_callback(typename Receive<ProtobufMessage>::CallbackType f)
    {
        std::unique_ptr<ReceiveBase> rx(new Receive<ProtobufMessage>(f));
        rx_callbacks_.insert(
            std::make_pair(ProtobufMessage::descriptor()->full_name(), std::move(rx)));

        // std::cout  << "RX server" <<  " \n";
    }

  private:
    void accept();
    std::map<boost::asio::ip::tcp::endpoint, std::weak_ptr<tcp_session>> sessions_;
    boost::asio::ip::tcp::acceptor acceptor_;
    boost::asio::ip::tcp::socket socket_;

    std::unordered_multimap<std::string, std::unique_ptr<ReceiveBase>> rx_callbacks_;
};
} // namespace 

#endif
