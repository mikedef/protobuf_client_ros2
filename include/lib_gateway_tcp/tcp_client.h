// Copyright 2020:
//   GobySoft, LLC (2017-)
//   Massachusetts Institute of Technology (2017-)
// File authors:
//   Toby Schneider <toby@gobysoft.org>
//
//
// This file was adopted from GobySoft's NETSIM Libraries and modified by Supun Randeni.

#ifndef TCP_CLIENT_GATEWAY_H
#define TCP_CLIENT_GATEWAY_H

#include "tcp_session.h"

namespace gateway
{
class tcp_client : public tcp_session
{
  public:
    static std::shared_ptr<tcp_client> create(boost::asio::io_service& io_service);

    void connect(const std::string& server, unsigned short port);

  private:
    tcp_client(boost::asio::io_service& io_service);

  private:
    boost::asio::io_service& io_service_;
};

} // namespace gateway

#endif
