// Copyright 2020:
//   GobySoft, LLC (2017-)
//   Massachusetts Institute of Technology (2017-)
// File authors:
//   Toby Schneider <toby@gobysoft.org>
//
//
// This file was adopted from GobySoft's NETSIM Libraries and modified by Supun Randeni.

#include "tcp_client.h"

std::shared_ptr<gateway::tcp_client> gateway::tcp_client::create(boost::asio::io_service& io_service)
{
    return std::shared_ptr<tcp_client>(new tcp_client(io_service));
}

gateway::tcp_client::tcp_client(boost::asio::io_service& io_service)
    : tcp_session(std::unique_ptr<boost::asio::ip::tcp::socket>(
          new boost::asio::ip::tcp::socket(io_service))),
      io_service_(io_service)
{
}

void gateway::tcp_client::connect(const std::string& server, unsigned short port)
{
    socket().reset(new boost::asio::ip::tcp::socket(io_service_));
    if (io_service_.stopped())
        io_service_.reset();

    boost::asio::ip::tcp::resolver resolver(io_service_);
    boost::asio::ip::tcp::resolver::query query(server, std::to_string(port));
    auto endpoints = resolver.resolve(query);

    boost::asio::async_connect(
        *socket(), endpoints,
#if BOOST_VERSION < 106600
        [this](const boost::system::error_code& error, boost::asio::ip::tcp::resolver::iterator i)
#else
        [this](const boost::system::error_code& error, boost::asio::ip::tcp::endpoint i)
#endif
        {
            if (!error)
                start();
            else
                throw(error);
        });
}
