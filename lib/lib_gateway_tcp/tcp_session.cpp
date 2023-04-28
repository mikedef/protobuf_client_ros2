// Copyright 2020:
//   GobySoft, LLC (2017-)
//   Massachusetts Institute of Technology (2017-)
// File authors:
//   Toby Schneider <toby@gobysoft.org>
//
//
// This file was adopted from GobySoft's NETSIM Libraries and modified by Supun Randeni.


#include <boost/algorithm/string.hpp>
#include <iostream>

#include "tcp_session.h"

// session
gateway::tcp_session::tcp_session(std::unique_ptr<boost::asio::ip::tcp::socket> socket)
    : socket_(std::move(socket))
{
}

void gateway::tcp_session::start()
{
    self_ = shared_from_this();
    read();
}

void gateway::tcp_session::read()
{
    async_read_until(*socket_, buffer_, end_of_line_,
                     [this](boost::system::error_code ec, std::size_t length) {
                         if (!ec)
                         {
                             std::string preamble, pb_name, pb_data, pb_data_b64;
                             std::istream is(&buffer_);
                             std::getline(is, preamble, delimiter_);
                             std::getline(is, pb_name, delimiter_);
                             std::getline(is, pb_data_b64, end_of_line_);
                             pb_data = b64_decode(pb_data_b64);
                             auto rx_range = rx_callbacks_.equal_range(pb_name);

                             boost::asio::ip::tcp::endpoint remote;
                             // the remote may no longer be connected
                             try
                             {
                                 remote = socket_->remote_endpoint();
                             }
                             catch (boost::exception& e)
                             {
                             }

                             for (auto it = rx_range.first; it != rx_range.second; ++it)
                                 it->second->post(pb_name, pb_data, remote);
                             for (auto& rx : rx_all_) rx->post(pb_name, pb_data, remote);

                             read();
                         }
                         else
                         {
                             disconnect();
                         }
                     });
}

void gateway::tcp_session::write(const google::protobuf::Message& message)
{
    std::string bytes = preamble_ + delimiter_ + message.GetDescriptor()->full_name() + delimiter_ +
                        boost::erase_all_copy(b64_encode(message.SerializeAsString()), "\n") +
                        end_of_line_;

    // std::cout 
    //   << "Encoded bytes to send = "
    //   << bytes << " \n";

    boost::asio::async_write(*socket_, boost::asio::buffer(bytes),
                             [this](boost::system::error_code ec, std::size_t length) {
                                 if (ec)
                                 {
                                     disconnect();
                                 }
                             });
}

void gateway::tcp_session::disconnect() { self_.reset(); }

inline std::string gateway::tcp_session::b64_encode(const std::string& in)
    {
        std::stringstream instream(in);
        std::stringstream outstream;
        base64::encoder D;
        D.encode(instream, outstream);
        return outstream.str();
    }

inline std::string gateway::tcp_session::b64_decode(const std::string& in)
    {
        std::stringstream instream(in);
        std::stringstream outstream;
        base64::decoder D;
        D.decode(instream, outstream);
        return outstream.str();
    }
