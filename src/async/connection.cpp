// This file is distributed under the MIT license.
// See the LICENSE file for details

// connection.cpp

#include "connection.h"
#include "connection_manager.h"

#include <boost/asio/buffer.hpp>
#include <boost/asio/connect.hpp>
#include <boost/asio/placeholders.hpp>
#include <boost/asio/read.hpp>
#include <boost/asio/write.hpp>

using namespace async;

using boost::asio::ip::tcp;

//--------------------------------------------------------------------------------------------------
// Connection
//--------------------------------------------------------------------------------------------------

Connection::Connection(ConnectionManager& manager)
    : manager_(manager)
    , socket_(manager.io_service_)
{
#ifndef NDEBUG
    std::cout << "Connection::Connection [" << (void*)this << "]\n";
#endif
}

Connection::~Connection()
{
#ifndef NDEBUG
    std::cout << "Connection::~Connection [" << (void*)this << "]\n";
#endif

#if 0
    close(); // NEIN!!!
#else
    remove_handler();
#endif
}

void Connection::start()
{
}

void Connection::stop()
{
}

void Connection::set_handler(SignalType::slot_function_type handler)
{
    // Remove existing handler.
    // Only a single handler is currently supported.
    remove_handler();

    slot_ = signal_.connect(handler);
}

void Connection::remove_handler()
{
    signal_.disconnect(slot_);
}

void Connection::close()
{
    manager_.close(shared_from_this());
}

void Connection::write(MessagePointer message)
{
    manager_.write(message, shared_from_this());
}
