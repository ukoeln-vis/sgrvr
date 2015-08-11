// This file is distributed under the MIT license.
// See the LICENSE file for details

// connection.h

#pragma once

// Boost.ASIO needs _WIN32_WINNT
#ifdef _WIN32
#ifndef _WIN32_WINNT
#define _WIN32_WINNT 0x0501 // Require Windows XP or later
#endif
#endif

#include "message.h"

#include <boost/asio/io_service.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/strand.hpp>

#include <boost/signals2/connection.hpp>
#include <boost/signals2/signal.hpp>

namespace async
{

class Connection;
class ConnectionManager;

using ConnectionPointer = std::shared_ptr<Connection>;

//--------------------------------------------------------------------------------------------------
// Connection
//--------------------------------------------------------------------------------------------------

class Connection : public std::enable_shared_from_this<Connection>
{
    friend class ConnectionManager;

public:
    enum Reason { Read, Write };

    using SignalType = boost::signals2::signal<void (Reason reason, MessagePointer message, boost::system::error_code const& e)>;

public:
    // Constructor.
    Connection(ConnectionManager& manager);

    // Destructor.
    ~Connection();

    // Start reading from the socket
    void start();

    // Stop/Close the connection
    void stop();

    // Sets the handler for this connection
    // Thread-safe.
    void set_handler(SignalType::slot_function_type handler);

    // Removes the handler for this connection.
    // Thread-safe.
    void remove_handler();

    // Close the connection
    void close();

    // Sends a message to the other side.
    void write(MessagePointer message);

    // Sends a message to the other size.
    template <class It>
    void write(unsigned type, It first, It last)
    {
        write(makeMessage(type, first, last));
    }

    // Sends a message to the other side.
    template <class Cont>
    void write(unsigned type, Cont const& cont)
    {
        write(makeMessage(type, std::begin(cont), std::end(cont)));
    }

private:
    // The manager for this connection
    ConnectionManager& manager_;
    // The underlying socket.
    boost::asio::ip::tcp::socket socket_;
    // Signal (called from ConnectionManager if anything happens)
    SignalType signal_;
    // Slot
    boost::signals2::connection slot_;
};

} // namespace async
