// This file is distributed under the MIT license.
// See the LICENSE file for details

// connection_manager.h

#include "connection.h"

#include <boost/thread.hpp>

#include <deque>
#include <vector>
#include <set>

namespace async
{

class Connection;
class ConnectionManager;

using ConnectionManagerPointer = std::shared_ptr<ConnectionManager>;

//--------------------------------------------------------------------------------------------------
// ConnectionManager
//--------------------------------------------------------------------------------------------------

class ConnectionManager : public std::enable_shared_from_this<ConnectionManager>
{
    friend class Connection;

public:
    using Handler = std::function<bool(ConnectionPointer conn, boost::system::error_code const& e)>;

public:
    ConnectionManager();
    explicit ConnectionManager(unsigned short port);

    ~ConnectionManager();

    // Starts the message loop
    void run();

    // Starts a new thread which in turn starts the message loop
    void run_in_thread();

    // Wait for the thread to finish
    void wait();

    // Stops the message loop
    void stop();

    // Starts a new accept operation.
    // Use bind_port() to specifiy the port.
    void accept(Handler handler);

    // Starts a new connect operation
    void connect(std::string const& host, unsigned short port, Handler handler);

    // Starts a new connect operation and waits until the connection is connected
    ConnectionPointer connect(std::string const& host, unsigned short port);

    // Returns an existing connection or creates a new one
    ConnectionPointer get_or_connect(std::string const& host, unsigned short port);

    // Close the given connection
    void close(ConnectionPointer conn);

    // Close all sockets
    void close_all();

    // Search for an existing connection
    ConnectionPointer find(std::string const& host, unsigned short port);

private:
    // Start an accept operation
    void do_accept(Handler handler);

    // Handle completion of a accept operation.
    void handle_accept(boost::system::error_code const& e, ConnectionPointer conn, Handler handler);

    // Starts a new connect operation.
    void do_connect(std::string const& host, unsigned short port, Handler handler);

    // Handle completion of a connect operation.
    void handle_connect(boost::system::error_code const& e, ConnectionPointer conn, Handler handler);

    // Read the next message from the given client.
    void do_read(ConnectionPointer conn);

    // Called when a message header is read.
    void handle_read_header(boost::system::error_code const& e, MessagePointer message, ConnectionPointer conn);

    // Called when a complete message is read.
    void handle_read_data(boost::system::error_code const& e, MessagePointer message, ConnectionPointer conn);

    // Sends a message to the given client
    void write(MessagePointer msg, ConnectionPointer conn);

    // Starts a new write operation.
    void do_write(MessagePointer msg, ConnectionPointer conn);

    // Write the next message
    void do_write_0();

    // Called when a complete message is written.
    void handle_write(boost::system::error_code const& e, MessagePointer message, ConnectionPointer conn);

    // Add a new connection
    void add_connection(ConnectionPointer conn);

    // Remove an existing connection
    void remove_connection(ConnectionPointer conn);

private:
    using Connections = std::set<ConnectionPointer>;
    using Messages = std::deque<std::pair<ConnectionPointer, MessagePointer> >;

    // The IO service
    boost::asio::io_service io_service_;
    // The acceptor object used to accept incoming socket connections.
    boost::asio::ip::tcp::acceptor acceptor_;
    // To protect the list of messages...
    boost::asio::strand strand_;
    // To keep the io_service running...
    std::shared_ptr<boost::asio::io_service::work> work_;
    // The list of active connections
    Connections connections_;
    // List of messages to be written
    Messages write_queue_;
    // A thread to process the message queue
    boost::thread runner_;
};

inline ConnectionManagerPointer makeConnectionManager()
{
    return std::make_shared<ConnectionManager>();
}

inline ConnectionManagerPointer makeConnectionManager(unsigned short port)
{
    return std::make_shared<ConnectionManager>(port);
}

} // namespace async
