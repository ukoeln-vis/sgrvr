// This file is distributed under the MIT license.
// See the LICENSE file for details

// message.h

#pragma once

#include <boost/uuid/uuid.hpp>

#include <assert.h>

#include <memory>
#include <stdexcept>
#include <vector>

namespace async
{

class Message;

using MessagePointer = std::shared_ptr<Message>;

//--------------------------------------------------------------------------------------------------
// Message
//--------------------------------------------------------------------------------------------------

class Message
{
    friend class Connection;
    friend class ConnectionManager;

    struct Header
    {
        // The unique ID of this message
        boost::uuids::uuid id_; // POD, 16 bytes
        // The type of this message
        unsigned type_;
        // The length of this message
        unsigned size_;

        Header();
        Header(boost::uuids::uuid const& id, unsigned type, unsigned size);

        ~Header();
    };

public:
    using DataType = std::vector<char>;

private:
    // The message data
    DataType data_;
    // The message header
    Header header_;

public:
    Message();

    explicit Message(unsigned type);

    // Creates a message from the given buffer.
    template<class It>
    explicit Message(unsigned type, It first, It last)
        : data_(first, last)
        , header_(GenerateID(), type, static_cast<unsigned>(data_.size()))
    {
    }

    ~Message();

    // Returns the unique ID of this message
    boost::uuids::uuid const& id() const
    {
        return header_.id_;
    }

    // Returns the type of this message
    unsigned type() const
    {
        return header_.type_;
    }

    // Returns the size of the message
    unsigned size() const
    {
        assert( header_.size_ == data_.size() );
        return static_cast<unsigned>(data_.size());
    }

    // Returns an iterator to the first element of the data
    DataType::iterator begin()
    {
        return data_.begin();
    }

    // Returns an iterator to the element following the last element of the data
    DataType::iterator end()
    {
        return data_.end();
    }

    // Returns an iterator to the first element of the data
    DataType::const_iterator begin() const
    {
        return data_.begin();
    }

    // Returns an iterator to the element following the last element of the data
    DataType::const_iterator end() const
    {
        return data_.end();
    }

    // Swaps the data buffer with the given buffer and resets the header.
    void swap_data(DataType& buffer)
    {
        data_.swap(buffer);
        header_ = {};
    }

    // Returns a pointer to the data
    char* data()
    {
        return data_.data();
    }

    // Returns a pointer to the data
    char const* data() const
    {
        return data_.data();
    }

private:
    // Creates a new unique ID for this message
    static boost::uuids::uuid GenerateID();
};

inline MessagePointer makeMessage(unsigned type = 0)
{
    return std::make_shared<Message>(type);
}

template<class It>
inline MessagePointer makeMessage(unsigned type, It first, It last)
{
    return std::make_shared<Message>(type, first, last);
}

} // namespace async
