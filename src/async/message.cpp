// This file is distributed under the MIT license.
// See the LICENSE file for details

// message.cpp

#include "message.h"

#include <boost/uuid/uuid_generators.hpp>

using namespace async;

//--------------------------------------------------------------------------------------------------
// Message::Header
//--------------------------------------------------------------------------------------------------

Message::Header::Header()
    : id_(boost::uuids::nil_uuid())
    , type_(0)
    , size_(0)
{
}

Message::Header::Header(boost::uuids::uuid const& id, unsigned type, unsigned size)
    : id_(id)
    , type_(type)
    , size_(size)
{
}

Message::Header::~Header()
{
}

//--------------------------------------------------------------------------------------------------
// Message
//--------------------------------------------------------------------------------------------------

Message::Message()
{
}

Message::Message(unsigned type)
    : data_()
    , header_(boost::uuids::nil_uuid(), type, 0)
{
}

Message::~Message()
{
}

boost::uuids::uuid Message::GenerateID()
{
    static boost::uuids::random_generator gen;
    return gen();
}
