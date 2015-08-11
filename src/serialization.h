// This file is distributed under the MIT license.
// See the LICENSE file for details.

#pragma once

#include <cstring> // memcpy

#include <visionaray/math/math.h>

namespace visionaray
{
namespace splatrend
{

//-------------------------------------------------------------------------------------------------
//
//

template <typename VectorT, typename VectorU>
void deserialize(VectorT& dst, VectorU const& bytes)
{
    dst.resize(bytes.size() / sizeof(typename VectorT::value_type));
    std::memcpy(dst.data(), bytes.data(), bytes.size());
}

template <typename VectorT>
void deserialize(VectorT& bytes, char const* first, char const* last)
{
    size_t len = (size_t)(last - first);

    bytes.resize(len / sizeof(typename VectorT::value_type));
    std::memcpy(bytes.data(), first, len);
}

template <typename VectorU, typename VectorT>
void serialize(VectorU& bytes, VectorT const& src)
{
    bytes.resize(src.size() * sizeof(typename VectorT::value_type));
    std::memcpy(bytes.data(), src.data(), bytes.size());
}

} // splatrend
} // visionaray
