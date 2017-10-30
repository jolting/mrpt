/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <cstdint>
#include <iostream>
#include <string>
#include <memory>

namespace mrpt
{
/** Serial and networking devices and utilities */
namespace comms
{

class CBeast;

template <typename T> class CWebSocketImpl;

template<typename T>
std::ostream &operator <<(std::ostream &os, const CWebSocketImpl<T> &ws);

template<typename T>
std::istream &operator >>(std::istream &is, const CWebSocketImpl<T> &ws);

template <typename T>
class CWebSocketImpl
{
public:
	CWebSocketImpl(const std::string &bindToAddress, uint16_t port);
//private:
	std::unique_ptr<T> m_pImpl;
};  // End of class def.

using CWebSocket = CWebSocketImpl<CBeast>;


}  // End of namespace
}  // end of namespace
