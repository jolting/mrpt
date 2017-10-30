/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "comms-precomp.h"  // Precompiled headers

#include <mrpt/comms/CWebSocket.h>
#define BOOST_COROUTINE_NO_DEPRECATION_WARNING
#define BOOST_COROUTINES_NO_DEPRECATION_WARNING
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/spawn.hpp>
#include <boost/beast/core/ostream.hpp>

#include <list>
#include <condition_variable>
#include <queue>
#include <mutex>
#include <thread>

namespace mrpt
{
namespace comms
{
using tcp = boost::asio::ip::tcp;  // from <boost/asio/ip/tcp.hpp>
namespace websocket =
	boost::beast::websocket;  // from <boost/beast/websocket.hpp>

//------------------------------------------------------------------------------

// Report a failure

//------------------------------------------------------------------------------

class CBeast
{
   public:
	CBeast(
		const std::string& bindTo, uint16_t port, unsigned int threadCount = 1)
		: m_ios(threadCount)
	{
		auto const address = boost::asio::ip::address::from_string(bindTo);
		boost::asio::spawn(
			m_ios, std::bind(
					   &CBeast::do_listen, this, std::ref(m_ios),
					   tcp::endpoint{address, port}, std::placeholders::_1));

		for (unsigned int i = 0; i < threadCount; i++)
		{
			m_threads.emplace_back([& ios = m_ios]() { ios.run(); });
		}
	}

	void fail(boost::system::error_code ec, char const* what)
	{
		std::cerr << what << ": " << ec.message() << "\n";
	}

	// Accepts incoming connections and launches the sessions
	void do_listen(
		boost::asio::io_service& ios, tcp::endpoint endpoint,
		boost::asio::yield_context yield)
	{
		boost::system::error_code ec;

		// Open the acceptor
		tcp::acceptor acceptor(ios);
		acceptor.open(endpoint.protocol(), ec);
		if (ec) return fail(ec, "open");

		// Bind to the server address
		acceptor.bind(endpoint, ec);
		if (ec) return fail(ec, "bind");

		// Start listening for connections
		acceptor.listen(boost::asio::socket_base::max_connections, ec);
		if (ec) return fail(ec, "listen");

		for (;;)
		{
			tcp::socket socket(ios);
			acceptor.async_accept(socket, yield[ec]);
			if (ec)
				fail(ec, "accept");
			else
				boost::asio::spawn(
					acceptor.get_io_service(),
					std::bind(
						&CBeast::do_session,this, std::move(socket), std::placeholders::_1));
		}
	}

	using SocketList = std::list<websocket::stream<tcp::socket>>;
	SocketList m_sockets;
	std::queue<boost::beast::multi_buffer> m_recvQueue;

	std::mutex m_recvMutex;
	std::mutex m_listMutex;
	std::mutex m_wsMutex;
	std::condition_variable m_cvRecv;

	void do_session(tcp::socket& socket, boost::asio::yield_context yield)
	{
		boost::system::error_code ec;

		// Construct the stream by moving in the socket
		websocket::stream<tcp::socket> ws(std::move(socket));

		// Accept the websocket handshake
		ws.async_accept(yield[ec]);
		if (ec)
			return fail(ec, "accept");

		SocketList::iterator it;
		{
			std::lock_guard<std::mutex> lock(m_listMutex);
			m_sockets.emplace_front(std::move(ws));
			it = m_sockets.begin();
		}


		while (ec == websocket::error::closed)
		{
			// This buffer will hold the incoming message
			boost::beast::multi_buffer buffer;
			// Read a message
			{
				std::lock_guard<std::mutex> lock(m_wsMutex);
				ws.async_read(buffer, yield[ec]);
			}

			if (ec)
			{
				fail(ec, "write");
				break;
			}

			std::lock_guard<std::mutex> lock(m_recvMutex);
			m_recvQueue.emplace(buffer);
			m_cvRecv.notify_one();
		}

		{
			std::lock_guard<std::mutex> lock(m_listMutex);
			m_sockets.erase(it);
		}
	}

	void WriteStream(std::istream& is)
	{
		boost::beast::multi_buffer buffer;
		boost::beast::ostream(buffer) << is.rdbuf();
		std::lock_guard<std::mutex> lock(m_wsMutex);
		for(auto &ws: m_sockets)
		{
			ws.write(buffer.data());
		}
	}

	void ReadStream(std::ostream& os)
	{
		std::unique_lock<std::mutex> lock(m_recvMutex);
		m_cvRecv.wait(lock, [&](){return !m_recvQueue.empty();});

		os << boost::beast::buffers(m_recvQueue.front().data());
 		m_recvQueue.pop();
	}

	~CBeast()
	{
		m_ios.stop();
		for (auto& thread : m_threads)
		{
			thread.join();
		}
	}

   private:
	boost::asio::io_service m_ios;
	std::vector<std::thread> m_threads;
};

template <>
CWebSocket::CWebSocketImpl(const std::string& bindTo, uint16_t port)
	: m_pImpl(std::make_unique<CBeast>(bindTo, port))
{
}

template <>
std::ostream& operator << <CBeast>(
	std::ostream& os, const CWebSocketImpl<CBeast>& ws)
{
	ws.m_pImpl->ReadStream(os);
	return os;
}

template <>
std::istream& operator >> <CBeast>(
	std::istream& is, const CWebSocketImpl<CBeast>& ws)
{
	ws.m_pImpl->WriteStream(is);
	return is;
}

}  // namespace comms
}  // namespace mrpt
