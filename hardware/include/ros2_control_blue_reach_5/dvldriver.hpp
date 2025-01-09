// Copyright 2025, Edward Morgan
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#pragma once
#ifndef ROS2_CONTROL_BLUE_REACH_5__DVLDRIVER_HPP_
#define ROS2_CONTROL_BLUE_REACH_5__DVLDRIVER_HPP_

#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <boost/asio.hpp>
#include "json.hpp"  // nlohmann JSON

namespace a50dvl
{
namespace driver
{

/**
 * @brief A lightweight TCP-based driver that connects to a DVL and
 * reads JSON lines at runtime, invoking a callback when new data arrives.
 */
class DVLDriver
{
public:
  /**
   * @brief Construct a new DVLDriver object
   */
  DVLDriver();

  /**
   * @brief Destroy the DVLDriver object
   *
   * Automatically stops the driver thread if it is running.
   */
  ~DVLDriver();

  /**
   * @brief Start the driver by connecting to the DVL's TCP port
   * and spinning an internal thread that reads JSON lines.
   *
   * @param host The hostname or IP of the DVL
   * @param port The port on which the DVL is streaming data
   * @throws std::runtime_error if the driver is already running or if connection fails
   */
  void start(const std::string & host, int port);

  /**
   * @brief Stop the driver by closing the TCP connection
   * and terminating the reading thread.
   */
  void stop();

  /**
   * @brief Subscribe to new JSON messages read from the DVL.
   *
   * @param callback A function that will be invoked whenever the driver parses
   *        a new JSON line from the DVL.
   */
  void subscribe(const std::function<void(const nlohmann::json &)> & callback);

private:
  /**
   * @brief The internal thread routine that continuously blocks on TCP reads,
   *        parses JSON lines, and invokes the callback (if set).
   */
  void pollData();

private:
  std::atomic<bool> running_;
  std::thread poll_thread_;

  // Connection details
  std::string host_;
  int port_;

  // Boost ASIO objects
  boost::asio::io_context io_context_;
  std::unique_ptr<boost::asio::ip::tcp::socket> socket_;

  // Optional callback for incoming JSON
  std::function<void(const nlohmann::json &)> callback_;
};

}  // namespace driver
}  // namespace a50dvl

#endif // ROS2_CONTROL_BLUE_REACH_5__DVLDRIVER_HPP_