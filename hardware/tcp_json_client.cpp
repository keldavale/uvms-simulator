#include <iostream>
#include <string>
#include <boost/asio.hpp>
#include "json.hpp"  // Changed from <nlohmann/json.hpp> to "json.hpp"

using boost::asio::ip::tcp;
using json = nlohmann::json;

// Function to handle received JSON data
void handle_json(const json& data) {
    std::cout << "Received JSON data:\n" << data.dump(4) << std::endl;
}

int main() {
    // Server details
    std::string host = "192.168.2.95";
    int port = 16171;

    try {
        // Create an IO context
        boost::asio::io_context io_context;

        // Resolve the host and port
        tcp::resolver resolver(io_context);
        tcp::resolver::results_type endpoints = resolver.resolve(host, std::to_string(port));

        // Create and connect the socket
        tcp::socket socket(io_context);
        std::cout << "Connecting to " << host << ":" << port << "..." << std::endl;
        boost::asio::connect(socket, endpoints);
        std::cout << "Connected successfully." << std::endl;

        // Buffer to store incoming data
        boost::asio::streambuf buffer;
        std::string data_buffer;

        while (true) {
            // Read until newline
            boost::system::error_code error;
            std::size_t bytes_transferred = boost::asio::read_until(socket, buffer, '\n', error);

            if (error) {
                if (error == boost::asio::error::eof) {
                    std::cout << "No more data received. Closing connection." << std::endl;
                } else {
                    std::cerr << "Error while reading: " << error.message() << std::endl;
                }
                break;
            }

            // Convert buffer to string
            std::istream is(&buffer);
            std::string line;
            std::getline(is, line);

            // Ignore empty lines
            if (line.empty()) {
                continue;
            }

            try {
                // Parse JSON
                json json_data = json::parse(line);
                handle_json(json_data);
            } catch (json::parse_error& e) {
                std::cerr << "Failed to decode JSON: " << e.what() << std::endl;
                std::cerr << "Received line: " << line << std::endl;
            }
        }

        // Socket is closed automatically when it goes out of scope
    } catch (std::exception& e) {
        std::cerr << "An error occurred: " << e.what() << std::endl;
    }

    return 0;
}



// {
//     "altitude": 0.07584815472364426,
//     "covariance": [
//         [
//             9.612158464733511e-07,
//             5.287523094921198e-07,
//             -2.482067884557182e-08
//         ],
//         [
//             5.287523094921198e-07,
//             9.510851555205591e-07,
//             -6.069751812276536e-09
//         ],
//         [
//             -2.482067884557182e-08,
//             -6.069751812276536e-09,
//             5.731113716933578e-08
//         ]
//     ],
//     "fom": 0.0012187137035652995,
//     "format": "json_v3.1",
//     "status": 0,
//     "time": 80.79552459716797,
//     "time_of_transmission": 1716833956699325,
//     "time_of_validity": 1716833956611093,
//     "transducers": [
//         {
//             "beam_valid": true,
//             "distance": 0.09440000355243683,
//             "id": 0,
//             "nsd": -85.89305877685547,
//             "rssi": -26.390926361083984,
//             "velocity": -0.0006506689824163914
//         },
//         {
//             "beam_valid": true,
//             "distance": 0.08260000497102737,
//             "id": 1,
//             "nsd": -99.83978271484375,
//             "rssi": -26.118144989013672,
//             "velocity": -0.0009426509495824575
//         },
//         {
//             "beam_valid": true,
//             "distance": -1,
//             "id": 2,
//             "nsd": -98.708251953125,
//             "rssi": -49.003028869628906,
//             "velocity": -0.0003290083259344101
//         },
//         {
//             "beam_valid": true,
//             "distance": 0.11800000071525574,
//             "id": 3,
//             "nsd": -95.7412338256836,
//             "rssi": -30.960033416748047,
//             "velocity": -0.0007235361263155937
//         }
//     ],
//     "type": "velocity",
//     "velocity_valid": true,
//     "vx": 0.00044141573016531765,
//     "vy": -0.00018516674754209816,
//     "vz": -0.0006241798400878906
// }