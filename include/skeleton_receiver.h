#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <string>
#include <iostream>
#include <thread>

#include <Eigen/Dense>
#include <zmq.hpp>


class SkeletonZmqSubscriber {
  public:
    // Costruttore
    explicit SkeletonZmqSubscriber(int device_id, const std::string& topic, int port)
        : topic_(topic), device_id_(device_id), port_(port), context_(1), socket_(context_, zmq::socket_type::sub) {}

    // Distruttore
    ~SkeletonZmqSubscriber() { stop(); }

    // Avvia il thread di ricezione
    void start() {
      if (running_.exchange(true)) return;
      worker_ = std::thread([this]() { loop(); });
    }

    // Ferma il thread di ricezione
    void stop() {
      if (!running_.exchange(false)) return;
      if (worker_.joinable()) worker_.join();
    }

    std::string get_skeleton_data() {
      return skeleton_data_;
    }


  private:
    // Loop principale del thread worker
    void loop() {


      // socket_.set(zmq::sockopt::conflate, 1);
      // socket_.set(zmq::sockopt::rcvtimeo, 1000);

      std::string sub_topic = topic_ + "_" + std::to_string(device_id_);
      socket_.set(zmq::sockopt::subscribe, sub_topic);
      socket_.connect("tcp://localhost:" + std::to_string(port_ + device_id_));
      
      // std::this_thread::sleep_for(std::chrono::milliseconds(200));



      while (running_.load(std::memory_order_relaxed)) {
        zmq::message_t msg;
        auto result = socket_.recv(msg, zmq::recv_flags::none);

        if (!result) {
            std::cout << "Timeout / no message\n";
            continue;
        }

        // std::cout << "Bytes: " << msg.size() << "\n";
        std::string text(static_cast<char*>(msg.data()), msg.size());
        skeleton_data_ = text;
        // std::cout << "Text: " << skeleton_data_ << "\n";
      }
    }

    std::atomic<bool> running_{false};
    std::thread worker_;

    std::string topic_;
    int device_id_;
    int port_;

    std::string skeleton_data_;
    zmq::context_t context_;
    zmq::socket_t socket_;
    zmq::socket_t* self_socket_;
};
