#pragma once
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/transport/Node.hh>
#include "ssb_core/pipeline.hpp"

namespace ssb_gazebo {
// Independent of physics updates: remains alive while the plugin destructor
// waits for rendering, writing and durable commit. No files or image copies.
class CaptureProgress {
 public:
  CaptureProgress(ssb::Pipeline& pipeline, const std::string& session,
                  const std::string& topic)
      : publisher_(node_.Advertise<gz::msgs::StringMsg>(topic)),
        worker_([this, &pipeline, session] {
          std::unique_lock<std::mutex> lock(mutex_);
          while (!stopped_) {
            lock.unlock();
            try {
              gz::msgs::StringMsg message;
              message.set_data(nlohmann::json({{"session", session},
                  {"capture", pipeline.Progress()}}).dump());
              publisher_.Publish(message);
            } catch (...) { /* Observer failure never aborts raw capture. */ }
            lock.lock();
            cv_.wait_for(lock, std::chrono::milliseconds(250), [this] { return stopped_; });
          }
        }) {}
  ~CaptureProgress() {
    { std::lock_guard<std::mutex> lock(mutex_); stopped_ = true; }
    cv_.notify_all();
    worker_.join();
  }
 private:
  gz::transport::Node node_;
  gz::transport::Node::Publisher publisher_;
  std::mutex mutex_;
  std::condition_variable cv_;
  bool stopped_ = false;
  std::thread worker_;
};
}  // namespace ssb_gazebo
