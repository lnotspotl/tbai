#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <spdlog/spdlog.h>

namespace tbai {

struct ResourceMetrics {
    double cpu_percent = 0.0;
    std::vector<double> cpu_cores_percent;  // Per-core CPU utilization
    double memory_percent = 0.0;
    double memory_used_mb = 0.0;
    double memory_total_mb = 0.0;
    std::chrono::steady_clock::time_point timestamp;
};

class ResourceMonitor {
   public:
    explicit ResourceMonitor(double infoFrequency = 1.0, double debugFrequency = 1.0, bool perCoreMetrics = false);
    ~ResourceMonitor();

    void startThread();
    void stopThread();

   private:
    void monitorLoop();
    ResourceMetrics collectMetrics();
    double getCpuUsage();
    std::vector<double> getCpuCoresUsage();
    ResourceMetrics getMemoryUsage();

    std::shared_ptr<spdlog::logger> logger_;
    std::unique_ptr<std::thread> monitor_thread_;

    bool running_;
    double infoFrequency_;
    double debugFrequency_;
    double updateFrequency_;
    bool perCoreMetrics_;
    mutable std::mutex metrics_mutex_;
    ResourceMetrics latest_metrics_;

    std::chrono::steady_clock::time_point last_cpu_time_;
    long long last_cpu_idle_ = 0;
    long long last_cpu_total_ = 0;

    // Per-core CPU tracking
    std::vector<long long> last_cpu_cores_idle_;
    std::vector<long long> last_cpu_cores_total_;
    std::chrono::steady_clock::time_point last_cpu_cores_time_;
    int num_cores_ = 0;
};

}  // namespace tbai