#include "tbai_core/ResourceMonitor.hpp"

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "tbai_core/Asserts.hpp"
#include "tbai_core/Logging.hpp"

namespace tbai {

ResourceMonitor::ResourceMonitor(double infoFrequency, double debugFrequency, bool perCoreMetrics)
    : infoFrequency_(infoFrequency),
      debugFrequency_(debugFrequency),
      perCoreMetrics_(perCoreMetrics),
      logger_(tbai::getLogger("resource_monitor")),
      running_(false) {
    TBAI_ASSERT(infoFrequency > 0, "Info frequency must be greater than 0");
    TBAI_ASSERT(debugFrequency > 0, "Debug frequency must be greater than 0");

    updateFrequency_ = std::max(infoFrequency, debugFrequency);
    TBAI_LOG_INFO(logger_, "ResourceMonitor initialized with info frequency: {:.2f} Hz, debug frequency: {:.2f} Hz",
                  infoFrequency, debugFrequency);

    last_cpu_time_ = std::chrono::steady_clock::now();
    last_cpu_cores_time_ = std::chrono::steady_clock::now();

    // Initialize per-core tracking
    std::ifstream file("/proc/stat");
    std::string line;
    num_cores_ = 0;

    // Count CPU cores by reading /proc/stat
    while (std::getline(file, line)) {
        if (line.substr(0, 3) == "cpu" && line[3] >= '0' && line[3] <= '9') {
            num_cores_++;
        }
    }

    TBAI_LOG_INFO(logger_, "ResourceMonitor initialized with {} CPU cores", num_cores_);

    // Initialize vectors for per-core tracking
    last_cpu_cores_idle_.resize(num_cores_, 0);
    last_cpu_cores_total_.resize(num_cores_, 0);
}

ResourceMonitor::~ResourceMonitor() {
    stopThread();
}

void ResourceMonitor::startThread() {
    if (running_) {
        TBAI_LOG_WARN(logger_, "ResourceMonitor is already running");
        return;
    }

    running_ = true;
    monitor_thread_ = std::make_unique<std::thread>(&ResourceMonitor::monitorLoop, this);
    TBAI_LOG_INFO(logger_, "ResourceMonitor started with frequency: {:.2f} Hz", updateFrequency_);
}

void ResourceMonitor::stopThread() {
    if (!running_) {
        return;
    }

    running_ = false;
    if (monitor_thread_ && monitor_thread_->joinable()) {
        monitor_thread_->join();
    }
    monitor_thread_.reset();
    TBAI_LOG_INFO(logger_, "ResourceMonitor stopped");
}

void ResourceMonitor::monitorLoop() {
    while (running_) {
        ResourceMetrics metrics = collectMetrics();

        {
            std::lock_guard<std::mutex> lock(metrics_mutex_);
            latest_metrics_ = metrics;
        }

        // Log total CPU and memory usage
        TBAI_LOG_INFO_THROTTLE(
            logger_, 1.0 / infoFrequency_, "Resource Usage - Total CPU: {:.1f}%, Memory: {:.1f}% ({:.1f}/{:.1f} MB)",
            metrics.cpu_percent, metrics.memory_percent, metrics.memory_used_mb, metrics.memory_total_mb);

        if (debugFrequency_ != infoFrequency_) {
            TBAI_LOG_DEBUG_THROTTLE(logger_, 1.0 / debugFrequency_,
                                    "Resource Usage - Total CPU: {:.1f}%, Memory: {:.1f}% ({:.1f}/{:.1f} MB)",
                                    metrics.cpu_percent, metrics.memory_percent, metrics.memory_used_mb,
                                    metrics.memory_total_mb);
        }

        // Log per-core CPU usage
        if (perCoreMetrics_ && !metrics.cpu_cores_percent.empty()) {
            std::string core_usage = "CPU Cores: ";
            for (size_t i = 0; i < metrics.cpu_cores_percent.size(); ++i) {
                if (i > 0) core_usage += ", ";
                core_usage += "Core" + std::to_string(i) + ": " +
                              std::to_string(static_cast<int>(metrics.cpu_cores_percent[i])) + "%";
            }
            TBAI_LOG_DEBUG_THROTTLE(logger_, 1.0 / debugFrequency_, core_usage);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(1000.0 / updateFrequency_)));
    }
}

ResourceMetrics ResourceMonitor::collectMetrics() {
    ResourceMetrics metrics;
    metrics.timestamp = std::chrono::steady_clock::now();

    metrics.cpu_percent = getCpuUsage();
    if (perCoreMetrics_) {
        metrics.cpu_cores_percent = getCpuCoresUsage();
    }
    ResourceMetrics memory_metrics = getMemoryUsage();
    metrics.memory_percent = memory_metrics.memory_percent;
    metrics.memory_used_mb = memory_metrics.memory_used_mb;
    metrics.memory_total_mb = memory_metrics.memory_total_mb;

    return metrics;
}

double ResourceMonitor::getCpuUsage() {
    std::ifstream file("/proc/stat");
    std::string line;

    // Read the first line (aggregate "cpu" line)
    if (!std::getline(file, line)) {
        TBAI_LOG_ERROR(logger_, "Failed to read /proc/stat");
        return 0.0;
    }

    std::istringstream iss(line);
    std::string cpu_label;
    long long user = 0, nice = 0, system = 0, idle = 0, iowait = 0, irq = 0, softirq = 0, steal = 0;
    // The /proc/stat cpu line may have more than 8 fields, but we only care about the first 8
    iss >> cpu_label >> user >> nice >> system >> idle >> iowait >> irq >> softirq >> steal;

    long long current_idle = idle + iowait;
    long long current_total = user + nice + system + idle + iowait + irq + softirq + steal;

    auto current_time = std::chrono::steady_clock::now();
    auto time_diff = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_cpu_time_);

    // If this is the first call, initialize and return 0.0
    if (last_cpu_time_.time_since_epoch().count() == 0) {
        last_cpu_idle_ = current_idle;
        last_cpu_total_ = current_total;
        last_cpu_time_ = current_time;
        return 0.0;
    }

    // Only update if at least 100ms have passed
    if (time_diff.count() < 100) {
        return 0.0;
    }

    long long idle_diff = current_idle - last_cpu_idle_;
    long long total_diff = current_total - last_cpu_total_;

    double cpu_percent = 0.0;
    if (total_diff > 0) {
        cpu_percent = 100.0 * (1.0 - static_cast<double>(idle_diff) / static_cast<double>(total_diff));
    }

    last_cpu_idle_ = current_idle;
    last_cpu_total_ = current_total;
    last_cpu_time_ = current_time;

    // Clamp to [0, 100]
    return std::max(0.0, std::min(100.0, cpu_percent));
}

std::vector<double> ResourceMonitor::getCpuCoresUsage() {
    std::vector<double> cpu_cores_percent(num_cores_, 0.0);

    if (num_cores_ == 0) {
        return cpu_cores_percent;
    }

    std::ifstream file("/proc/stat");
    std::string line;

    auto current_time = std::chrono::steady_clock::now();
    auto time_diff = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_cpu_cores_time_);

    // Only update if at least 100ms have passed
    if (time_diff.count() < 100) {
        return cpu_cores_percent;
    }

    // Skip the first line (aggregate "cpu" line)
    std::getline(file, line);

    for (int core = 0; core < num_cores_; ++core) {
        if (!std::getline(file, line)) {
            break;
        }

        std::istringstream iss(line);
        std::string cpu_label;
        long long user = 0, nice = 0, system = 0, idle = 0, iowait = 0, irq = 0, softirq = 0, steal = 0;

        iss >> cpu_label >> user >> nice >> system >> idle >> iowait >> irq >> softirq >> steal;

        long long current_idle = idle + iowait;
        long long current_total = user + nice + system + idle + iowait + irq + softirq + steal;

        // If this is the first call, initialize and return 0.0
        if (last_cpu_cores_time_.time_since_epoch().count() == 0) {
            last_cpu_cores_idle_[core] = current_idle;
            last_cpu_cores_total_[core] = current_total;
            continue;
        }

        long long idle_diff = current_idle - last_cpu_cores_idle_[core];
        long long total_diff = current_total - last_cpu_cores_total_[core];

        double cpu_percent = 0.0;
        if (total_diff > 0) {
            cpu_percent = 100.0 * (1.0 - static_cast<double>(idle_diff) / static_cast<double>(total_diff));
        }

        last_cpu_cores_idle_[core] = current_idle;
        last_cpu_cores_total_[core] = current_total;

        // Clamp to [0, 100]
        cpu_cores_percent[core] = std::max(0.0, std::min(100.0, cpu_percent));
    }

    last_cpu_cores_time_ = current_time;
    return cpu_cores_percent;
}

ResourceMetrics ResourceMonitor::getMemoryUsage() {
    ResourceMetrics metrics;
    std::ifstream file("/proc/meminfo");
    std::string line;

    long long mem_total = 0, mem_free = 0, mem_available = 0, buffers = 0, cached = 0;

    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string key;
        long long value;
        std::string unit;

        iss >> key >> value >> unit;

        if (key == "MemTotal:") {
            mem_total = value;
        } else if (key == "MemFree:") {
            mem_free = value;
        } else if (key == "MemAvailable:") {
            mem_available = value;
        } else if (key == "Buffers:") {
            buffers = value;
        } else if (key == "Cached:") {
            cached = value;
        }
    }

    if (mem_total == 0) {
        TBAI_LOG_ERROR(logger_, "Failed to read memory information from /proc/meminfo");
        return metrics;
    }

    metrics.memory_total_mb = mem_total / 1024.0;

    long long mem_used = mem_total - mem_available;
    if (mem_available == 0) {
        mem_used = mem_total - mem_free - buffers - cached;
    }

    metrics.memory_used_mb = mem_used / 1024.0;
    metrics.memory_percent = 100.0 * static_cast<double>(mem_used) / static_cast<double>(mem_total);

    return metrics;
}

}  // namespace tbai