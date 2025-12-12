#include <unistd.h>

#include <iostream>
#include <string>

#include <unitree/idl/ros2/PointCloud2_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

using namespace unitree::common;
using namespace unitree::robot;

#define TOPIC_RAW "rt/utlidar/cloud"

void dds_msg_callback(const void *msg2) {
    std::cout << "hello, point cloud published" << std::endl;
    auto *msg = reinterpret_cast<const sensor_msgs::msg::dds_::PointCloud2_ *>(msg2);
    // Print Header information
    std::cout << "Header:" << std::endl;
    std::cout << "  stamp: " << msg->header().stamp().sec() << "." << msg->header().stamp().nanosec() << std::endl;
    std::cout << "  frame_id: " << msg->header().frame_id() << std::endl;

    // Print 2D structure
    std::cout << "height: " << msg->height() << std::endl;
    std::cout << "width: " << msg->width() << std::endl;

    // Print fields info
    std::cout << "fields (" << msg->fields().size() << "):" << std::endl;
    for (size_t i = 0; i < msg->fields().size(); ++i) {
        const auto &field = msg->fields()[i];
        std::cout << "  [" << i << "] name: " << field.name() << ", offset: " << field.offset()
                  << ", datatype: " << static_cast<int>(field.datatype()) << ", count: " << field.count() << std::endl;
    }

    // Print endianness, point_step, row_step, data size
    std::cout << "is_bigendian: " << (msg->is_bigendian() ? "true" : "false") << std::endl;
    std::cout << "point_step: " << msg->point_step() << std::endl;
    std::cout << "row_step: " << msg->row_step() << std::endl;
    std::cout << "data size: " << msg->data().size() << std::endl;

    // Print is_dense
    std::cout << "is_dense: " << (msg->is_dense() ? "true" : "false") << std::endl;
}

int main(int argc, char **argv) {
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <network_interface> [<domain_id>]" << std::endl;
        std::cout << "Please specify the network interface" << std::endl;
        return 1;
    }

    std::string interface = argv[1];
    int domain = 0;
    if (argc > 2) {
        domain = std::atoi(argv[2]);
    }

    ChannelFactory::Instance()->Init(domain, interface);

    ChannelSubscriberPtr<sensor_msgs::msg::dds_::PointCloud2_> sub;
    sub.reset(new ChannelSubscriber<sensor_msgs::msg::dds_::PointCloud2_>(TOPIC_RAW));
    sub->InitChannel(std::bind(&dds_msg_callback, std::placeholders::_1), 1);

    while (true) {
        sleep(1);  // Keep the program running to receive messages
    }

    return 0;
}