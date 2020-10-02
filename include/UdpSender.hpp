#ifndef __UDP_SENDER__H
#define __UDP_SENDER__H

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <opencv2/core.hpp>


// struct bbox_packet {
// 	float min_distance;
// 	int32_t x_min;
// 	int32_t x_max;
// 	int32_t y_min;
// 	int32_t y_max;
// };

struct bbox_packet {
	float xc;
};

class UdpSender {
private:
	struct sockaddr_in image_server;
	int image_sockfd;

public:
	UdpSender(const char*, const char*, uint16_t);
	void _sendto(const char* host, uint16_t port, const void* buf, size_t len);
	void sendImage(cv::Mat);
};




#endif // __UDP_SENDER__H
