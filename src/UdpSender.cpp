
#include <stdio.h>
#include <strings.h>  // for bzero()

#include "UdpSender.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>


UdpSender::UdpSender(const char* ServerAddress, const char* StreamAddress, uint16_t imageRxPort) {
	bzero((char*)&image_server, sizeof(image_server));
	image_server.sin_family = AF_INET;
	image_server.sin_addr.s_addr = inet_addr(StreamAddress);   //inet_addr(ServerAddress);
	image_server.sin_port = htons(imageRxPort);

	image_sockfd = socket(AF_INET, SOCK_DGRAM, 0);
	if (image_sockfd < 0) {
		fprintf(stderr, "Error opening socket");
		//return EXIT_FAILURE;
	}
}




void UdpSender::_sendto(const char* host, uint16_t port, const void* buf, size_t len) {
	struct sockaddr_in addr;
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = inet_addr(host);
	addr.sin_port = htons(port);

	sendto(image_sockfd, buf, len, 0,
	       (const struct sockaddr*)&addr, sizeof(addr));
}


void UdpSender::sendImage(cv::Mat image) {
	#define START_MAGIC "__HylPnaJY_START_JPG %09d\n"
	#define STOP_MAGIC "_g1nC_EOF"
	#define STOP_MAGIC_LEN 9

	std::vector<uchar> outputBuffer;
	std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 42};

	if (!cv::imencode("*.jpg", image, outputBuffer, params)) {
		printf("failed to imencode\n");
		return;
	}

	int bufLen = outputBuffer.size();

	int filepos = 0;
	int numbytes = 0;

	char startString[32];
	int startStringLen;

	startStringLen = sprintf(startString, START_MAGIC, bufLen);

	// Send the jpg image out via UDP:
	sendto(image_sockfd, startString, startStringLen, 0,
	       (const struct sockaddr*)&image_server, sizeof(image_server));

	while (filepos < bufLen) {
		if (bufLen - filepos < 1400) {
			numbytes = bufLen - filepos;
		} else {
			numbytes = 1400;  // Ethernet MTU is 1500
			// (... but the max *safe* UDP payload is 508 bytes...)
		}

		sendto(image_sockfd, &outputBuffer[filepos], numbytes, 0,
		       (const struct sockaddr*)&image_server, sizeof(image_server));

		filepos += numbytes;
	}

	sendto(image_sockfd, STOP_MAGIC, STOP_MAGIC_LEN, 0,
	       (const struct sockaddr*)&image_server, sizeof(image_server));
}



