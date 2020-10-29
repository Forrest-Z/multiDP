/*
***********************************************************************
* tcpclient.h:
* This header file can be read by C++ compilers
*
*  by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#include <arpa/inet.h>
#include <errno.h>
#include <netdb.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <chrono>
#include <thread>

class tcpclient {
 public:
  tcpclient(const std::string &_ip, const std::string &_port)
      : sockfd(0), results(0), ip_server(_ip), port(_port) {
    connect2server();
  }
  ~tcpclient() { close(sockfd); }

  void senddata(char *recv_buffer, const char *send_buffer, int recv_size,
                int send_size) {
    int send_bytes = send(sockfd, send_buffer, send_size, 0);
    if (send_bytes == -1) {
      perror("send");
      results = 1;
    }
    int recv_bytes = recv(sockfd, recv_buffer, recv_size, 0);
    if (recv_bytes == -1) {
      perror("recv");
      results = 1;
    }
  }

  void reconnect() {
    while (1) {
      connect2server();
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
  }

  int getsocketresults() const noexcept { return results; }

 private:
  int sockfd;
  int results;
  std::string ip_server;
  std::string port;  // the port client will be connecting to
  // get sockaddr, IPv4 or IPv6:
  void *get_in_addr(struct sockaddr *sa) {
    if (sa->sa_family == AF_INET) {
      return &(((struct sockaddr_in *)sa)->sin_addr);
    }

    return &(((struct sockaddr_in6 *)sa)->sin6_addr);
  }

  void connect2server() {
    struct addrinfo hints, *servinfo, *p;
    char s[INET6_ADDRSTRLEN];
    memset(&hints, 0, sizeof hints);  //将参数全设为0,需要的再修改
    hints.ai_family = AF_UNSPEC;      //协议无关,还可以指定IPV4或ipv6
    hints.ai_socktype = SOCK_STREAM;  // tcp字节流,还可以数据报
    int rv = getaddrinfo(ip_server.c_str(), port.c_str(), &hints, &servinfo);
    if (rv != 0) {  // getaddrinfo 出错,输出错误信息
      fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(rv));
      results = 1;
    }
    // loop through all the results and connect to the first we can
    // 循环addrinfo返回的链表
    for (p = servinfo; p != NULL; p = p->ai_next) {
      sockfd = socket(p->ai_family, p->ai_socktype, p->ai_protocol);
      if (sockfd == -1) {  // socket描述符错误
        perror("client: socket");
        continue;
      }

      if (connect(sockfd, p->ai_addr, p->ai_addrlen) == -1) {
        perror(
            "client: connect");  // socket返回的描述符部分打开,还不能用,要connect
        close(sockfd);
        continue;
      }

      break;
    }

    if (p == NULL) {
      fprintf(stderr, "client: failed to connect\n");
      results = 2;
    }

    inet_ntop(p->ai_family, get_in_addr((struct sockaddr *)p->ai_addr), s,
              sizeof s);  //二进制转点分十进制,用getnameinfo更好吧
    printf("client: connecting to %s\n", s);
    freeaddrinfo(servinfo);  // all done with this structure,释放链表结构
  }
};
