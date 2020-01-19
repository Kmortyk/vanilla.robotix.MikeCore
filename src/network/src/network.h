//
// Created by kmortyk on 19.01.2020.
//

#ifndef NETWORK_NETWORK_H
#define NETWORK_NETWORK_H

#include <zconf.h>
#include <sys/socket.h>
#include <iostream>
#include <netinet/in.h>
#include <string.h>
#include "err_exit.h"

class UdpNetwork {

    struct sockaddr_in client_inf;
    char read_buf[BUFSIZ];
    socklen_t socklen;
    int sock_d; // socket descriptor

public:

    UdpNetwork(int srv_port, int clt_port) {
        socklen = sizeof(struct sockaddr_in);

        struct sockaddr_in addr_inf;
        addr_inf.sin_family = AF_INET;
        addr_inf.sin_port = htons(srv_port);
        addr_inf.sin_addr.s_addr = htonl(INADDR_ANY);
        // create
        sock_d = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock_d == -1)
            err_exit("Couldn't create socket.");
        // bind
        int b_res = bind(sock_d, (const struct sockaddr *) &addr_inf, sizeof(addr_inf));
        if (b_res == -1)
            err_exit("bind failed");
    }

    char* recv() {
        printf("Try to receive.\n");
        ssize_t bytes_read = recvfrom(sock_d, read_buf, BUFSIZ, 0, (struct sockaddr *) &client_inf, &socklen);
        read_buf[bytes_read] = '\0';
        printf("Broadcast received message: '%s'\n", read_buf);
        return read_buf;
    }

    char* read(int conn_d) {
        bzero(read_buf, BUFSIZ);
        ::read(conn_d, read_buf, BUFSIZ);
        printf("Received message: '%s'\n", read_buf);
        return read_buf;
    }

    int accept() {
        // Accept the data packet from client and verification
        socklen_t client_len = sizeof(client_inf);
        int conn_d = ::accept(sock_d, (struct sockaddr *) &client_inf, &client_len);
        if (conn_d < 0)
            err_exit("tcp accept failed");

        // printf("Connection accepted.\n");

        return conn_d;
    }
};

#endif //NETWORK_NETWORK_H
