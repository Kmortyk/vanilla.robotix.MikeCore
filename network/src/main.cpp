//
// Created by kmortyk on 19.01.2020.
//

#include "network.h"

int main() {

    UdpNetwork udp_net(1031, 1031);

    while(true) {
        std::cout << udp_net.recv() << std::endl;
    }
}
