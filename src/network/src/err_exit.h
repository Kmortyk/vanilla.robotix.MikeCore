//
// Created by kmortyk on 19.01.2020.
//

#ifndef NETWORK_ERR_EXIT_H
#define NETWORK_ERR_EXIT_H

void err_exit(const char *message) {
    char buf[BUFSIZ];
    snprintf(buf, BUFSIZ, "\nError [%i] %s", errno, message);
    perror(buf);
    exit(EXIT_FAILURE);
}

#endif //NETWORK_ERR_EXIT_H
