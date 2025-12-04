#include <iostream>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>

int main() {
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        std::cerr << "Erreur à la création du socket\n";
        return 1;
    }

    sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_port = htons(6969);       // même port que l’émetteur
    addr.sin_addr.s_addr = INADDR_ANY; // écoute toutes les interfaces

    if (bind(sockfd, (sockaddr*)&addr, sizeof(addr)) < 0) {
        std::cerr << "Erreur au bind\n";
        close(sockfd);
        return 1;
    }

    std::cout << "Serveur UDP en écoute sur le port 6969...\n";

    while (true) {
        float buffer[20]; // exemple : 20 floats max par paquet
        sockaddr_in sender_addr;
        socklen_t sender_len = sizeof(sender_addr);

        ssize_t n = recvfrom(sockfd, buffer, sizeof(buffer), 0,
                             (sockaddr*)&sender_addr, &sender_len);

        if (n > 0) {
            size_t n_floats = n / sizeof(float);
            std::cout << "Paquet reçu : ";
            for (size_t i = 0; i < n_floats; ++i) {
                std::cout << buffer[i] << " ";
            }
            std::cout << "\n";
        }
    }

    close(sockfd);
    return 0;
}