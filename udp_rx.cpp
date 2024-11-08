#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/time.h> 
#include <time.h>
#include <sys/shm.h>

#define PORT 8080
#define BUFFER_SIZE 1024

struct data {
    int key0;
    int key1;
    int key2;
    int key3;
    int pitch;
    int roll;
    int yaw;
    int thrust;
    int sequence_num;
};

int main() {
    struct sockaddr_in6 server_addr, client_addr; // Use sockaddr_in6 for IPv6
    int sockfd, nbytes;
    socklen_t addr_len;
    char buffer[BUFFER_SIZE];

    // Create UDP socket with AF_INET6 for IPv6
    if ((sockfd = socket(AF_INET6, SOCK_DGRAM, 0)) < 0) {
        perror("socket");
        exit(EXIT_FAILURE);
    }

    // Bind to port with IPv6 address settings
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin6_family = AF_INET6; // AF_INET6 for IPv6
    server_addr.sin6_addr = in6addr_any; // IN6ADDR_ANY_INIT to accept any incoming messages
    server_addr.sin6_port = htons(PORT);
    if (bind(sockfd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        perror("bind");
        exit(EXIT_FAILURE);
    }

    printf("Listening for UDP datagrams on port %d...\n", PORT);

    // Shared memory init (unchanged)
    int segment_id;
    struct data* shared_memory;
    struct shmid_ds shmbuffer;
    int segment_size;
    const int shared_segment_size = 0x6400;
    int smhkey = 33222;

    // Allocate a shared memory segment
    segment_id = shmget(smhkey, shared_segment_size, IPC_CREAT | 0666);
    // Attach the shared memory segment
    shared_memory = (struct data*) shmat(segment_id, 0, 0);
    printf("Shared memory attached at address %p\n", shared_memory);
    // Determine the segment's size
    shmctl(segment_id, IPC_STAT, &shmbuffer);
    segment_size = shmbuffer.shm_segsz;
    printf("Segment size: %d\n", segment_size);

    // Receive data (adapted for IPv6)
    addr_len = sizeof(client_addr);
    while (1) {
        nbytes = recvfrom(sockfd, (char *)buffer, BUFFER_SIZE, 0, (struct sockaddr *)&client_addr, &addr_len);
        if (nbytes < 0) {
            perror("recvfrom");
            exit(EXIT_FAILURE);
        }
        buffer[nbytes] = '\0';

        // Update shared memory structure with data
        shared_memory->key0 = buffer[4];
        shared_memory->key1 = buffer[5];
        shared_memory->key2 = buffer[6];
        shared_memory->key3 = buffer[7];
        shared_memory->thrust = buffer[1];
        shared_memory->yaw = buffer[0];
        shared_memory->pitch = buffer[3];
        shared_memory->roll = buffer[2];
        shared_memory->sequence_num = buffer[8];

        char client_addr_str[INET6_ADDRSTRLEN];
        inet_ntop(AF_INET6, &(client_addr.sin6_addr), client_addr_str, sizeof(client_addr_str));
        printf("Received %d bytes from %s:%d\n", nbytes, client_addr_str, ntohs(client_addr.sin6_port));
        printf("byte 0 is %d\n",buffer[0]);        
        printf("byte 1 is %d\n",buffer[1]);        
        printf("byte 2 is %d\n",buffer[2]);        
        printf("byte 3 is %d\n",buffer[3]);        
        printf("byte 4 is %d\n",buffer[4]);          
        printf("byte 5 is %d\n",buffer[5]);          
        printf("byte 6 is %d\n",buffer[6]);  
        printf("byte 7 is %d\n",buffer[7]);
        printf("byte 8 is %d\n",buffer[8]);
    }

    // Close socket (commented out)
    // close(sockfd);

    return 0;
}
