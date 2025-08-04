#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>

int main()
{
    int fd;
    char buffer[128];

    while (1)
    {
        fd = open("/dev/dht11", O_RDONLY);
        if (fd < 0)
        {
            perror("open");
            sleep(1);
            continue;
        }

        ssize_t len = read(fd, buffer, sizeof(buffer) - 1);
        if (len < 0)
        {
            perror("read");
            close(fd);
            sleep(1);
            continue;
        }

        buffer[len] = '\0';
        printf("%s\n", buffer);
        close(fd);
        sleep(2); // wait before next read
    }

    return 0;
}