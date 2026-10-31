#define _DEFAULT_SOURCE
#define _POSIX_C_SOURCE 200809L

#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#define GPIO_NUM 401
#define GPIO_DIR "/sys/class/gpio/gpio401"
#define GPIO_DIRECTION GPIO_DIR "/direction"
#define GPIO_VALUE GPIO_DIR "/value"

static int direction_fd = -1;
static int value_fd = -1;
static int exported_by_program = 0;
static volatile sig_atomic_t running = 1;

static uint64_t now_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}

static void delay_us(uint64_t us)
{
    uint64_t end = now_us() + us;
    while (now_us() < end) {
        
    }
}

static int write_text_file(const char *path, const char *text)
{
    int fd = open(path, O_WRONLY);
    ssize_t n;

    if (fd < 0)
        return -1;
    n = write(fd, text, strlen(text));
    close(fd);
    return n < 0 ? -1 : 0;
}

static int export_gpio(void)
{
    struct stat st;
    char number[16];
    int i;

    if (stat(GPIO_DIR, &st) == 0)
        return 0;

    snprintf(number, sizeof(number), "%d", GPIO_NUM);
    if (write_text_file("/sys/class/gpio/export", number) < 0) {
        if (errno != EBUSY) {
            perror("Failed to export GPIO401");
            return -1;
        }
    } else {
        exported_by_program = 1;
    }

    for (i = 0; i < 100; ++i) {
        if (stat(GPIO_DIR, &st) == 0)
            return 0;
        usleep(5000);
    }

    fprintf(stderr, "Timeout occurred while waiting for %s to appear.\n", GPIO_DIR);
    return -1;
}

static int gpio_set_direction(const char *direction)
{
    if (lseek(direction_fd, 0, SEEK_SET) < 0)
        return -1;
    return write(direction_fd, direction, strlen(direction)) < 0 ? -1 : 0;
}

static int gpio_write(int value)
{
    char c = value ? '1' : '0';

    if (lseek(value_fd, 0, SEEK_SET) < 0)
        return -1;
    return write(value_fd, &c, 1) == 1 ? 0 : -1;
}

static int gpio_read(void)
{
    char c;

    if (lseek(value_fd, 0, SEEK_SET) < 0)
        return -1;
    if (read(value_fd, &c, 1) != 1)
        return -1;
    return c == '0' ? 0 : 1;
}

static int wait_for_level(int target, uint64_t timeout_us)
{
    uint64_t start = now_us();
    int value;

    do {
        value = gpio_read();
        if (value < 0)
            return -2;
        if (value == target)
            return 0;
    } while (now_us() - start < timeout_us);

    return -1;
}

static int measure_level(int level, uint64_t timeout_us, uint64_t *duration)
{
    uint64_t start = now_us();
    int value;

    do {
        value = gpio_read();
        if (value < 0)
            return -2;
        if (value != level) {
            *duration = now_us() - start;
            return 0;
        }
    } while (now_us() - start < timeout_us);

    return -1;
}


static int read_dht11(uint8_t data[5], char *error, size_t error_size)
{
    uint64_t duration;
    int i;
    int rc;

    memset(data, 0, 5);

   
    if (gpio_set_direction("out") < 0 || gpio_write(1) < 0) {
        snprintf(error, error_size, "Cannot set GPIO as output");
        return -2;
    }
    usleep(50000);
    if (gpio_write(0) < 0) {
        snprintf(error, error_size, "Unable to lower DATA");
        return -2;
    }
    usleep(20000);
    if (gpio_write(1) < 0) {
        snprintf(error, error_size, "Unable to increase DATA");
        return -2;
    }
    delay_us(30);
    if (gpio_set_direction("in") < 0) {
        snprintf(error, error_size, "Cannot set GPIO as input");
        return -2;
    }

  
    rc = wait_for_level(0, 200);
    if (rc != 0) {
        snprintf(error, error_size, "Waiting for the DHT11 response to go low level before timing out");
        return rc == -2 ? -2 : -1;
    }
    rc = measure_level(0, 200, &duration);
    if (rc != 0) {
        snprintf(error, error_size, "DHT11 response does not end with low level signal");
        return rc == -2 ? -2 : -1;
    }
    rc = measure_level(1, 200, &duration);
    if (rc != 0) {
        snprintf(error, error_size, "DHT11 response does not end with a high level");
        return rc == -2 ? -2 : -1;
    }

    for (i = 0; i < 40; ++i) {
        rc = measure_level(0, 150, &duration);
        if (rc != 0) {
            snprintf(error, error_size, "The %dth position is waiting for the low level to end before timing out.", i + 1);
            return rc == -2 ? -2 : -1;
        }

        rc = measure_level(1, 150, &duration);
        if (rc == -2) {
            snprintf(error, error_size, "The %dth read operation of GPIO failed.", i + 1);
            return -2;
        }
        if (rc == -1) {
            if (i == 39) {
               
                duration = 150;
            } else {
                snprintf(error, error_size,
                         "The %dth position is waiting for the high level to end before timing out.", i + 1);
                return -1;
            }
        }

        data[i / 8] <<= 1;
        if (duration > 50)
            data[i / 8] |= 1;
    }

    if (((data[0] + data[1] + data[2] + data[3]) & 0xff) != data[4]) {
        snprintf(error, error_size,
                 "Checksum error, original data：%02X %02X %02X %02X %02X",
                 data[0], data[1], data[2], data[3], data[4]);
        return -3;
    }

    return 0;
}

static void cleanup(void)
{
    if (direction_fd >= 0)
        gpio_set_direction("in");
    if (value_fd >= 0)
        close(value_fd);
    if (direction_fd >= 0)
        close(direction_fd);

    value_fd = -1;
    direction_fd = -1;

    if (exported_by_program)
        write_text_file("/sys/class/gpio/unexport", "401");
}

static void stop_handler(int signal_number)
{
    (void)signal_number;
    running = 0;
}

int main(int argc, char *argv[])
{
    uint8_t data[5];
    char error[160];
    unsigned int success = 0;
    unsigned int failure = 0;
    int server_mode = (
        argc >= 2
        && strcmp(argv[1], "--server") == 0
    );

    if (geteuid() != 0) {
        fprintf(stderr, "Please run this program using sudo.\n");
        return 1;
    }

    signal(SIGINT, stop_handler);
    signal(SIGTERM, stop_handler);

    if (export_gpio() < 0)
        return 1;

    direction_fd = open(GPIO_DIRECTION, O_RDWR);
    value_fd = open(GPIO_VALUE, O_RDWR);
    if (direction_fd < 0 || value_fd < 0) {
        perror("Failed to open the GPIO401 file");
        cleanup();
        return 1;
    }

    if (server_mode) {
        char command[128];

      
        setvbuf(stdout, NULL, _IOLBF, 0);

        fprintf(
            stderr,
            "DHT11 service has been started: Physical pin 37 (GPIO 401)\n"
        );
        fflush(stderr);

        while (
            running
            && fgets(command, sizeof(command), stdin) != NULL
        ) {
            char point_id[64];

            if (strncmp(command, "QUIT", 4) == 0)
                break;

            if (
                sscanf(
                    command,
                    "READ %63s",
                    point_id
                ) != 1
            ) {
                continue;
            }

            int rc = read_dht11(
                data,
                error,
                sizeof(error)
            );

            if (rc == 0) {
                printf(
                    "DHT\t%s\tOK\t%u.%u\t%u.%u\n",
                    point_id,
                    data[0],
                    data[1],
                    data[2],
                    data[3]
                );
            } else {
                printf(
                    "DHT\t%s\tERROR\t%s\n",
                    point_id,
                    error
                );
            }
        }

        cleanup();
        fprintf(stderr, "DHT11 service has been stopped\n");
        return 0;
    }

    printf("RDK X5 DHT11 C language test\n");
    printf("DATA: Physical pin 37 (GPIO401)\n");
    printf("VCC: Physical pin 1 (3.3V); GND: Physical pin 6\n");
    printf("Press Ctrl+C to stop\n");
    printf("--------------------------------------------------\n");

    while (running) {
        int rc = read_dht11(data, error, sizeof(error));

        if (rc == 0) {
            ++success;
            printf("✓ humidity：%u.%u %%RH，temperature：%u.%u °C",
                   data[0], data[1], data[2], data[3]);
            printf("，Success/Failure：%u/%u\n", success, failure);
        } else {
            ++failure;
            printf("✗ Read failure：%s，Success/Failure：%u/%u\n",
                   error, success, failure);
        }
        fflush(stdout);

       
        for (int i = 0; i < 20 && running; ++i)
            usleep(100000);
    }

    cleanup();
    printf("\nThe test is over and the GPIO has been released.\n");
    return 0;
}
