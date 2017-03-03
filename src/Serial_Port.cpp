#include "serial_port.h"

Serial_Port::
Serial_Port()
{
    initialize_defaults();
}

Serial_Port::
Serial_Port(const char *usb_name_, int baudrate_)
{
    initialize_defaults();
    usb_name = usb_name_;
    baudrate = baudrate_;
}

Serial_Port::
~Serial_Port()
{
    pthread_mutex_destroy(&lock);
}

void Serial_Port::
initialize_defaults()
{
    debug = false;
    fd = -1;
    status = SERIAL_PORT_CLOSED;

    usb_name = (char*) "/dev/ttyUSB0";      // "/dev/ttyS0"
    baudrate = 57600;

    // Start Mutex
    int result = pthread_mutex_init(&lock, NULL);
    if (result != 0) {
        printf("Error: mutex init failed!");
        throw 1;
    }
}

int Serial_Port::
read_message(mavlink_message_t &message)
{
    uint8_t cp;
    mavlink_status_t status;
    uint8_t msgReceived = false;

    // read from port
    int result = _read_port(cp);
    if (result > 0) {
        msgReceived = mavlink_parse_char(MAVLINK_COMM_1, cp, &message, &status);

        if ((lastStatus.packet_rx_drop_count != status.packet_rx_drop_count) && debug) {
            printf("Error: dropped %d packets\n", status.packet_rx_drop_count);
            unsigned char v = cp;
            fprintf(stderr, "%02x", v);
        }
        lastStatus = status;
    } else {
        fprintf(stderr, "Error: Couldn't read from fd %d\n", fd);
    }

    return msgReceived;
}

int Serial_Port::
write_message(const mavlink_message_t &message)
{
    char bug[300];
    unsigned len = mavlink_msg_to_send_buffer((uint8_t*) buf, &message);
    int bytesWritten = _write_port(buf, len);
    return bytesWritten;
}

void Serial_Port::
open_serial()
{
    printf("open port\n");
    fd = _open_port(usb_name);
    if (fd == -1) {
        printf("Failure, couldn't open port.\n");
        throw EXIT_FAILURE;
    }

    bool success = _setup_port(baudrate, 8, 1, false, false);

    if (!success) {
        printf("Failure: couldn't configure port.\n");
        throw EXIT_FAILURE;
    }
    if (fd <= 0) {
        printf("Failure: connection to port %s with %d baudrate.\n", usb_name, baudrate);
        throw EXIT_FAILURE;
    }

    printf("Connected to %s with %d baudrate, 8data bits, no parity, 1stop bit\n", usb_name, baudrate);
    lastStatus.packet_rx_drop_count = 0;

    status = SERIAL_PORT_OPEN;
    return;
}

void Serial_Port::
close_serial()
{
    printf("CLOSE PORT\n");
    int result = close(fd);
    if (result) {
        fprintf(stderr, "WARNING: Error on port close %i\n", result);
    }
    status = SERIAL_PORT_CLOSED;
}

void Serial_Port::
start()
{
    open_serial();
}

void Serial_Port::
stop()
{
    close_serial();
}

void Serial_Port::
handle_quit(int sig)
{
    try {
        stop();
    }
    catch (int error) {
        fprintf(stderr, "WARNING: could not stop serial port\n");
    }
}

int Serial_Port::
_open_port(const char* port)
{
    fd = open(port, 0_RDWR | 0_NOCTTY | 0_NDELAY);
    if (fd == -1) {
        return -1;
    } else {
        fcntl(fd, F_SETFL, 0);
    }

    return fd;
}

bool Serial_Port::
_setup_port(int baud, int data_bits, int stop_bits, bool parity, bool hardware_control)
{
    if (!isatty(fd)) {
        fprintf(stderr, "Error: file descriptor %d is NOT a serial port\n", fd);
        return false;
    }

    struct termios config;
    if (tcgetattr(fd, &config) < 0) {
        fprintf(stderr, "ERROR: could not read configuration of fd %d\n", fd);
        return false;
    }

    // Input flags - Turn off input processing
    // convert break to null byte, no CR to NL translation,
    // no NL to CR translation, don't mark parity errors or breaks
    // no input parity check, don't strip high bit off,
    // no XON/XOFF software flow control
    config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
                        INLCR | PARMRK | INPCK | ISTRIP | IXON);

    // Output flags - Turn off output processing
    // no CR to NL translation, no NL to CR-NL translation,
    // no NL to CR translation, no column 0 CR suppression,
    // no Ctrl-D suppression, no fill characters, no case mapping,
    // no local output processing
    config.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
                         ONOCR | OFILL | OPOST);

    #ifdef OLCUC
        config.c_oflag &= ~OLCUC;
    #endif

    #ifdef ONOEOT
        config.c_oflag &= ~ONOEOT;
    #endif

    // No line processing:
    // echo off, echo newline off, canonical mode off,
    // extended input processing off, signal chars off
    config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

    // Turn off character processing
    // clear current char size mask, no parity checking,
    // no output processing, force 8 bit input
    config.c_cflag &= ~(CSIZE | PARENB);
    config.c_cflag |= CS8;

    // One input byte is enough to return from read()
    // Inter-character timer off
    config.c_cc[VMIN]  = 1;
    config.c_cc[VTIME] = 10; // was 0

    // Get the current options for the port
    ////struct termios options;
    ////tcgetattr(fd, &options);

    // Apply baudrate
    switch (baud)
    {
        case 1200:
            if (cfsetispeed(&config, B1200) < 0 || cfsetospeed(&config, B1200) < 0)
            {
                fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
                return false;
            }
            break;
        case 1800:
            cfsetispeed(&config, B1800);
            cfsetospeed(&config, B1800);
            break;
        case 9600:
            cfsetispeed(&config, B9600);
            cfsetospeed(&config, B9600);
            break;
        case 19200:
            cfsetispeed(&config, B19200);
            cfsetospeed(&config, B19200);
            break;
        case 38400:
            if (cfsetispeed(&config, B38400) < 0 || cfsetospeed(&config, B38400) < 0)
            {
                fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
                return false;
            }
            break;
        case 57600:
            if (cfsetispeed(&config, B57600) < 0 || cfsetospeed(&config, B57600) < 0)
            {
                fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
                return false;
            }
            break;
        case 115200:
            if (cfsetispeed(&config, B115200) < 0 || cfsetospeed(&config, B115200) < 0)
            {
                fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
                return false;
            }
            break;

        // These two non-standard (by the 70'ties ) rates are fully supported on
        // current Debian and Mac OS versions (tested since 2010).
        case 460800:
            if (cfsetispeed(&config, B460800) < 0 || cfsetospeed(&config, B460800) < 0)
            {
                fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
                return false;
            }
            break;
        case 921600:
            if (cfsetispeed(&config, B921600) < 0 || cfsetospeed(&config, B921600) < 0)
            {
                fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
                return false;
            }
            break;
        default:
            fprintf(stderr, "ERROR: Desired baud rate %d could not be set, aborting.\n", baud);
            return false;

            break;
    }

    // Finally, apply the configuration
    if(tcsetattr(fd, TCSAFLUSH, &config) < 0)
    {
        fprintf(stderr, "\nERROR: could not set configuration of fd %d\n", fd);
        return false;
    }

    return true;
}

int Serial_Port::
_read_port(uint8_t &cp)
{
    pthread_mutex_lock(&lock);
    int result = read(fd, &cp, 1);
    pthread_mutex_unlock(&lock);

    return result;
}

int Serial_Port::
_write_port(char *buf, unsigned len)
{
    pthread_mutex_lock(&lock);
    const int bytesWritten = static_cast<int>(write(fd, buf, len));
    tcdrain(fd);
    pthread_mutex_unlock(&lock);
    return bytesWritten;
}