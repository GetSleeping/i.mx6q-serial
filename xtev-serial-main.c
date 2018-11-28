#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <asm-generic/ioctls.h>
#include <pthread.h>

#include "xtev-serial-debug.h"
#include "xtev-serial-parse.h"


#define serial_debug(fmt, ...) \
	xtev_debug("xtev-serial-main.c", fmt, ##__VA_ARGS__)
#define serial_info(fmt, ...) \
	xtev_info("xtev-serial-main.c", fmt, ##__VA_ARGS__)
#define serial_warning(fmt, ...) \
	xtev_warning("xtev-serial-main.c", fmt, ##__VA_ARGS__)
#define serial_error(fmt, ...) \
	xtev_error("xtev-serial-main.c", fmt, ##__VA_ARGS__)

#define UART_SPEED              115200U
#define BUFFER_LENGTH           255

struct _XTEV_SERIAL_ {
	pthread_t threadID;					/*  */
	int8_t threadStatus;				/*  */
	char bufferAddr[BUFFER_LENGTH];		/*  */
	int32_t fd;							/*  */
} ;

static struct _XTEV_SERIAL_ g_structReadStruct;
static struct _XTEV_SERIAL_ g_structWriteStruct;
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t cond = PTHREAD_COND_INITIALIZER;


int consoleLength = 0;
/**
 * @func:uart_speed 
 * @desc:
 * @para:
 */
static int 
uart_speed(uint32_t speed) {
	switch (speed) {
	case 9600:
		return B9600;
	case 19200:
		return B19200;
	case 38400:
		return B38400;
	case 57600:
		return B57600;
	case 115200:
		return B115200;
	case 230400:
		return B230400;
	case 460800:
		return B460800;
	case 500000:
		return B500000;
	case 576000:
		return B576000;
	case 921600:
		return B921600;
	case 1000000:
		return B1000000;
	case 1152000:
		return B1152000;
	case 1500000:
		return B1500000;
	case 2000000:
		return B2000000;
	case 2500000:
		return B2500000;
	case 3000000:
		return B3000000;
	case 3500000:
		return B3500000;
	case 4000000:
		return B4000000;
	default:
		return B0;
	}
}

/**
 * @func:set_uart_speed 
 * @desc:
 * @para:
 */
static int
set_uart_speed(int32_t fd, struct termios* terminalInfo, uint32_t uartSpeed) {
	cfsetospeed(terminalInfo, uart_speed(uartSpeed));
	cfsetispeed(terminalInfo, uart_speed(uartSpeed));
	if (tcsetattr(fd, TCSANOW, terminalInfo) < 0) {
		serial_error("Set uart speed %d failed", uartSpeed);
		return -1;
	}
	serial_info("Uart speed set to %d\n", uartSpeed);
	return 0;
}


/**
 * @func:init_uart 
 * @desc:uart init
 * @para:
 */
static int
init_uart(char* dev, struct termios* terminalInfo) {
    int fd, i;
    int pins;

    fd = open(dev, O_RDWR | O_NOCTTY);
    if(fd < 0) {
        serial_error("Open %s device failed",dev);
        return -1;
    }

    tcflush(fd, TCIOFLUSH);
    
    ioctl(fd, TIOCMGET, &pins);
    pins &= ~TIOCM_LOOP;

    ioctl(fd, TIOCMSET, &pins);

    if(tcgetattr(fd, terminalInfo) < 0) {
        serial_error("Get serial port settings failed");
		return -1;
    }

    cfmakeraw(terminalInfo);

    serial_info("Open %s device successed", dev);
    return fd;
}

/**
 * @func:pthread_uart_write
 * @desc:
 * @para:
 */
void 
pthread_uart_write(void* arg) {
	int32_t currentThreadStatus = 0;
	struct _XTEV_SERIAL_* ptrXtevSerial = (struct _XTEV_SERIAL_ *)arg;

	while(!currentThreadStatus) {
		pthread_mutex_lock(&mutex);
		pthread_cond_wait(&cond, &mutex);
		serial_debug("Current thread id: %d", (int)pthread_self());
		serial_debug("Current process id: %d", (int)getpid());
		write(ptrXtevSerial->fd, ptrXtevSerial->bufferAddr, consoleLength);
		currentThreadStatus = ptrXtevSerial->threadStatus;
		pthread_mutex_unlock(&mutex);
	}
}

/**
 * @func:pthread_uart_read
 * @desc:
 * @para:
 */
void 
pthread_uart_read(void* arg) {
	static int32_t currentThreadStatus = 0;
	struct _XTEV_SERIAL_* ptrXtevSerial = (struct _XTEV_SERIAL_ *)arg;

	while(!currentThreadStatus) {
		memset(ptrXtevSerial->bufferAddr, 0, BUFFER_LENGTH);
		read(ptrXtevSerial->fd, ptrXtevSerial->bufferAddr, BUFFER_LENGTH);
		serial_debug("Current thread id: %d", (int)pthread_self());
		serial_debug("Current process id: %d", (int)getpid());
		cmd_parse(ptrXtevSerial->bufferAddr);
		currentThreadStatus = ptrXtevSerial->threadStatus;
	}
}
/**
 * @func:threads_destroy
 * @desc:
 * @para:
 */
static void 
threads_destroy(struct _XTEV_SERIAL_* g_ptrReadStruct, \
				struct _XTEV_SERIAL_* g_ptrWriteStruct) {
	int32_t ret;
	
	safely_kill_tracker();

	pthread_mutex_lock(&mutex);
	g_ptrWriteStruct->threadStatus = 1;
	pthread_mutex_unlock(&mutex);
	pthread_cond_signal(&cond);
	if(!(ret = pthread_join(g_ptrWriteStruct->threadID, NULL))) {
		serial_info("Write thread exit successed");
	} else {
		serial_error("Write thread exit error with %d", ret);
	}

	if(!(ret = pthread_detach(g_ptrReadStruct->threadID))) {
		serial_info("Read thread detach successed");
	} else {
		serial_error("Read thread detach error with %d", ret);
	}

	if(!(ret = pthread_mutex_destroy(&mutex))) {
		serial_info("Mutex destroy successed");
	} else {
		serial_error("Mutex destroy error with %d", ret);
	}

	if(!(ret = pthread_cond_destroy(&cond))) {
		serial_info("Cond destroy successed");
	} else {
		serial_error("Cond destroy error with %d", ret);
	}
}


/**
 * @func:signal_handler
 * @desc:
 * @para:
 */
static void 
signal_handler(int sig) {
	int32_t ret = 0;
	
	serial_debug("Current thread id: %d", (int)pthread_self());
	serial_debug("Current process id: %d", (int)getpid());

	threads_destroy(&g_structReadStruct, \
					&g_structWriteStruct);

	close(g_structWriteStruct.fd);
	serial_warning("Xtev serial was killed");
    exit(0);
}
/**
 * @func:signal_init
 * @desc:
 * @para:
 */
static void 
signal_init(void) {
    signal(SIGKILL, &signal_handler);
    signal(SIGINT,  &signal_handler);
    signal(SIGTERM, &signal_handler);
    signal(SIGQUIT, &signal_handler);
    signal(SIGTSTP, &signal_handler);
    signal(SIGSTOP, &signal_handler);
}

/**
 * @func:main 
 * @desc:entry
 * @para:
 *      argc:parameter numbers
 *      argv:list of parameter content
 */
int 
main(int argc, char* argv[]) {
    struct termios terminalInfo;
    int32_t fd, ret;
	
    if(argc != 2) {
        serial_error("Input arguments invalid");
        goto ERROR_EXIT;
    }

	if((fd = init_uart(argv[1], &terminalInfo)) < 0) {
        serial_error("Init uart failed with %d", fd);
        goto ERROR_EXIT;
    }

	g_structReadStruct.fd = g_structWriteStruct.fd = fd;
    if(set_uart_speed(fd, &terminalInfo, UART_SPEED)) {
        serial_error("Set uart speed failed,%d", fd);
        goto CLOSE_FILE;
    }

	if((ret = pthread_create(&g_structWriteStruct.threadID, \
							NULL, \
							(void *)pthread_uart_write, 
							&g_structWriteStruct)) != 0) {
		serial_error("Create write thread failed with %d", ret);
		goto CLOSE_FILE;
	}

	if((ret = pthread_create(&g_structReadStruct.threadID, \
							NULL, \
							(void *)pthread_uart_read, \
							&g_structReadStruct)) != 0) {
		serial_error("Create read thread failed with %d", ret);
		goto CLOSE_FILE;
	}
	
	signal_init();

	while(1){
		//memset(g_structWriteStruct.bufferAddr, '\0', BUFFER_LENGTH);
		serial_debug("Current thread id: %d", (int)pthread_self());
		serial_debug("Current process id: %d", (int)getpid());
		consoleLength = read(STDIN_FILENO, g_structWriteStruct.bufferAddr, BUFFER_LENGTH);
		serial_debug("console legth is %d", consoleLength);
		//pthread_mutex_lock(&mutex);
		pthread_cond_signal(&cond);
		//pthread_mutex_unlock(&mutex);
	}

	CLOSE_FILE:
		close(fd);
	ERROR_EXIT:
		exit(1);
}





