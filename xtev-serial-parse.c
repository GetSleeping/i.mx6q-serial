#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <asm-generic/ioctls.h>
#include <pthread.h>
#include "sys/types.h"
#include "sys/wait.h"

#include "xtev-serial-debug.h"

#define serial_debug(fmt, ...) \
	xtev_debug("xtev-serial-parse.c", fmt, ##__VA_ARGS__)
#define serial_info(fmt, ...) \
	xtev_info("xtev-serial-parse.c", fmt, ##__VA_ARGS__)
#define serial_warning(fmt, ...) \
	xtev_warning("xtev-serial-parse.c", fmt, ##__VA_ARGS__)
#define serial_error(fmt, ...) \
	xtev_error("xtev-serial-parse.c", fmt, ##__VA_ARGS__)

#define RUN_TRACKER             "run tracker"
#define KILL_TRACKER            "kill tracker"


static pthread_t threadTrackerID = -1;
static pid_t     processTrackerID = -1;

static void
run_track_process(void *ptr) {
	pid_t* child_pid = (pid_t*)ptr;
	pid_t f_pid = fork();
	if (f_pid < 0) {
		serial_error("Create child process failed.\
				Could not show the track.");
	} else if (f_pid == 0){
		serial_info("Create child process successed");
		//system("track all");
        char* const argv[] = {"track", "all", NULL};
        execv("/system/bin/track", argv);
	} else {
        *child_pid = f_pid;
        serial_debug("*child_pid is %d", (int)(*child_pid));
    }

    serial_debug("Current thread id: %d", (int)pthread_self());
    serial_debug("Current process id: %d", (int)getpid());
    pthread_exit(NULL);
}

int 
safely_kill_tracker(void) {
    int32_t ret = 0;
    char killCmd[15] = { '\0' };
    pid_t waitRet;
    serial_debug("process track id is %d", (int)processTrackerID);

	if(processTrackerID > 0) {
        #if 0
        if(!kill(processTrackerID,SIGKILL)) {
		    serial_debug("kill tracker successed");
        } else {
            serial_error("Cannot kill tracker process");
            return -1;
        }
        #endif
        sprintf(killCmd, "%s %d", "kill -9", (int)processTrackerID);
        system(killCmd);
        waitRet = waitpid(processTrackerID, &ret, 0);
        serial_debug("waitRet is %d", (int)waitRet);
	} else {
        serial_error("Invalid tracker process ID:%d", (int)processTrackerID);
        return -1;
    }

    return 0;
}

int
cmd_parse(char* cmd) {
    serial_debug("Receive command: %s", cmd);

    if(!strcmp(cmd, RUN_TRACKER)) {
        serial_info("run tracker command");
        if(!pthread_create(&threadTrackerID, \
							NULL, \
							(void *)run_track_process, \
							&processTrackerID)) {
            serial_debug("Create tracker thread successed");
            //serial_debug("");
        } else {
            serial_error("Create tracker thread failed");
            return -1;
        }
    } else if(!strcmp(cmd, KILL_TRACKER)) {
        serial_info("kill tracker command");
        safely_kill_tracker();
    } else {
        serial_error("Inavalid command");
        return -1;
    }

    serial_debug("Current thread id: %d", (int)pthread_self());
    serial_debug("Current process id: %d", (int)getpid());

    return 0;
}
