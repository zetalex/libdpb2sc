
#ifndef TIMER_H_   /* Include guard */
#define TIMER_H_


#include <stdio.h>
#include <signal.h>
#include <errno.h>
#include <time.h>
#include <pthread.h>
#include <unistd.h>


struct periodic_info {
	int sig;
	sigset_t alarm_sig;
};

int make_periodic(int unsigned period, struct periodic_info *info);
void wait_period(struct periodic_info *info);

#endif

