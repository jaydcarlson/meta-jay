/*
 * watchdogtest.h
 *
 *  Created on: Feb 17, 2017
 *      Author: fikayo
 */

#ifndef WATCHDOGTEST_H_
#define WATCHDOGTEST_H_

#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <getopt.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <unistd.h>

enum Processes {
	GALLANT = 21,
	INTELLIFARM,
	RAINDETECT,
	MODEM,
	WEATHERCRASH,
	RAINCRASH,

	DONE
};

#define TEST_GALLANT			"/usr/local/watchdog/processcheck gallant.sh"
#define TEST_INTELLIFARM		"/usr/local/watchdog/processcheck intellifarm.sh"
#define TEST_MODEM				"/usr/local/watchdog/checkmodemconnection 0"
#define TEST_RAINDETECT			"/usr/local/watchdog/processcheck raincheck"
#define TEST_WEATHERCRASH		"/usr/local/watchdog/checkweatherappcrash"
#define TEST_RAINCRASH			""

#define REPAIR_GALLANT			"/usr/local/watchdog/restartprocess gallant.sh /etc/init.d/gallant.sh"
#define REPAIR_INTELLIFARM		"/usr/local/watchdog/restartprocess intellifarm.sh /etc/init.d/intellifarm.sh"
#define REPAIR_MODEM			"/usr/local/watchdog/checkmodemconnection 1"
#define REPAIR_RAINDETECT		"/usr/local/watchdog/restartprocess raincheck /etc/init.d/raincheck"
#define REPAIR_WEATHERCRASH		"/usr/local/watchdog/checkweatherappcrash"
#define REPAIR_RAINCRASH		""

#endif /* WATCHDOGTEST_H_ */
