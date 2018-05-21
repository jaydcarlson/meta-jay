#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

/******************************************************
 * parse GPS NMEA sentences and set time
 *
 ******************************************************/
#define GPSDEVICE "/dev/ttymxc3"

#define TRUE 1
#define FALSE 0
#define SENTENCE_MAX 128

typedef struct date_time {
	int year;
	int month;
	int date;
	int hour;
	int min;
	int sec;
	int msec;
} date_time_t;

static void serial_setup(int fd, int baud);
static void process_sentence(char *s);
static int have_fix = 0;
static int have_time = 0;

int main()
{
	char sentence[SENTENCE_MAX];
	int fix;
	int fd;
	FILE *fp;

	fd = open(GPSDEVICE, O_RDWR | O_NOCTTY );
	if (fd == NULL) {
		printf("opening %s for setup, returned failed \n", GPSDEVICE);
		return -1;
	}
	serial_setup(fd, 9600);
	close(fd);

	fp = fopen(GPSDEVICE, "r");
	if (fp == NULL) {
		return -1;
	}

	while (!have_time && (fgets(sentence, SENTENCE_MAX, fp) != NULL)) {
		//printf("%s", sentence);
		process_sentence(sentence);
	}
	fclose(fp);
	return 0;
}

//char sent_gpgga[] = "$GPGGA,002153.000,3342.6618,N,11751.3858,W,1,10,1.2,27.0,M,-34.2,M,,0000*5E";
//char sent_gnrmc[] = "$GPRMC,161229.487,A,3723.2475,N,12158.3416,W,0.13,309.62,120598, ,*10";

static int parse_gprmc(char *line, 	struct date_time *datetime);
static int parse_gpgga(char *nmea, int *fix);
static void set_time(date_time_t *gpstime);

static void process_sentence(char *s)
{
	struct date_time datetime;
	static int ret;
	if (strncmp(s, "$GPGGA", 6) == 0) {
		ret = parse_gpgga(s, &have_fix);
	}
	if ((strncmp(s, "$GPRMC", 6)==0) ||(strncmp(s, "$GNRMC", 6)==0)) {
		if(have_fix != 0) {
			if(parse_gprmc(s, &datetime)) {
				set_time(&datetime);
			}
			have_time = 1;
		}
	}
}


/*
eg4. $GPRMC,hhmmss.ss,A,llll.ll,a,yyyyy.yy,a,x.x,x.x,ddmmyy,x.x,a*hh
1    = UTC of position fix
2    = Data status (V=navigation receiver warning)
3    = Latitude of fix
4    = N or S
5    = Longitude of fix
6    = E or W
7    = Speed over ground in knots
8    = Track made good in degrees True
9    = UT date
10   = Magnetic variation degrees (Easterly var. subtracts from true course)
11   = E or W
12   = Checksum
*/
static int parse_gprmc(char *line, 	struct date_time *datetime) {
	char *ptr;

	ptr = strsep(&line, ",");
    if (ptr==NULL) return(FALSE);

	ptr = strsep(&line, ",");
    if (ptr==NULL) return(FALSE);
		//printf("getting hhmmss %s \n", ptr);
		sscanf(&ptr[0], "%2d%2d%2d.%3d", &datetime->hour, &datetime->min, &datetime->sec, &datetime->msec);

	ptr = strsep(&line, ",");
    if (ptr==NULL) return(FALSE);

	ptr = strsep(&line, ",");
    if (ptr==NULL) return(FALSE);

	ptr = strsep(&line, ",");
    if (ptr==NULL) return(FALSE);

	ptr = strsep(&line, ",");
    if (ptr==NULL) return(FALSE);

	ptr = strsep(&line, ",");
    if (ptr==NULL) return(FALSE);

	ptr = strsep(&line, ",");
    if (ptr==NULL) return(FALSE);

	ptr = strsep(&line, ",");
    if (ptr==NULL) return(FALSE);

		ptr = strsep(&line, ",");
	    if (ptr==NULL) return(FALSE);
			//printf("getting ddmmyy %s \n", ptr);
			sscanf(&ptr[0], "%2d%2d%2d.%2d", &datetime->date, &datetime->month, &datetime->year);
			datetime->year += 2000;

	return TRUE;
}

//$GPGGA,hhmmss.sss,lati.tudexx,N,longi.tudexx,W,F,xx,x.xx,xxx.xxx,M,-34.458,M,0000,0000*54
static int parse_gpgga(char *line, int *fix)
{
	char *ptr;

	ptr = strsep(&line, ",");
    if (ptr==NULL) return(FALSE);

	ptr = strsep(&line, ",");
    if (ptr==NULL) return(FALSE);

	ptr = strsep(&line, ",");
    if (ptr==NULL) return(FALSE);

	ptr = strsep(&line, ",");
    if (ptr==NULL) return(FALSE);

	ptr = strsep(&line, ",");
    if (ptr==NULL) return(FALSE);

	ptr = strsep(&line, ",");
    if (ptr==NULL) return(FALSE);

	ptr = strsep(&line, ",");
    if (ptr==NULL) return(FALSE);
	sscanf(&ptr[0], "%d", fix);

	ptr = strsep(&line, ",");
    if (ptr==NULL) return(FALSE);

	return (TRUE);
}

static void set_time(date_time_t *datetime)
{
	char command[128];

	sprintf(command, "date -s %04d%02d%02d%02d%02d.%02d",
			datetime->year, datetime->month, datetime->date,
			datetime->hour, datetime->min, datetime->sec);
	//printf("command is ... %s \n", command);
	system(command);
	return;
}

static void serial_setup(int fd, int baud) {
	struct termios settings;
	tcgetattr(fd, &settings);

	cfsetospeed(&settings, baud); /* baud rate */
	settings.c_cflag &= ~PARENB; /* no parity */
	settings.c_cflag &= ~CSTOPB; /* 1 stop bit */
	settings.c_cflag &= ~CSIZE;
	settings.c_cflag |= CS8 | CLOCAL; /* 8 bits */
	settings.c_lflag = ICANON; /* canonical mode */
	settings.c_oflag &= ~OPOST; /* raw output */
	tcsetattr(fd, TCSANOW, &settings); /* apply the settings */
}
