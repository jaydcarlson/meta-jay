#include "watchdogtest.h"

static void print_usage(FILE * stream, char *app_name, int exit_code)
{
	fprintf(stream, "Usage: %s [options]\n", app_name);
	fprintf(stream,
			" test                  Call to run the tests.\n"
			" repair <error_code>   Use <error_code> fla for repair.\n");

	exit(exit_code);
}

#define BUFSIZE 16
static char * check_processes(char * cmd)
{
	char buf[BUFSIZE];
	FILE *fp;
	char * ret = "-1";

	if ((fp = popen(cmd, "r")) == NULL) {
		printf("Error opening pipe!\n");
		return ret;
	}

	while (fgets(buf, BUFSIZE, fp) != NULL) {
		//printf("Number of failed processes: %s", buf); // TODO For Debug
		ret = buf;
	}

	if(pclose(fp))  {
		printf("Command not found or exited with error status\n");
		return ret;
	}

	return ret;
}

static char* do_tests(enum Processes test)
{

	char * ret = (char*)malloc(8);
	snprintf(ret, 8, "%d", test);

	switch(test) {
	case GALLANT:
	{
		ret = check_processes(TEST_GALLANT);
		break;
	}
	case INTELLIFARM:
	{
		ret = check_processes(TEST_INTELLIFARM);
		break;
	}
	case RAINDETECT:
	{
		ret = check_processes(TEST_RAINDETECT);
		break;
	}
	case MODEM:
	{
		ret = check_processes(TEST_MODEM);
		break;
	}
	case WEATHERCRASH:
	{
		ret = check_processes(TEST_WEATHERCRASH);
		break;
	}
	case RAINCRASH:
	{
		//ret = check_processes(TEST_RAINCRASH);
		ret = "0";
		break;
	}
	default:
		break;
	}

	return ret;
}

static char* do_repairs(enum Processes repair)
{
	char * ret = (char*)malloc(8);
	snprintf(ret, 8, "%d", repair);

	switch(repair) {
	case GALLANT:
	{
		ret = check_processes(REPAIR_GALLANT);
		break;
	}
	case INTELLIFARM:
	{
		ret = check_processes(REPAIR_INTELLIFARM);
		break;
	}
	case RAINDETECT:
	{
		ret = check_processes(REPAIR_RAINDETECT);
		break;
	}
	case MODEM:
	{
		ret = check_processes(REPAIR_MODEM);
		break;
	}
	case WEATHERCRASH:
	{
		ret = check_processes(REPAIR_WEATHERCRASH);
		break;
	}
	case RAINCRASH:
	{
		//ret = check_processes(REPAIR_RAINCRASH);
		ret = "0";
		break;
	}
	default:
		break;
	}

	return ret;
}

int main(int argc, char **argv)
{
	char * ret = "";
	char exit_code[8];
	char *argv1, *argv2;

	if (argc < 2) {
		print_usage(stderr, argv[0], EXIT_FAILURE);
		return -1;
	}

	if (argc >= 2) {
		argv1 = argv[1];
		if (argc > 2)
			argv2 = argv[2];
	}

	snprintf(exit_code, sizeof(exit_code), "%d", 0);
	if (strcmp(argv1,"test") == 0)
	{
		for (int i = GALLANT; i < DONE; i++) {
			ret = do_tests((enum Processes)i);

			if((strncmp("-1", ret, 1) == 0)) {
				snprintf(exit_code, sizeof(exit_code), "%s", ret);
				break;
			}

			if(i == GALLANT || i == INTELLIFARM || i == RAINDETECT) {
				if((strncmp("IS RUNNING", ret, 10) != 0)) {
					snprintf(exit_code, sizeof(exit_code), "%d", i);
					break;
				}
			} else {
				if((strncmp("0", ret, 1) != 0)) {
					snprintf(exit_code, sizeof(exit_code), "%d", i);
					break;
				}
			}
		}
	}
	else if (strcmp(argv1,"repair") == 0)
	{
		ret = do_repairs((enum Processes)atoi(argv2));
		sleep(10);
		int repair_num = atoi(argv2);
		if(repair_num == GALLANT || repair_num == INTELLIFARM || repair_num == RAINDETECT) {
			ret = do_tests((enum Processes)repair_num);
			if((strncmp("IS RUNNING", ret, 10) != 0)) {
				snprintf(exit_code, sizeof(exit_code), "%d", repair_num);
			}
		} else {
			// Simply restart
			snprintf(exit_code, sizeof(exit_code), "%d", repair_num);
		}
	}
	else
	{
		print_usage(stderr, argv[0], EXIT_FAILURE);
	}

	//printf("Return code is %s\n", exit_code);
	return atoi(exit_code);

}
