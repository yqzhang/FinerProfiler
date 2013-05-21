/*
 * trace.c - example of a task counting event in a tree of child processes
 *
 * Copyright (c) 2009 Google, Inc
 * Contributed by Stephane Eranian <eranian@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
 * CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
 * OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
#include <sys/types.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdarg.h>
#include <signal.h>
#include <sys/wait.h>
#include <locale.h>
#include <err.h>

#include <sched.h>

#include "perf_util.h"

#define MAX_GROUPS 16

// State Machine for Stable State Detection
#define EPSILON 0.01

#define SS 0
#define WS 1
#define WN 2
#define SN 3

static int app_state[2] = {SN, SN};
static uint64_t s_cycles[2] = {0, 0};
static uint64_t s_instructions[2] = {0, 0};
static double   s_ipc[2] = {0.01, 0.01};
// State Machine for Stable State Detection

typedef struct {
	const char *events[MAX_GROUPS];
	int num_groups;
	int format_group;
	int inherit;
	int print;
	int pin;
	pid_t pid;
} options_t;

static options_t options;
static volatile int quit;

int
child(char **arg)
{
	/*
	 * execute the requested command
	 */
	execvp(arg[0], arg);
	errx(1, "cannot exec: %s\n", arg[0]);
	/* not reached */
}

static void
read_groups(perf_event_desc_t *fds, int num)
{
	uint64_t *values = NULL;
	size_t new_sz, sz = 0;
	int i, evt;
	ssize_t ret;

	/*
	 * 	{ u64		nr;
	 * 	  { u64		time_enabled; } && PERF_FORMAT_ENABLED
	 * 	  { u64		time_running; } && PERF_FORMAT_RUNNING
	 * 	  { u64		value;
	 * 	    { u64	id;           } && PERF_FORMAT_ID
	 * 	  }		cntr[nr];
	 * 	} && PERF_FORMAT_GROUP
	 *
	 * we do not use FORMAT_ID in this program
	 */

	for (evt = 0; evt < num; ) {
		int num_evts_to_read;

		if (options.format_group) {
			num_evts_to_read = perf_get_group_nevents(fds, num, evt);
			new_sz = sizeof(uint64_t) * (3 + num_evts_to_read);
		} else {
			num_evts_to_read = 1;
			new_sz = sizeof(uint64_t) * 3;
		}

		if (new_sz > sz) {
			sz = new_sz;
			values = realloc(values, sz);
		}

		if (!values)
			err(1, "cannot allocate memory for values\n");

		ret = read(fds[evt].fd, values, new_sz);
		if (ret != (ssize_t)new_sz) { /* unsigned */
			if (ret == -1)
				err(1, "cannot read values event %s", fds[evt].name);

			/* likely pinned and could not be loaded */
			warnx("could not read event %d, tried to read %zu bytes, but got %zd",
				evt, new_sz, ret);
		}

		/*
		 * propagate to save area
		 */
		for (i = evt; i < (evt + num_evts_to_read); i++) {
			if (options.format_group)
				values[0] = values[3 + (i - evt)];
			/*
			 * scaling because we may be sharing the PMU and
			 * thus may be multiplexed
			 */
			fds[i].values[0] = values[0];
			fds[i].values[1] = values[1];
			fds[i].values[2] = values[2];
		}
		evt += num_evts_to_read;
	}
	if (values)
		free(values);
}

static void
print_counts(perf_event_desc_t **all_fds, int *num)
{
	double ratio;
	uint64_t val, delta;
	int i;

    int c;
    perf_event_desc_t *fds = NULL;    
    for(c=0; c < 2; c++) {
        fds = all_fds[c];
	    read_groups(fds, num[c]);

	    for(i=0; i < num[c]; i++) {

		    val   = perf_scale(fds[i].values);
		    delta = perf_scale_delta(fds[i].values, fds[i].prev_values);
		    ratio = perf_scale_ratio(fds[i].values);

            /* instruction counts */
            if (i == 0) {
                printf ("#%d: instruction - %"PRIu64"\n", c, val);
                s_instructions[c] = delta;
            }
            else if (i == 1) {
                s_cycles[c] = delta;
            }

		    if (options.print) {
                printf("\t%s: %"PRIu64"\n", fds[i].name, delta);
            }
		    else {
			    printf("%'20"PRIu64" %s (%.2f%% scaling, ena=%'"PRIu64", run=%'"PRIu64")\n",
				    val,
				    fds[i].name,
				    (1.0-ratio)*100.0,
				    fds[i].values[1],
				    fds[i].values[2]);
            }

		    fds[i].prev_values[0] = fds[i].values[0];
		    fds[i].prev_values[1] = fds[i].values[1];
		    fds[i].prev_values[2] = fds[i].values[2];
	    }
    
        /* State Machine */
        double n_ipc = (double)s_instructions[c] / (double) s_cycles[c];
        if (n_ipc / s_ipc[c] > 1 - EPSILON && n_ipc / s_ipc[c] < 1 + EPSILON) {
            app_state[c] = (app_state[c] <= SS) ? (app_state[c]) : (app_state[c] - 1);
        }
        else {
            app_state[c] = (app_state[c] >= SN) ? (app_state[c]) : (app_state[c] + 1);
        }
        s_ipc[c] = n_ipc;
        /* State Machine */
        printf("#%d: state - %d\n\n", c, app_state[c]);
    }
}

static void sig_handler(int n)
{
	quit = 1;
}

int
parent(char **arg)
{
    perf_event_desc_t **all_fds;
	perf_event_desc_t *fds = NULL;
	int status, ret, i, *num_fds, grp, group_fd;
	int ready[2][2], go[2][2];
	char buf_0, buf_1;
	pid_t pid[2];

	go[0][0] = go[0][1] = go[1][0] = go[1][1] = -1;

    all_fds = calloc(2, sizeof(perf_event_desc_t));
    num_fds = calloc(2, sizeof(int));

	if (pfm_initialize() != PFM_SUCCESS)
		errx(1, "libpfm initialization failed");

    int c;
    for (c = 0; c < 2; c++) {
	    for (grp = 0; grp < options.num_groups; grp++) {
		    int ret;
		    ret = perf_setup_list_events(options.events[grp], &all_fds[c], &num_fds[c]);
		    if (ret || !num_fds[c])
			    exit(1);
	    }
    }

	pid[0] = pid[1] = options.pid;
	if (!pid[0] && !pid[1]) {
        /* first thread */
		ret = pipe(ready[0]);
		if (ret)
			err(1, "cannot create pipe ready");
		ret = pipe(go[0]);
		if (ret)
			err(1, "cannot create pipe go");
		/*
		 * Create the child task
		 */
		if ((pid[0]=fork()) == -1)
			err(1, "Cannot fork process");
		if (pid[0] == 0) {

            cpu_set_t cmask_0;
            CPU_ZERO (&cmask_0);
            CPU_SET (0, &cmask_0);
            sched_setaffinity (pid[0], sizeof(cpu_set_t), &cmask_0);

            printf("child_0: step 1\n");
			close(ready[0][0]);
            printf("child_0: step 2\n");
			close(go[0][1]);
            printf("child_0: step 3\n");
			close(ready[0][1]);
            printf("child_0: step 4\n");
			if (read(go[0][0], &buf_0, 1) == -1)
				err(1, "unable to read go_pipe");
            printf("child_0: step 5\n");
            printf("I'm child_0\n");
			exit(child(arg));
		}
        
		close(ready[0][1]);
		close(go[0][0]);
		if (read(ready[0][0], &buf_0, 1) == -1)
			err(1, "unable to read child_ready_pipe");
		close(ready[0][0]);

        /* second thread */
		ret = pipe(ready[1]);
		if (ret)
			err(1, "cannot create pipe ready");
		ret = pipe(go[1]);
		if (ret)
			err(1, "cannot create pipe go");
		/*
		 * Create the child task
		 */
		if ((pid[1]=fork()) == -1)
			err(1, "Cannot fork process");
		if (pid[1] == 0) {

            cpu_set_t cmask_1;
            CPU_ZERO (&cmask_1);
            CPU_SET (4, &cmask_1);
            sched_setaffinity (pid[1], sizeof(cpu_set_t), &cmask_1);

			close(ready[1][0]);
			close(go[1][1]);
			close(ready[1][1]);
			if (read(go[1][0], &buf_1, 1) == -1)
				err(1, "unable to read go_pipe");
            printf("I'm child_1\n");
			exit(child(arg));
		}

		close(ready[1][1]);
		close(go[1][0]);
		if (read(ready[1][0], &buf_1, 1) == -1)
			err(1, "unable to read child_ready_pipe");
		close(ready[1][0]);
	}

    printf("I'm parent\n");
    for(c=0; c < 2; c++) {
        fds = all_fds[c];
	    for(i=0; i < num_fds[c]; i++) {
		    int is_group_leader; /* boolean */

            if (i == 0 || i == 2 || i == 6)
                is_group_leader = 1;
            else
                is_group_leader = 0;
		    //is_group_leader = perf_is_group_leader(fds, i);
		    if (is_group_leader) {
			    /* this is the group leader */
			    group_fd = -1;
		    } else {
			    group_fd = fds[fds[i].group_leader].fd;
		    }

		    /*
		     * create leader disabled with enable_on-exec
		     */
		    if (!options.pid) {
			    fds[i].hw.disabled = is_group_leader;
			    fds[i].hw.enable_on_exec = is_group_leader;
		    }

		    fds[i].hw.read_format = PERF_FORMAT_SCALE;
		    /* request timing information necessary for scaling counts */
		    if (is_group_leader && options.format_group)
			    fds[i].hw.read_format |= PERF_FORMAT_GROUP;

		    if (options.inherit)
			    fds[i].hw.inherit = 1;

		    if (options.pin && is_group_leader)
			    fds[i].hw.pinned = 1;
		    fds[i].fd = perf_event_open(&fds[i].hw, pid[c], -1, group_fd, 0);
		    if (fds[i].fd == -1) {
			    warn("cannot attach event%d %s", i, fds[i].name);
			    goto error;
		    }
	    }
    }

	if (!options.pid && go[0][1] > -1)
		close(go[0][1]);
    if (!options.pid && go[1][1] > -1)
        close(go[1][1]);

	if (options.print) {
		if (!options.pid) {
			while(waitpid(pid[0], &status, WNOHANG) == 0) {
				sleep(1);
				print_counts(all_fds, num_fds);
			}
		} else {
			while(quit == 0) {
				sleep(1);
				print_counts(all_fds, num_fds);
			}
		}
	} else {
		if (!options.pid)
			waitpid(pid[0], &status, 0);
		else
			pause();
		print_counts(all_fds, num_fds);
	}

    for(c=0; c < 2; c++) {
        fds = all_fds[c];
	    for(i=0; i < num_fds[c]; i++)
		    close(fds[i].fd);
	    perf_free_fds(fds, num_fds[c]);
    }

	/* free libpfm resources cleanly */
	pfm_terminate();

	return 0;
error:
    for (c=0; c < 2; c++) {
        fds = all_fds[c];
	    free(fds);
	    if (!options.pid)
		    kill(SIGKILL, pid[c]);
    }

	/* free libpfm resources cleanly */
	pfm_terminate();

	return -1;
}

static void
usage(void)
{
	printf("usage: task [-h] [-i] [-g] [-p] [-P] [-t pid] [-e event1,event2,...] cmd\n"
		"-h\t\tget help\n"
		"-i\t\tinherit across fork\n"
		"-f\t\tuse PERF_FORMAT_GROUP for reading up counts (experimental, not working)\n"
		"-p\t\tprint counts every second\n"
		"-P\t\tpin events\n"
		"-t pid\tmeasure existing pid\n"
		"-e ev,ev\tgroup of events to measure (multiple -e switches are allowed)\n"
		);
}

int
main(int argc, char **argv)
{
	int c;

	setlocale(LC_ALL, "");

	while ((c=getopt(argc, argv,"+he:ifpPt:")) != -1) {
		switch(c) {
			case 'e':
				if (options.num_groups < MAX_GROUPS) {
					options.events[options.num_groups++] = optarg;
				} else {
					errx(1, "you cannot specify more than %d groups.\n",
						MAX_GROUPS);
				}
				break;
			case 'f':
				options.format_group = 1;
				break;
			case 'p':
				options.print = 1;
				break;
			case 'P':
				options.pin = 1;
				break;
			case 'i':
				options.inherit = 1;
				break;
			case 't':
				options.pid = atoi(optarg);
				break;
			case 'h':
				usage();
				exit(0);
			default:
				errx(1, "unknown error");
		}
	}
	if (options.num_groups == 0) {
		options.events[0] = "instructions,cycles";
        options.events[1] = "DTLB_LOAD_MISSES:MISS_CAUSES_A_WALK,iTLB-load-misses,MISPREDICTED_BRANCH_RETIRED,L1-icache-load-misses";
        options.events[2] = "MEM_LOAD_RETIRED:L1_HIT,MEM_LOAD_RETIRED:L2_HIT,MEM_LOAD_RETIRED:L3_HIT,LLC_MISSES";
		options.num_groups = 3;
	}
	if (!argv[optind] && !options.pid)
		errx(1, "you must specify a command to execute or a thread to attach to\n");

	signal(SIGINT, sig_handler);

	return parent(argv+optind);
}
