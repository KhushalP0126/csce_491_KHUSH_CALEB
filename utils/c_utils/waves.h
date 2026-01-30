/*
 * Copyright 2021 Jason Bakos, Philip Conrad, Charles Daniels
 *
 * Distributed as part of the University of South Carolina CSCE491 course
 * materials. Please do not redistribute without written authorization.
 *
 * This file implements a library which can be used to interact with waves
 * in the format prescribed for the CSCE491 labs.
 */

/* This library implements utilities to parse the wave format used in the
 * CSCE491 course. This library is intended to only offer read-only
 * functionality for students completing their assignments using C or C++. A
 * much more featureful library is provided in Python, which supports
 * read/write, and can also convert to and from VCD. Note that the grade.sh
 * tool provided with the assignments skeleton breaks out the VCD<->plaintext
 * converter so it can be used from the shell.
 *
 * Note that this library is not intended to be robust against bad inputs or
 * improper usage.
 */

#ifndef WAVES_H
#define WAVES_H

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <err.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>

#ifndef __OpenBSD__
#ifndef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE 200809L
#endif
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#endif


/* The waves structure represents a collection of one or more digital signals.
 * Each signal is between 1 and 32 bits long, and all of the signals are
 * sampled at arbitrary (positive) time intervals. Each sample includes the
 * value at that time of every signal in the waves object.
 */
typedef struct waves_t {
	/* The list of signal names in the waves object.
	 */
	char** signals;

	/* The list of signal widths in the waves object.
	 */
	uint32_t* widths;

	/* The total number of signals (length of the signals and widths
	 * arrays).
	 */
	uint32_t nsignals;

	/* The array of signal data. The data array has a length of nsignals x
	 * nsamples. data[i * nsignals + j] is the value of the jth signal at
	 * the ith sample.
	 */
	uint32_t* data;

	/* The array of timestamps for each sample; timestamps[i] is the
	 * timestamp of the ith sample.
	 */
	float* timestamps;

	/* The total number of samples recorded in this waves object.
	 */
	uint32_t nsamples;

} waves;

/* Allocate a new waves object with enough space to store the given number of
 * signals and samples. Signal names are all initialized to be NULL pointers.
 * The resulting object must be free-ed using free_waves(). */
waves* alloc_waves(uint32_t nsignals, uint32_t nsamples);

/* Free a previously allocated waves object. */
void free_waves(waves* w);

/* Parse a waves file in the ASCII format specified for the CSCE491 course into
 * a new waves object. The waves object needs to be free-ed using free_waves().
 *
 * The caller must guarantee that the text is null-terminated. */
waves* parse(char* text);

/* Wrapper for parse() that reads the entire contents of a stream and passes it
 * to parse(), returning the result. */
waves* parse_file(FILE* stream);

/* Given a signal name, determine the index at which it occurs. The resulting
 * index should be suitable for use as a subscript into widths, or for indexing
 * into data assuming it is combined with an appropriate sample index.
 *
 * Returns -1 if the signal is not found in the waves object.
 *
 * The caller must guarantee that signal is NULL-terminated. */
int signal2index(waves* w, char* signal);

/* Given a signal index, return the corresponding signal name. The returned
 * string will be a reference into w->signals, and SHOULD NOT BE free()-ed OR
 * MODIFIED BY THE CALLER. If it is necessary to perform string operations on
 * a signal name, then the caller should use strdup() to create a duplicate
 * copy of the returned reference. */
char* index2signal(waves* w, int sigidx);

/* Given a time, find the sample index at which the given time is current, that
 * is the sample index i such that w->timestamps[i] <= time, and
 * w->timestamps[i+1] > time. If the time is earlier than the first timestamp,
 * this function returns 0. If it is greater than the last timestamp, it
 * returns w->timestamps[w->nsamples-1]. */
int time2index(waves* w, float time);

/* Given a sample index, return the corresponding floating-point timestamp.
 *
 * If the sample index is invalid, then this function will crash the program. */
float index2time(waves* w, int sampleidx);

/* Given a signal index, return a mask which will mask off all bits beyond
 * the size of the signal. For example, for a signal which is 3 bits long,
 * this function will return 0xfffffff8.
 *
 * If the signal index is invalid, then this function will crash the program. */
uint32_t mask(waves* w, int sigidx);

/* Retrieve the value of the specified signal at the specified sample index. 
 * Remember you can use signal2index() and time2index() to retrieve appropriate
 * signal and sample indices, respectively.
 *
 * The returned value will already be masked appropriately, so the caller may
 * need not mask it themselves.
 *
 * If the signal or sample indices are invalid, then this function will crash
 * the program. */
uint32_t signal_at_idx(waves* w, int sigidx, int sampleidx);

/* Wrapper around signal_at_idx() which automatically uses signal2index() and
 * time2index() to retrieve appropriate indices for the given signal name
 * and timestamp. */
uint32_t signal_at(waves* w, char* signal, float time);

/* This function returns the sample index at which the next edge for the
 * specified signal index will occur. It will return a result no earlier than
 * the sample index specified by the 'after' parameter.
 *
 * If the posedge parameter is true, then this function will consider rising
 * edges to be edges.
 *
 * If the negedge parameter is true, then this function will consider falling
 * edges to be edges.
 *
 * In short, this function can easily be used to find the next rising edge,
 * falling edge, or edge in either direction.
 *
 * If no edge is found, then this function returns -1. If the signal index is
 * invalid, then this function will crash the program. */
int next_edge_idx(waves* w, int sigidx, int after, bool posedge, bool negedge);

/* This function is a wrapper around the next_edge_idx() method, which
 * automatically looks up the signal and sample indices.
 *
 * If no edge is found, then this function returns +Inf. */
float next_edge(waves* w, char* signal, float after, bool posedge, bool negedge);

/* This macro causes the program to immediately exit with an error. */
#define panic(...) do { \
		fprintf(stderr, "PANIC at %s:%d:%s(): ", __FILE__, __LINE__, __func__); \
		fprintf(stderr, __VA_ARGS__); \
		exit(1); \
	} while(0);

/* This macro is a wrapper around malloc which will cause the program to panic
 * if the allocation fails. */
#define must_malloc(size) __extension__ ({ \
		void* _res = malloc( (size) ); \
		if (_res == NULL) { panic("malloc() failed!\n"); } \
		_res;})

/* This macro will panic if the signal index is not valid for the given waves
 * object. */
#define validate_sigidx(_w, _i) do { \
		if ((_i < 0) || ( ( (typeof(w->nsignals)) _i) >= w->nsignals)) { \
			panic("signal index %i is invalid\n", _i); \
		} \
	} while(0);

/* This macro will panic if the sample index is not valid for the given waves
 * object. */
#define validate_sampleidx(_w, _i) do { \
		if ((_i < 0) || ( (typeof(w->nsamples)) _i >= w->nsamples)) { \
			panic("sample index %i is invalid\n", _i); \
		} \
	} while(0);

/* This macro can be used to explicitly silence compiler warning about unused
 * variables. */
#define UNUSED(x) (void)(x)

/* This macro can be used to safely log data to standard error without needing
 * to worry about interfering with output that the grading script will parse.
 * */
#define log(...) do { \
		fprintf(stderr, "%s:%d:%s(): ", __FILE__, __LINE__, __func__); \
		fprintf(stderr, __VA_ARGS__); \
	} while(0);



#endif /* WAVES_H */
