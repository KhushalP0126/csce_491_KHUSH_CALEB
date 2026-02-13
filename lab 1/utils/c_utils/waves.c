/*
 * Copyright 2021 Jason Bakos, Philip Conrad, Charles Daniels
 *
 * Distributed as part of the University of South Carolina CSCE491 course
 * materials. Please do not redistribute without written authorization.
 *
 * This file implements a library which can be used to interact with waves
 * in the format prescribed for the CSCE491 labs.
 */

/* Documentation accompanies in the wave.h file. */

#include "waves.h"

waves* alloc_waves(uint32_t nsignals, uint32_t nsamples) {
	waves* w = must_malloc(sizeof(waves));

	w->nsignals = nsignals;
	w->nsamples = nsamples;

	w->signals = must_malloc(nsignals * sizeof(char*));
	for (uint32_t i = 0 ; i < nsignals ; i++) {
		w->signals[i] = NULL;
	}

	w->widths = must_malloc(nsignals * sizeof(uint32_t));
	for (uint32_t i = 0 ; i < nsignals ; i++) {
		w->widths[i] = 0;
	}

	w->data = must_malloc(nsignals * nsamples * sizeof(uint32_t));
	for (uint32_t i = 0 ; i < nsamples * nsignals ; i++) {
		w->data[i] = 0;
	}

	w->timestamps = must_malloc(nsamples * sizeof(float));
	for (uint32_t i = 0 ; i < nsamples; i++) {
		w->timestamps[i] = 0.0;
	}

	return w;
}

void free_waves(waves* w) {
	for (uint32_t i = 0 ; i < w->nsignals ; i ++) {
		if (w->signals[i] != NULL) {
			free(w->signals[i]);
		}
	}
	free((w->signals));
	free(w->widths);
	free(w->data);
	free(w->timestamps);
	free(w);
}

/* Parser states. */

/* We are currently reading the number of samples. */
#define STATE_READING_SAMPLES 1

/* We are current reading the signal names. */
#define STATE_READING_SIGNALS 2

/* We are currently reading the widths. */
#define STATE_READING_WIDTHS 3

/* We are currently reading the timestamp of a data line. */
#define STATE_READING_TIMESTAMP 4

/* We are currently reading the data values of a data line. */
#define STATE_READING_DATA 5

/* We are looking for the number of samples, but haven't found it yet so we
 * are eating whitespace and comments until we do.
 */
#define STATE_SEEKING_SAMPLES 6
#define STATE_SEEKING_SIGNALS 7
#define STATE_SEEKING_WIDTHS 8
#define STATE_SEEKING_TIMESTAMP 9
#define STATE_SEEKING_DATA 10

/* We found a comment and we need to process it before we can go back to
 * what we were doing */
#define STATE_COMMENT 11

waves* parse(char* text) {
	size_t textlen = strlen(text);
	unsigned int cursor = 0;
	int state = STATE_SEEKING_SAMPLES;
	/* used when we encounter comments */
	int laststate = STATE_SEEKING_SAMPLES;
	int lineno = 0;
	int nsamples = -1;
	unsigned int nsignals = 0;
	unsigned int widthidx = 0;
	int sampleidx = 0;
	int signalidx = 0;
	waves* w = NULL;
	bool valid = false;

	/* accum is used to accumulate text we are processing, but haven't
	 * finished reading in yet. We make it the size of the whole input
	 * to guarantee we don't run out of space. */
	char* accum = must_malloc(sizeof(char) * (textlen+1));
	int accumidx = 0;
	accum[0] = '\0';
	accum[textlen] = '\0';

	/* Helper macro which appends to accum, advancing the null terminator
	 * appropriately. */
#define append(c) do { accum[accumidx]=c; accumidx++; accum[accumidx]='\0'; } while(0)

	/* Helper macro to empty the accum buffer. */
#define clear() do { accum[0]='\0'; accumidx=0; } while(0)

	/* Helper macro that evaluates to true if the character is a decimal
	 * digit. */
#define is_dec(c) ( (c >= '0') && (c <= '9') )

	/* Helper macro that evaluates to true if the character is a decimal
	 * digit, or '.'. */
#define is_float(c) ( is_dec(c) || (c == '.') )

	/* Helper macro that evaluates to true if the character is whitespace
	 * */
#define is_whitespace(c) ( (c == '\n') || (c == '\r') || (c == '\t') || (c == ' ') )


	while (true) {
		if (cursor >= textlen) { break; }

		char ch = text[cursor];
		char next = '\0';
		if (cursor <= (textlen-1)) { next = text[cursor+1]; }
		cursor ++;
		if (ch == '\n') { lineno ++; }

		/*
		log("lineno=%i, cursor=%i, ch='%c' (0x%x), next='%c' (0x%x), state=%i\n",
				lineno, cursor, ch, ch, next, next, state);
		*/

		if (state == STATE_READING_SAMPLES) {
			if (is_dec(ch)) { append(ch); }

			if (!(is_dec(next))) {
				/* Finished reading all the digits. */
				nsamples = atoi(accum);
				if (nsamples < 1) {
					panic("expected to read a positive number of samples, but got %i\n", nsamples);
				}
				clear();
				state = STATE_SEEKING_SIGNALS;
				continue;
			}

			if (is_dec(ch)) { continue; }

			panic("Syntax error on line %i, got unexpected character '%c' while reading a sample count\n", lineno, ch);
		}

		if (state == STATE_READING_SIGNALS) {
			/* we want to skip the space in between signals */
			if (is_whitespace(ch)) {continue;}

			if (!is_whitespace(ch)) { append(ch); }

			if (is_whitespace(ch) || is_whitespace(next)) {
				/* end of this signal name */
				nsignals ++;

				/* We will use \0 as a delimiter while we count
				 * up the signals. Note that this technically
				 * creates an edge case, since if textlen is
				 * small enough, there may not be enough memory
				 * for our extra null bytes. This is very
				 * unlikely if the file is valid though, since
				 * the widths line will be at least as long over
				 * again as our signals line. */
				append('\0');

				if ((next == '\n') || (ch == '\n')) {
					if (w != NULL) {
						panic("You should never see this. If you do, it means the parser has decided it is reading the signals line again, which means Charles messed up the state transitions.\n");
					}

					/* This is the end of the signals line - we
					 * already accumulated the whole final signal
					 * name. This also means we know both the
					 * number of signals and the number of samples
					 * so we can allocate the waves object now. */
					w = alloc_waves(nsignals, nsamples);

					/* We need to go back and read all the signal
					 * names we left in accum. ptr references some
					 * character between the start and end of
					 * accum. We carefully maintain it so that it
					 * always points to the start of one of our
					 * null-delimited strings. */
					char* ptr = accum;
					for (uint32_t i = 0 ; i < nsignals ; i++) {
						w->signals[i] = strndup(ptr, textlen);

						/* This advances the pointer by the
						 * length of this particular string. */
						ptr = &(ptr[strnlen(ptr, textlen)+1]);
					}

					clear();
					state = STATE_SEEKING_WIDTHS;
				}
				continue;
			}

			if (!is_whitespace(ch)) { continue; }

			/* This is almost certainly unreachable. */
			panic("Syntax error on line %i, got unexpected character '%c' while parsing signal names\n", lineno, ch);
		}

		if (state == STATE_READING_WIDTHS) {
			if (is_dec(ch)) { append(ch); }

			if (is_whitespace(next)) {
				/* We finished reading the current character
				 * width. */

				w->widths[widthidx] = atoi(accum);
				widthidx ++;
				clear();

				if (widthidx > nsignals) {
					panic("Syntax error on line %i, too many widths, should only be %i\n", lineno, nsignals);
				}

				if (next == '\n') {
					if (widthidx < nsignals) {
						panic("Syntax error on line %i, too few widths %i, should be %i\n", lineno, widthidx, nsignals);
					}
					state = STATE_SEEKING_TIMESTAMP;
				}
				continue;
			}

			if (is_dec(next)) {continue;}

			panic("Syntax error on line %i, got unexpected character '%c' while parsing a signal width\n", lineno, ch);
		}

		if (state == STATE_READING_TIMESTAMP) {
			if (sampleidx >= nsamples) {
				panic("Syntax error on line %i, too many sample lines, should only be %i\n", lineno, nsamples);
			}

			if (is_float(ch)) { append(ch); }

			if (is_whitespace(next)) {
				w->timestamps[sampleidx] = atof(accum);

				/* Make sure we aren't moving backwards in
				 * time. */
				if (sampleidx > 0) {
					if (w->timestamps[sampleidx] < w->timestamps[sampleidx-1]) {
						panic("Syntax error on line %i, time seems to be moving backwards!\n", lineno);
					}
				}

				state = STATE_SEEKING_DATA;
				clear();
				continue;
			}

			if (is_float(ch)) { continue; }

			panic("Syntax error on line %i, got unexpected character '%c' while parsing a timestamp\n", lineno, ch);
		}

		if (state == STATE_READING_DATA) {
			if (is_dec(ch)) { append(ch); }

			if (is_whitespace(ch) || is_whitespace(next)) {
				w->data[sampleidx*nsignals + signalidx] = atoi(accum) & mask(w, signalidx);

				/* log("signalidx=%i, accum='%s', mask=%i val=%i\n", signalidx, accum, mask(w, signalidx), w->data[sampleidx*nsignals + signalidx]); */

				signalidx++;
				clear();

				// We need to check for ch == \n in case this
				// is a one-character long data item.
				if ((next == '\n') || (ch == '\n')) {
					state = STATE_SEEKING_TIMESTAMP;

					if ((unsigned int) signalidx != (w->nsignals)) {
						/* note that signalidx has
						 * aleady been incremented at
						 * this point */
						panic("Syntax error on line %i, expected %i data values, but got %i\n", lineno, w->nsignals, signalidx);
					}

					signalidx = 0;
					sampleidx ++;

					/* If we have read at least one data
					 * line, then w is valid and can be
					 * returned when we hit the end of the
					 * file. */
					valid = true;
				} else {
					state = STATE_SEEKING_DATA;
				}
				continue;
			}

			/* Skip whitespace in between the data items. */
			if (is_whitespace(ch)) { continue; }

			if (is_dec(ch)) { continue; }

			panic("Syntax error on line %i, got unexpected character '%c' while parsing a data item\n", lineno, ch);
		}

		if (state == STATE_SEEKING_SAMPLES) {
			/* If we are looking for the number of samples and
			 * we have just found a decimal digit, we found it. */
			if (is_dec(ch)) {
				clear();
				append(ch);
				state = STATE_READING_SAMPLES;
				continue;
			}

			/* Ignore leading whitespace. */
			if (is_whitespace(ch)) { continue; }

			/* Ignore comments. */
			if (ch == '#') {
				laststate = state;
				state = STATE_COMMENT;
				continue;
			}

			panic("Syntax error on line %i, got unexpected character '%c' while looking for a sample count\n", lineno, ch);
		}

		if (state == STATE_SEEKING_SIGNALS) {
			if (!is_whitespace(ch)) {
				clear();
				append(ch);
				state = STATE_READING_SIGNALS;
				continue;
			}

			/* Ignore leading whitspace. */
			if (is_whitespace(ch)) { continue; }

			/* Ignore comments. */
			if (ch == '#') {
				laststate = state;
				state = STATE_COMMENT;
				continue;
			}

			panic("Syntax error on line %i, got unexpected character '%c' while looking for a signal line\n", lineno, ch);
		}

		if (state == STATE_SEEKING_WIDTHS) {
			if (is_dec(next)) {
				clear();
				state = STATE_READING_WIDTHS;
				continue;
			}

			/* Ignore leading whitespace. */
			if (is_whitespace(ch)) { continue; }

			/* Ignore comments. */
			if (ch == '#') {
				laststate = state;
				state = STATE_COMMENT;
				continue;
			}

			panic("Syntax error on line %i, got unexpected character '%c' while looking for a widths line.\n", lineno, ch);
		}

		if (state == STATE_SEEKING_TIMESTAMP) {
			if (is_float(ch)) {
				clear();
				append(ch);
				state = STATE_READING_TIMESTAMP;
				continue;
			}

			/* Ignore leading whitespace. */
			if (is_whitespace(ch)) { continue; }

			/* Ignore comments. */
			if (ch == '#') {
				laststate = state;
				state = STATE_COMMENT;
				continue;
			}

			panic("Syntax error on line %i, got unexpected character '%c' while looking for a timestamp.\n", lineno, ch);
		}

		if (state == STATE_SEEKING_DATA) {
			if (is_dec(ch)) {
				clear();
				append(ch);
				state = STATE_READING_DATA;
				continue;
			}

			/* Ignore leading whitespace. */
			if (is_whitespace(ch)) { continue; }

			/* Comments are now allowed between timestamps and data
			 * items. */

			panic("Syntax error on line %i, got unexpected character '%c' while looking for a data item.\n", lineno, ch);
		}

		if (state == STATE_COMMENT) {
			/* We finished reading in the comment, go back to
			 * what we were doing */
			if (ch == '\n') {
				state = laststate;
			}

			/* Any character is allowed in a comment. */
			continue;
		}

	}

	free(accum);

	if (valid) {
		return w;
	}

	panic("Reached end of input file, but didn't read enough data to generate a valid waves object. Input file is malformed.\n");
	return NULL;

#undef append
#undef clear
#undef is_dec
#undef is_float
#undef is_whitespace

}
#undef STATE_READING_SAMPLES
#undef STATE_READING_SIGNALS
#undef STATE_READING_WIDTHS
#undef STATE_READING_TIMESTAMP
#undef STATE_READING_DATA
#undef STATE_SEEKING_SAMPLES
#undef STATE_SEEKING_SIGNALS
#undef STATE_SEEKING_WIDTHS
#undef STATE_SEEKING_TIMESTAMP
#undef STATE_SEEKING_DATA
#undef STATE_COMMENT

waves* parse_file(FILE* stream) {
#define BUFSZ 1024

	char buf[BUFSZ];
	size_t len = 1;
	char* text = must_malloc(sizeof(char) * BUFSZ);
	text[0] = '\0';

	while(fgets(buf, BUFSZ, stream)) {
		len += strlen(buf);
		text = realloc(text, len);
		if (text == NULL) {
			panic("Failed to allocate memory.");
		}
		strcat(text, buf);
	}

	waves* w = parse(text);
	free(text);
	return w;

#undef BUFSZ
}

int signal2index(waves* w, char* signal) {
	for (uint32_t i = 0 ; i < w->nsignals; i++) {
		if (strcmp(signal, w->signals[i]) == 0) {
			return i;
		}
	}

	return -1;
}

char* index2signal(waves* w, int sigidx) {
	validate_sigidx(w, sigidx);

	return w->signals[sigidx];
}

int time2index(waves* w, float time) {
	int index = w->nsamples / 2;
	int step = index - 1;

	while (true) {
		if (index >= (int) (w->nsamples-1)) {
			if (step < 2) {
				return w->nsamples-1;
			} else {
				index = w->nsamples-1;
			}
		}

		if (index < 0) {
			if (step < 2) {
				return 0;
			} else {
				index = 0;
			}
		}

		if ((w->timestamps[index] <= time) && (w->timestamps[index+1] > time)) {
			return index;
		}

		if (w->timestamps[index] < time) { index = index + step; }
		else                             { index = index - step; }

		float fstep = (float) step / 2.0;
		if (fstep < 1.0) {
			fstep = 1;
		}
		step = (int) fstep;
	}
}

float index2time(waves* w, int sampleidx) {
	validate_sampleidx(w, sampleidx);
	return w->timestamps[sampleidx];
}

uint32_t mask(waves* w, int sigidx) {
	validate_sigidx(w, sigidx);
	return ((1 << w->widths[sigidx]) - 1);
}

uint32_t signal_at_idx(waves* w, int sigidx, int sampleidx) {
	validate_sampleidx(w, sampleidx);
	validate_sigidx(w, sigidx);

	return mask(w, sigidx) & w->data[sampleidx * w->nsignals + sigidx];
}

uint32_t signal_at(waves* w, char* signal, float time) {
	int sampleidx = time2index(w, time);
	int sigidx = signal2index(w, signal);
	return signal_at_idx(w, sigidx, sampleidx);
}

int next_edge_idx(waves* w, int sigidx, int after, bool posedge, bool negedge) {
	if (after == 0) { after = 1; }

	validate_sampleidx(w, after);
	validate_sigidx(w, sigidx);

	int sampleidx = after;

	while(sampleidx < (int) w->nsamples) {
		uint32_t now = signal_at_idx(w, sigidx, sampleidx-1);
		uint32_t next = signal_at_idx(w, sigidx, sampleidx);

		if (posedge && (now < next)) { return sampleidx; }
		if (negedge && (now > next)) { return sampleidx; }

		sampleidx ++;
	}

	/* ran off the end */
	return -1;
}

float next_edge(waves* w, char* signal, float after, bool posedge, bool negedge) {
	int afteridx = time2index(w, after)+1;
	if (afteridx >= (int) w->nsamples) {afteridx = w->nsamples-1;}
	int sigidx = signal2index(w, signal);
	int sampleidx = next_edge_idx(w, sigidx, afteridx, posedge, negedge);

	if (sampleidx < 0) { return INFINITY;  }

	return index2time(w, sampleidx);
}
