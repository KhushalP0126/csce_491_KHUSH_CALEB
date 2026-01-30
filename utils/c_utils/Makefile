all: waves.o
.PHONY: all

%.o: %.c %.h
	$(CC) $(CFLAGS) $< -c $(CINCLUDE)

clean:
	rm -f *.o
.PHONY: clean
