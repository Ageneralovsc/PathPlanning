CC=gcc
CFLAGS="-Wall"

debug:clean
	$(CC) $(CFLAGS) -g -o pathplanning main.c
stable:clean
	$(CC) $(CFLAGS) -o pathplanning main.c
clean:
	rm -vfr *~ pathplanning
