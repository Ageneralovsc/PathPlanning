CC=g++
CFLAGS="-Wall"

debug:clean
	$(CC) $(CFLAGS) -g -o pathplanning main.cc -lpthread
stable:clean
	$(CC) $(CFLAGS) -o pathplanning main.cc -lpthread
clean:
	rm -vfr *~ pathplanning
