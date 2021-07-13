CC = g++
CFLAGS = -Wall -W 
HEADERS = instance_util.h solver.h
OBJ = instance_util.o solver.o main.o

%.o: %.cpp $(DEPS)
	$(CC) $(CFLAGS) -c -o $@ $<

GVRPGreedyTabu: $(OBJ)
	$(CC) $(CFLAGS) -o $@ $^

clean:
	-rm -rf *.o GVRPGreedyTabu