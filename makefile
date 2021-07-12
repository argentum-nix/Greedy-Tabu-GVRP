CC = g++
HEADERS = instance_util.h
OBJ = instance_util.o main.o

%.o: %.cpp $(DEPS)
	$(CC) -Wall -c -o $@ $<

GVRPGreedyTaboo: $(OBJ)
	$(CC) -Wall -o $@ $^

clean:
	-rm *.o $(OBJ) GVRPGreedyTaboo