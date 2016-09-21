all: main.o rply.o
	g++ -L/Users/Vladimir/cv/cnpy/build -lcnpy main.o rply.o -o converter
	
main.o: PlyToPng.cpp
	g++ -O0 -Wall -I/Users/Vladimir/cv/cnpy -c PlyToPng.cpp -o main.o
	
rply.o: ./rply/rply.c ./rply/rply.h ./rply/rplyfile.h
	gcc -O3 -I./rply -c ./rply/rply.c -o rply.o
	
clean:
	rm main.o rply.o