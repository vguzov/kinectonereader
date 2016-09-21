all: main.o rply.o
	g++ -lfreenect2 main.o rply.o -o kinectreader
	
main.o: FirstApprox.cpp
	g++ -O3 -Wall -I/Users/Vladimir/cv/libfreenect2/include -c FirstApprox.cpp -o main.o
	
rply.o: ./rply/rply.c ./rply/rply.h ./rply/rplyfile.h
	gcc -I./rply -c ./rply/rply.c -o rply.o
	
clean:
	rm main.o rply.o