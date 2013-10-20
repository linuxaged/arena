# makefile for Linux
# or use command like:
# clang++ -std=c++11  -stdlib=libc++ -Weverything Server.cpp
# to compile single file
flags = -std=c++11  -stdlib=libc++ -Weverything

% : %.cpp Net.h
	clang++ $< -o $@ ${flags}

test : Test
	./Test
	
server : Example
	./Example

client : Example
	./Example 127.0.0.1
	
clean:
	rm -f Test Example
