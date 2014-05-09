UNAME := $(shell uname)
ifeq ($(UNAME), Linux)
flags = -std=c++11 -stdlib=libstdc++ -Weverything -DDEBUG
endif
ifeq ($(UNAME), Darwin)
flags = -std=c++11 -stdlib=libc++ -Weverything -DDEBUG
endif

# gcc -DMEMCPY
# flags = -std=c++11 -Wall -DDEBUG -DMEMCPY
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
