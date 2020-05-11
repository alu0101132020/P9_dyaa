IDIR = ../include
CC=g++
CFLAGS=-I$(IDIR) -std=c++17 -g

ODIR=../src

.PHONY: all

_OBJ = vector_s.o set-of-vectors.o main.o
OBJ = $(patsubst %,$(ODIR)/%,$(_OBJ))


all: max-diversity

$(ODIR)/vector_s.o: $(ODIR)/vector_s.cpp $(IDIR)/vector_s.hpp
	$(CC) -c -o $@ $< $(CFLAGS)

$(ODIR)/set-of-vectors.o: $(ODIR)/set-of-vectors.cpp $(IDIR)/set-of-vectors.hpp
	$(CC) -c -o $@ $< $(CFLAGS)

$(ODIR)/main.o: $(ODIR)/main.cpp
	$(CC) -c -o $@ $< $(CFLAGS)


max-diversity: $(OBJ)
	$(CC)  -o $@ $^ $(CFLAGS)

.PHONY: clean

clean:
	rm -f $(ODIR)/*.o ./src