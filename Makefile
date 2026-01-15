CC = gcc
LDLIBS = -lm
CFLAGS = -g -Iinclude -Wall -Wextra
PARAM ?= main.c # Default to running main program
SRCS = $(PARAM)
TARGET = test_autogyro

all: $(TARGET)

$(TARGET): $(SRCS)
	$(CC) $(CFLAGS) -o $(TARGET) $(SRCS) $(LDLIBS)

clean:
	rm -f $(TARGET)
