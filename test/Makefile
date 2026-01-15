CC = gcc
LDLIBS = -lm
CFLAGS = -g -Iinclude -Wall -Wextra
SRCS = src/test.c
TARGET = my_test

all: $(TARGET)

$(TARGET): $(SRCS)
	$(CC) $(CFLAGS) -o $(TARGET) $(SRCS) $(LDLIBS)

clean:
	rm -f $(TARGET)
