CC = gcc
LDLIBS = -lm
CFLAGS = -g -Iinclude -Wall -Wextra
SRCS = test.c
TARGET = test_autogyro

all: $(TARGET)

$(TARGET): $(SRCS)
	$(CC) $(CFLAGS) -o $(TARGET) $(SRCS) $(LDLIBS)

clean:
	rm -f $(TARGET)
