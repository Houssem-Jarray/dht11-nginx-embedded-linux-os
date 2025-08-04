CC = gcc
CFLAGS = -Wall -Iinclude

SRC = src/gpio.c
OBJ = $(SRC:.c=.o)
BIN = test/gpio_test

all: $(BIN)

$(BIN): test/main.c $(OBJ)
	$(CC) $(CFLAGS) -o $@ $^

clean:
	rm -f $(OBJ) $(BIN)
