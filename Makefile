all:
	gcc file.c spi.c temp.c main.c -o spi

clean:
	rm spi
