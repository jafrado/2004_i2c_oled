#
# Digole Display Driver library and test apps makefile
#
CFLAGS = -g -DDEBUG
CC=gcc

all: oledtest

oledtest: ssd13xx_20x4_oled.c i2c.c ssd13xx_io.c time.c main.c
	$(CC) $(CFLAGS)  ssd13xx_20x4_oled.c i2c.c time.c ssd13xx_io.c main.c  -o $@

etags TAGS ETAGS tags:
	etags -a *.c *.h

docs:
	doxygen Doxyfile

clean: 
	rm -f *.o rm ./oledtest tags ETAGS TAGS
	rm -fr doc/html doc/latex



