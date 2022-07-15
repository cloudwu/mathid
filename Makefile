CFLAGS = -Wall -O2

all :  mathid_test.exe

mathid_test.exe : mathid.c
	gcc $(CFLAGS) -DTEST_MATHID -o $@ $^