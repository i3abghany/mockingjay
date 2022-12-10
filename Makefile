bench.X86: bench.c
	gcc -g -DMAGIC bench.c --static --std=c99  -lm -o bench.X86
