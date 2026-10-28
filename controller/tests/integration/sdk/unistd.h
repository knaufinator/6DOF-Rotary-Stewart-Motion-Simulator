#pragma once
#include <cstdio>
#include <cstddef>
int simulated_fileno(FILE*);
int simulated_read(int,void*,size_t);
int simulated_write(int,const void*,size_t);
#define fileno simulated_fileno
#define read simulated_read
#define write simulated_write
