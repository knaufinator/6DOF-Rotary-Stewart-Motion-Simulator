#pragma once
#define F_GETFL 1
#define F_SETFL 2
#define O_NONBLOCK 4
inline int fcntl(int,int,int) { return 0; }
