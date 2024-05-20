#ifndef INC_circlebuffer_H_
#define INC_circlebuffer_H_

#include "main.h"


#define BUFFER_SIZE 32

typedef struct
{
	uint8_t Buffer[BUFFER_SIZE];
	uint8_t Head;
	uint8_t Tail;
	uint8_t Elements;
} RingBuffer;

int8_t WriteToBuffer(RingBuffer* Buffer, uint8_t Data);

int8_t ReadFromBuffer(RingBuffer* Buffer, uint8_t* Data);

#endif

