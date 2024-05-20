/*
 * circlebuffer.h
 *
 *  Created on: May 13, 2024
 *      Author: konra
 */


#include "circlebuffer.h"

int8_t WriteToBuffer(RingBuffer* Buffer, uint8_t Data)
{
	uint8_t TempHead;

	TempHead = (Buffer->Head + 1) % BUFFER_SIZE;

	if (TempHead == Buffer->Tail) // No room for new data
	{
		return -1;
	}
	else
	{
		Buffer->Buffer[Buffer->Head] = Data;

		Buffer->Head++;
		Buffer->Head %= BUFFER_SIZE;

		Buffer->Elements++;
	}

	return 1;
}

int8_t ReadFromBuffer(RingBuffer* Buffer, uint8_t* Data)
{
	if (Buffer->Tail == Buffer->Head) // No data to read
	{
		return -1;
	}
	else
	{
		*Data = Buffer->Buffer[Buffer->Tail];

		Buffer->Tail++;
		Buffer->Tail %= BUFFER_SIZE;

		Buffer->Elements--;
	}
	return 1;
}
