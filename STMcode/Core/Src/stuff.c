#include "stuff.h"



void swapEndian(uint8_t data[], uint16_t len) {
	// even len:
	// [0, 1, 2, 3] -> [3, 1, 2, 0] -> [3, 2, 1, 0]
	// i: 0 to 4/2 = 2

	// odd len:
	// [0, 1, 2, 3, 4] -> [4, 1, 2, 3, 0] -> [4, 3, 2, 1, 0]
	// i: 0 to 5/2 = 2

	uint8_t temp;


	for (int i = 0; i < (len / 2); i++) {
		temp = data[i];
		data[i] = data[len-1-i];
		data[len-1-i] = temp;
	}
}

uint8_t writeEnable() {
	uint8_t Tx[2];
	uint8_t Rx[2];


	HAL_Delay(1);

	// Send Write enable instruction
	Tx[0] = WRITE_ENABLE;

	HAL_GPIO_WritePin(CS_PIN_GPIO_Port, CS_PIN_Pin, GPIO_PIN_RESET);
	for (int i = 0; i < 25; i++) asm("nop");

	HAL_SPI_Transmit(&hspi1, Tx, 1, SPI_TIMEOUT);

	for (int i = 0; i < 25; i++) asm("nop");
	HAL_GPIO_WritePin(CS_PIN_GPIO_Port, CS_PIN_Pin, GPIO_PIN_SET);


	HAL_Delay(1);

	// Check status register if WEL is high
	Tx[0] = READ_STATUS_REG;

	HAL_GPIO_WritePin(CS_PIN_GPIO_Port, CS_PIN_Pin, GPIO_PIN_RESET);
	for (int i = 0; i < 25; i++) asm("nop");

	HAL_SPI_TransmitReceive(&hspi1, Tx, Rx, 2, SPI_TIMEOUT);

	for (int i = 0; i < 25; i++) asm("nop");
	HAL_GPIO_WritePin(CS_PIN_GPIO_Port, CS_PIN_Pin, GPIO_PIN_SET);

	// If the WEL bit is low or the BUSY flag is high
	if (  ((Rx[1] & 0x02) != 0x02) || ((Rx[1] & 0x01) == 0x01)  )
		return 0;

	// If a page program instruction can be executed
	else
		return 1;
}


uint8_t writeData(uint8_t *data, uint32_t startAddr, uint32_t bytesToWrite) {
	HAL_StatusTypeDef status;
	uint16_t writeLen = 0;
	uint32_t bytesWritten = 0;

	uint32_t initialBytesToWrite = bytesToWrite;

	uint8_t Tx[256+4];




	// 1.) basic 256            	-- 1 Instruction
	// 0x00, write 256
	// initial for 256 by

	// 2.) change 256				-- 2 Instructions
	// 0x00F, write 256
	// initial for 241 b
	// change 1 for 15 b

	// 3.) basic 512  				-- 3 Instructions
	// 0x000, write 512 bytes
	// initial for 256 b
	// change 1 for 256 b
	// 0x200

	// 4.) change 512				-- 4 Instructions
	// 0x0FE, write 512 b
	// initial for 1 byte
	// 0x100 change 1 for 256 b
	// 0x200 change 2 for 255 b
	// 0x2FE

	// 5.) special case				-- 1 instruction
	// 0x0F write 15 bytes
	// initial write for 15 b
	// 0x1E		-- instructionsNeeded = 1; isBasic = 0

	// 6.) special case 			-- 1 instruction
	// 0x00 write 15 bytes
	// initial write 15 b
	// 0x10  	-- instructionsNeeded = 0; isBasic = 1


	// addr is 0x2276
	// write 0xe4 bytes
	// needs two instrcutions
	// Write from 0x2276-0x22FF, 0xe4 - (0xff - 0x76) = 0x5b bytes left
	// Write from 0x2300-0x235b




	uint16_t instructionsNeeded = bytesToWrite / 256;			// Basic Case: Works for addrs aligned at 0x00 and writing more than 256 bytes
	uint8_t isBasic = ((startAddr & 0xFF) == 0) ? 1 : 0;

	// If there will be a roll over

	instructionsNeeded += (((startAddr & 0xFF) + bytesToWrite) > 0xFF) ? 1:0;


	instructionsNeeded += !isBasic;								// Special Case: add 1 instruction if the alignment is off

	if (isBasic && (bytesToWrite < 256))						// Special Case: If the starting addr is 0 and writing less than 256 bytes
		instructionsNeeded += 1;


	// TODO : test all cases
	for (int i = 0; i < instructionsNeeded; i++) {
		// Try to write enable, check if it worked and make sure mem is not BUSY
		if (writeEnable() == 0)
			return 0;


		memset(Tx, 0, 260);			// Reset to 0s every time

		/* If the addr is not aligned at 0x00 -- can't write a full page at once */
		if (!isBasic) {
			// Setup Instruction and addr
			Tx[0] = PAGE_PROGRAM;
			Tx[1] = (startAddr & 0x00FF0000) >> 16;
			Tx[2] = (startAddr & 0x0000FF00) >> 8;
			Tx[3] =  startAddr & 0x000000FF;


			writeLen = 0xFF - (startAddr & 0xFF);								// How many bytes left before rollover
			if (bytesToWrite < writeLen)										// If the bytes left fits in bytes before roll over
				writeLen = bytesToWrite;

			// Copy data after instruction and addr
			memcpy(Tx+4, data+bytesWritten, writeLen);

			bytesWritten += writeLen + 1;													// Advance the pointer to next write operation

			HAL_GPIO_WritePin(CS_PIN_GPIO_Port, CS_PIN_Pin, GPIO_PIN_RESET);
			for (int i = 0; i < 25; i++) asm("nop");

			status = HAL_SPI_Transmit(&hspi1, Tx, writeLen+4, SPI_TIMEOUT);

			for (int i = 0; i < 25; i++) asm("nop");
			HAL_GPIO_WritePin(CS_PIN_GPIO_Port, CS_PIN_Pin, GPIO_PIN_SET);

			if (writeLen == bytesToWrite)
				break;															// If we finished writing in 1 page exit loop

			startAddr += writeLen+1;												// If we are writing another page this should be 0x00
			bytesToWrite -= writeLen+1;
			isBasic = 1;														// After 1 write for alignement the rest should be the basic case
		}

		/* If the addr is aligned at 0x00 -- can write a full page at once */
		else {
			Tx[0] = PAGE_PROGRAM;
			Tx[1] = (startAddr & 0x00FF0000) >> 16;
			Tx[2] = (startAddr & 0x0000FF00) >> 8;
			Tx[3] =  startAddr & 0x000000FF;


			writeLen = (bytesToWrite > 256) ? 256 : bytesToWrite;				// If more than 256 write 256, if less, finish writing file

			// Copy data after instruction and addr
			memcpy(Tx+4, data+bytesWritten, writeLen);

			bytesWritten += writeLen;			// Advance the pointer to next write operation


			HAL_GPIO_WritePin(CS_PIN_GPIO_Port, CS_PIN_Pin, GPIO_PIN_RESET);
			for (int i = 0; i < 25; i++) asm("nop");

			status = HAL_SPI_Transmit(&hspi1, Tx, writeLen+4, SPI_TIMEOUT);

			for (int i = 0; i < 25; i++) asm("nop");
			HAL_GPIO_WritePin(CS_PIN_GPIO_Port, CS_PIN_Pin, GPIO_PIN_SET);

			startAddr += 0x100;										// If we are writing more than 256 bytes need in increment pointer
			bytesToWrite -= 0x100;									// Reduce bytes to write by 256
		}

		if (status != HAL_OK) {
			return 0;
		}

		// Check if the len rolled over, which indicates we are done writing
		if (bytesToWrite > initialBytesToWrite) {
			break;
		}
	}

	return 1;
}


uint8_t* readData(uint32_t startAddr, uint32_t bytesToRead) {
	HAL_StatusTypeDef status;
	uint8_t Tx[bytesToRead+4];


	// Check status register if BUSY
	Tx[0] = READ_STATUS_REG;
	uint8_t reg[2] = {0, 0x01};	// Start with 0x03 so that loop will run at least once

	while ((reg[1] & 0x01) == 0x01) { 		// Continue to read status register if BUSY is HIGH
		HAL_GPIO_WritePin(CS_PIN_GPIO_Port, CS_PIN_Pin, GPIO_PIN_RESET);
		for (int i = 0; i < 25; i++) asm("nop");

		HAL_SPI_TransmitReceive(&hspi1, Tx, reg, 2, SPI_TIMEOUT);

		for (int i = 0; i < 25; i++) asm("nop");
		HAL_GPIO_WritePin(CS_PIN_GPIO_Port, CS_PIN_Pin, GPIO_PIN_SET);
	}

	uint8_t *Rx = (uint8_t*) malloc(bytesToRead+4);
	memset(Rx, 0, bytesToRead+4);

	Tx[0] = READ_DATA;
	Tx[1] = (startAddr & 0x00FF0000) >> 16;
	Tx[2] = (startAddr & 0x0000FF00) >> 8;
	Tx[3] =  startAddr & 0x000000FF;


	HAL_GPIO_WritePin(CS_PIN_GPIO_Port, CS_PIN_Pin, GPIO_PIN_RESET);
	for (int i = 0; i < 25; i++) asm("nop");

	status = HAL_SPI_TransmitReceive(&hspi1, Tx, Rx, bytesToRead+4, SPI_TIMEOUT);

	for (int i = 0; i < 25; i++) asm("nop");
	HAL_GPIO_WritePin(CS_PIN_GPIO_Port, CS_PIN_Pin, GPIO_PIN_SET);

	if (status != HAL_OK) {
		return 0;
	}

	return Rx;
}

uint8_t chipErase(void) {
	// Try to write enable, check if it worked and make sure mem is not BUSY
	if (writeEnable() == 0)
		return 0;


	uint8_t Tx[4];


	// Setup Instruction and addr
	Tx[0] = CHIP_ERASE;

	HAL_GPIO_WritePin(CS_PIN_GPIO_Port, CS_PIN_Pin, GPIO_PIN_RESET);
	for (int i = 0; i < 25; i++) asm("nop");

	HAL_SPI_Transmit(&hspi1, Tx, 1, SPI_TIMEOUT);

	for (int i = 0; i < 25; i++) asm("nop");
	HAL_GPIO_WritePin(CS_PIN_GPIO_Port, CS_PIN_Pin, GPIO_PIN_SET);



	HAL_Delay(5);

	// Check status register if WEL is high
	Tx[0] = READ_STATUS_REG;
	uint8_t Rx[2] = {0, 0x03};							// Start with 0x03 so that loop will run at least once
	while ((Rx[1] & 0x03) != 0x00) { 					// Continue to read status register if BUSY is HIGH or WEL has not returned LOW
		HAL_GPIO_WritePin(CS_PIN_GPIO_Port, CS_PIN_Pin, GPIO_PIN_RESET);
		for (int i = 0; i < 25; i++) asm("nop");

		HAL_SPI_TransmitReceive(&hspi1, Tx, Rx, 2, SPI_TIMEOUT);

		for (int i = 0; i < 25; i++) asm("nop");
		HAL_GPIO_WritePin(CS_PIN_GPIO_Port, CS_PIN_Pin, GPIO_PIN_SET);

	}

	return 1;

}



uint8_t sectorErase(uint32_t addr) {

	// Try to write enable, check if it worked and make sure mem is not BUSY
	if (writeEnable() == 0)
		return 0;


	uint8_t Tx[4];


	// Setup Instruction and addr
	Tx[0] = SECTOR_ERASE;
	Tx[1] = (addr & 0x00FF0000) >> 16;
	Tx[2] = (addr & 0x0000FF00) >> 8;
	Tx[3] =  addr & 0x000000FF;


	HAL_GPIO_WritePin(CS_PIN_GPIO_Port, CS_PIN_Pin, GPIO_PIN_RESET);
	for (int i = 0; i < 25; i++) asm("nop");

	HAL_SPI_Transmit(&hspi1, Tx, 4, SPI_TIMEOUT);

	for (int i = 0; i < 25; i++) asm("nop");
	HAL_GPIO_WritePin(CS_PIN_GPIO_Port, CS_PIN_Pin, GPIO_PIN_SET);



	HAL_Delay(1);

	// Check status register if WEL is high
	Tx[0] = READ_STATUS_REG;
	uint8_t Rx[2] = {0, 0x03};	// Start with 0x03 so that loop will run at least once

	while ((Rx[1] & 0x03) != 0x00) { 		// Continue to read status register if BUSY is HIGH or WEL has not returned LOW
		HAL_GPIO_WritePin(CS_PIN_GPIO_Port, CS_PIN_Pin, GPIO_PIN_RESET);
		for (int i = 0; i < 25; i++) asm("nop");

		HAL_SPI_TransmitReceive(&hspi1, Tx, Rx, 2, SPI_TIMEOUT);

		for (int i = 0; i < 25; i++) asm("nop");
		HAL_GPIO_WritePin(CS_PIN_GPIO_Port, CS_PIN_Pin, GPIO_PIN_SET);

	}


	return 1;
}


void profileMemory(void) {
	struct INode *INodes = malloc(0);
	uint8_t *INodeNumbers = malloc(0);

	uint8_t numINodes;
	uint32_t sum = 0;
	unsigned char buff[64];
	int j;

	numINodes = readINodes(&INodes, &INodeNumbers);

	if (numINodes == 0) {
		sum = 0;
	}
	else {
		for (int i = 0; i < numINodes; i++) {
			sum += INodes[i].fileSize;
		}
	}

	/* Write free space available */
	HAL_UART_Transmit((UARTFlag) ? &huart1 : &huart3, "Free space:\t", 12, 100);

	sprintf(buff, "%.2lfKB", ((1024.0*504.0 - sum) / 1024.0));

	if (strlen(buff) < 8) {
		for (j = 0; j < (8 - strlen(buff)); j++) {
			HAL_UART_Transmit((UARTFlag) ? &huart1 : &huart3, "0", 1, 100);			// Pad left side with spaces if necessary
		}
	}

	HAL_UART_Transmit((UARTFlag) ? &huart1 : &huart3, buff, strlen(buff), 100);

	/* Write used space */
	HAL_UART_Transmit((UARTFlag) ? &huart1 : &huart3, "\tUsed space:\t", 13, 100);

	memset(buff, 0, 64);
	sprintf(buff, "%.2lfKB", (sum / 1024.0));

	if (strlen(buff) < 8) {
		for (j = 0; j < (8 - strlen(buff)); j++) {
			HAL_UART_Transmit((UARTFlag) ? &huart1 : &huart3, "0", 1, 100);			// Pad left side with spaces if necessary
		}
	}

	HAL_UART_Transmit((UARTFlag) ? &huart1 : &huart3, buff, strlen(buff), 100);
	HAL_UART_Transmit((UARTFlag) ? &huart1 : &huart3, "\n", 1, 100);


	if (INodeNumbers != NULL)
		free(INodeNumbers);
	if (INodes != NULL)
		free(INodes);

	return;
}



void getDir() {
	struct INode *INodes = malloc(0);
	uint8_t *INodeNumbers = malloc(0);

	uint8_t numINodes;
	uint8_t *file;

	uint8_t numFiles;
	uint8_t temp;
	uint8_t i;
	uint8_t j;

	numINodes = readINodes(&INodes, &INodeNumbers);
	if (numINodes == 0) {
		HAL_UART_Transmit((UARTFlag) ? &huart1 : &huart3, "No file system was found\n", 25, 100);
		free(INodes);
		free(INodeNumbers);
		return;
	}

	numFiles = INodes[0].fileSize / 64;					// Number of files in dir

	file = readFile(INodes[0]);							// Read contents of base dir
	char digit;
	char size[9];			// 000.00KB \0


	HAL_UART_Transmit((UARTFlag) ? &huart1 : &huart3, "ID    SIZE             File Name\n", 33, 100);

	// Iterate through entries in the directory file
	for (i = 0; i < numFiles; i++) {
		temp = file[i*(NAME_LIM+1)];					// INode number is first byte of directory entry

		// Write INode number
		if (temp < 9) {
			digit = '0' + temp;
			HAL_UART_Transmit((UARTFlag) ? &huart1 : &huart3, &digit, 1, 100);
			HAL_UART_Transmit((UARTFlag) ? &huart1 : &huart3, "    ", 4, 100);
		}
		else if (temp < 99) {
			digit = '0' + (temp / 10);
			HAL_UART_Transmit((UARTFlag) ? &huart1 : &huart3, &digit, 1, 100);

			digit = '0' + (temp % 10);
			HAL_UART_Transmit((UARTFlag) ? &huart1 : &huart3, &digit, 1, 100);

			HAL_UART_Transmit((UARTFlag) ? &huart1 : &huart3, "   ", 3, 100);
		}
		else {
			digit = '0' + (temp / 100);
			HAL_UART_Transmit((UARTFlag) ? &huart1 : &huart3, &digit, 1, 100);

			digit = '0' + (temp / 10);
			HAL_UART_Transmit((UARTFlag) ? &huart1 : &huart3, &digit, 1, 100);

			digit = '0' + (temp % 10);
			HAL_UART_Transmit((UARTFlag) ? &huart1 : &huart3, &digit , 1, 100);

			HAL_UART_Transmit((UARTFlag) ? &huart1 : &huart3, "  ", 2, 100);
		}

		HAL_UART_Transmit((UARTFlag) ? &huart1 : &huart3, "  ", 2, 100);


		// Write File len in Kb
		for (j = 0; j < numINodes; j++) {
			if (temp == INodeNumbers[j]) {					// File size is stored in the INode
				break;
			}
		}

		// Use sprintf to convert double to str **need to change project config to compile this
		sprintf(size, "%.2lfKB", (INodes[j].fileSize / 1024.0));

		if (strlen(size) < 8) {
			for (j = 0; j < (8 - strlen(size)); j++) {
				HAL_UART_Transmit((UARTFlag) ? &huart1 : &huart3, "0", 1, 100);			// Pad left side with spaces if necessary
			}
		}
		HAL_UART_Transmit((UARTFlag) ? &huart1 : &huart3, size, strlen(size), 100);

		HAL_UART_Transmit((UARTFlag) ? &huart1 : &huart3, "     ", 4, 100);

		// Write File Name
		HAL_UART_Transmit((UARTFlag) ? &huart1 : &huart3, &file[i*(NAME_LIM+1) + 1], strlen(&file[i*(NAME_LIM+1) + 1]), 100);

		HAL_UART_Transmit((UARTFlag) ? &huart1 : &huart3, "\n", 1, 100);
	}

	free(INodes);
	free(INodeNumbers);
	free(file);
	return;
}

uint8_t* readFile(struct INode node) {
	uint8_t *file = malloc(0);
	uint8_t *temp;
	uint32_t offset = 0;
	uint32_t segmentLen;


	for (int i = 0; i < 5; i++) {
		if (node.addr[i*2] != 0 && node.addr[i*2+1] != 0) {							// If the pointer is valid
			segmentLen = node.addr[i*2+1] - node.addr[i*2];

			temp = readData(node.addr[i*2], segmentLen);							// Read the data in the first pair

			file = realloc(file, offset+segmentLen);								// Increase the size by the new segment
			memcpy(file+offset, temp+4, segmentLen);									// Append if multiple segments

			offset += node.addr[i*2+1] - node.addr[i*2];								// Stores end of array
			free(temp);																// If read data is called need to free memory
		}
	}
	return file;
}

uint8_t SetupFilesystem() {
	struct INode baseDir;

	baseDir.fileType = 0;
	baseDir.fileSize = 0;
	memset(baseDir.addr, 0, 10 * sizeof(uint32_t));


	/* WRITE BASE DIR INODE TO MEMORY */
	uint8_t *INodeArray = malloc(INODE_SIZE);

	structToArray(baseDir, INodeArray);

	if (writeData(INodeArray, 0x000000, INODE_SIZE) != 1) {
		free(INodeArray);
		return 0;
	}

	free(INodeArray);

	HAL_Delay(1);

	return 1;
}

uint8_t writeFile(uint8_t *file, uint32_t len, char newFilename[NAME_LIM], uint8_t fileType) {
	uint8_t needNewPointer = 1;

	uint8_t numINodes;
	struct INode *INodes = malloc(0);
	uint8_t *INodeNumbers = malloc(0);

	uint8_t newINodeNumber = 1; 												// Start at 1 because INode 0 should always be base directory
	uint8_t flag = 0;															// Used in the loop that finds the new file's INode number

	struct INode newFile;


	/* 1.) Set up INode structure for new file */
	newFile.fileType = fileType;
	newFile.fileSize = len;
	memset(newFile.addr, 0, 10 * sizeof(uint32_t));


	/* Check available memory segments, determine where to store file */
	/* to do this read all of the INodes and check their pointers and numbers */

	numINodes = readINodes(&INodes, &INodeNumbers);
	if (numINodes == 0) {
		free(INodes);
		free(INodeNumbers);
		return -10;
	}

	int i;
	int j;

	/* Find the lowest available INode Number and set write addr for INode */
	for (newINodeNumber = 1; newINodeNumber < MAX_INODES; newINodeNumber++) {		// Iterate from 1-max possible Inode number
		flag = 0;
		for (i = 0; i < numINodes; i++) {											// Check if its in the list of valid INodes from memory
			if (newINodeNumber == INodeNumbers[i]) {								// If it is, increment new INode number, try again
				flag = 1;
				break;
			}
		}
		if (flag == 0) {															// If the new number was not in the list exit the loops
			break;
		}
	}


	/* Use INode pointers to find open memory area */
	uint16_t numPointers = numINodes*5;
	uint32_t *startPointers = malloc(sizeof(uint32_t) * numPointers);
	uint32_t *endPointers = malloc(sizeof(uint32_t) * numPointers);

	// a.) Make lists of starting and ending pointers
	for (i = 0; i < numINodes; i++) {
		for (j = 0; j < 5; j++) {
			startPointers[i*5 +j] = INodes[i].addr[j*2];
			endPointers[i*5 + j] = INodes[i].addr[j*2 +1];
		}
	}

	// b.) Sort startPointers, make identical swaps to endPointers array
	sort(startPointers, endPointers, numPointers);

	// c.) Find empty memory segments between used ones
	uint32_t *startFreeMemory = malloc(sizeof(uint32_t) * numPointers);
	uint32_t *endFreeMemory = malloc(sizeof(uint32_t) * numPointers);
	memset(startFreeMemory, 0, numPointers * 4);
	memset(endFreeMemory, 0, numPointers * 4);
	uint16_t index = 0;


	// Check if there is space open at start of mem
	if (startPointers[0] > RESERVED_ADDR) {
	    startFreeMemory[index] = RESERVED_ADDR;
	    endFreeMemory[index] = startPointers[0] - 1;
	    index++;
	}

	// Check if there are open segments between data
	for (i = 0; i < numPointers; i++) {										// Iterate through pointer pairs
		if (startPointers[i] == 0) {										// If the start is 0 then its blank, skip
			continue;
		}

		// Calculate the available memory segment between two used segments
		// If end is less than start it means there is no space between segments
		if ((endPointers[i] + 1) < (startPointers[i + 1] - 1)) {
			startFreeMemory[index] = endPointers[i] + 1;
			endFreeMemory[index] = startPointers[i + 1] - 1;
			index++;
		}

	}

	// Check if there is space open at end of mem
	if (endPointers[numPointers - 1] < END_ADDR) {
		if (endPointers[numPointers - 1] == 0) {							// This shoudl indicate that nothing is in memory, so all of it is open
			startFreeMemory[index] = RESERVED_ADDR+1;
		}
		else {
			startFreeMemory[index] = endPointers[numPointers - 1] + 1;
		}

	    endFreeMemory[index] = END_ADDR;
	    index++;
	}



	/* Use open memory segments to decide where to save */
	// TODO URG : If there is not one segment big enough use multiple
	for (i = 0; i < index; i++) {											// Iterate through open memory segments
		if (endFreeMemory[i] - startFreeMemory[i] > newFile.fileSize) {		// If the segment is big enough to hold the file
			newFile.addr[0] = startFreeMemory[i];							// Store the file at the start of the open memory
			newFile.addr[1] = startFreeMemory[i] + len;
			break;
		}
	}

	if (i == index) {
		free(INodes);
		free(INodeNumbers);
		free(startPointers);
		free(endPointers);
		free(startFreeMemory);
		free(endFreeMemory);
		return -7;															// no segment big enough
	}



	/* Write INode to memory */
	if (writeINode(INodes, INodeNumbers, numINodes, newINodeNumber, newFile) == 0) {
		free(INodes);
		free(INodeNumbers);
		free(startPointers);
		free(endPointers);
		free(startFreeMemory);
		free(endFreeMemory);
		return -5;																	// Failed to write INodes
	}


	free(INodes);
	free(INodeNumbers);

	free(startFreeMemory);
	free(endFreeMemory);
	free(startPointers);
	free(endPointers);

	HAL_Delay(1);


	/* 2.) Determine where in memory to store the new file */
	/* Check available memory segments, determine where to store file */
	/* to do this read all of the INodes and check their pointers and numbers */
	INodes = malloc(0);
	INodeNumbers = malloc(0);

	numINodes = readINodes(&INodes, &INodeNumbers);


	/* TODO : GET INode number of current directory */



	/* Use INode pointers to find open memory area */
	numPointers = numINodes*5;

	startPointers = malloc(sizeof(uint32_t) * numPointers);
	endPointers = malloc(sizeof(uint32_t) * numPointers);

	startFreeMemory = malloc(sizeof(uint32_t) * numPointers);
	endFreeMemory = malloc(sizeof(uint32_t) * numPointers);

	memset(startFreeMemory, 0, numPointers * 4);
	memset(endFreeMemory, 0, numPointers * 4);


	// a.) Make lists of starting and ending pointers
	for (i = 0; i < numINodes; i++) {
		for (j = 0; j < 5; j++) {
			startPointers[i*5 +j] = INodes[i].addr[j*2];
			endPointers[i*5 + j] = INodes[i].addr[j*2 +1];
		}
	}

	// b.) Sort startPointers, make identical swaps to endPointers array
	sort(startPointers, endPointers, numPointers);

	// c.) Find empty memory segments between used ones
	memset(startFreeMemory, 0, numPointers * 4);
	memset(endFreeMemory, 0, numPointers * 4);
	index = 0;

	// Check if there is space open at start of mem
	if (startPointers[0] > RESERVED_ADDR) {
	    startFreeMemory[index] = RESERVED_ADDR;
	    endFreeMemory[index] = startPointers[0] - 1;
	    index++;
	}

	// Check if there are open segments between data
	for (i = 0; i < numPointers; i++) {								// Iterate through pointer pairs
		if (startPointers[i] == 0) {								// If the start is 0 then its blank, skip
			continue;
		}

		// Calculate the available memory segment between two used segments
		// If end is less than start it means there is no space between segments
		if ((endPointers[i] + 1) < (startPointers[i + 1] - 1) && (startPointers[i+1] - 1 < END_ADDR)) {
			startFreeMemory[index] = endPointers[i] + 1;
			endFreeMemory[index] = startPointers[i + 1] - 1;
			index++;
		}

	}

	// Check if there is space open at end of mem
	if (endPointers[numPointers - 1] < END_ADDR) {
		if (endPointers[numPointers - 1] == 0) {							// This shoudl indicate that nothing is in memory, so all of it is open
			startFreeMemory[index] = RESERVED_ADDR+1;
		}
		else {
			startFreeMemory[index] = endPointers[numPointers - 1] + 1;
		}

	    endFreeMemory[index] = END_ADDR;
	    index++;
	}

	/* 3.) Update the INode for the directory */
	/* Update base dir INode so that the new file can be appended */
	struct INode baseDir = INodes[0];								// base dir should always be 0
	baseDir.fileSize = baseDir.fileSize + NAME_LIM+1; 				// Add the size of the entry to the current size

	// Find end of data in pointers
	for (j = 9; j > 0; j -= 2) {
		if (baseDir.addr[j] == 0) {									// If the addr is empty go to next one
			continue;
		}
		else
			break;													// If addr is not empty it is end of data
	}

	if (j == -1) {													// If there are no pointers
		// J needs to be set to 0, do that in needNewPointer if
		needNewPointer = 1;
	}
	else {
		// If there is more room at end add on to the last pointer
		for (i = 0; i < index; i++) {
			// If there is room at the end of the last pointer add on
			if (((baseDir.addr[j] + NAME_LIM+1) > startFreeMemory[i]) &&
				((baseDir.addr[j] + NAME_LIM+1) < endFreeMemory[i])) {
				baseDir.addr[j] += NAME_LIM+1;										// Extend pointer
				needNewPointer = 0;													// Set flag so new pointers arent added
			}
		}
	}

	//TODO : TEST THIS SHIT.. Pretty sure it works
	if (needNewPointer) {														// If the last pointer could not be extended
		if (j == 9) {
			free(INodes);
			free(INodeNumbers);
			free(startFreeMemory);
			free(endFreeMemory);
			free(startPointers);
			free(endPointers);
			return -1;															// And we have used all 5 pointer pairs, we are too fragmented
		}
		j += 1;																	// Increment 1 to move from previous ending pointer to new starting pointer

		// TODO URG: If there is not one segment big enough use multiple
		for (i = 0; i < index; i++) {											// Iterate through open memory segments
			if (endFreeMemory[i] - startFreeMemory[i] > 5*baseDir.fileSize) {		// If the segment is big enough to hold the new data
				// Store the data in the middle of the open memory to reduce future fragmentation
				baseDir.addr[j]   = (endFreeMemory[i] - startFreeMemory[i]) / 2 + startFreeMemory[i];
				baseDir.addr[j+1] = baseDir.addr[j] + NAME_LIM+1;
				break;
			}
		}
	}


	/* rewrite the directory INode */
	if (writeINode(INodes, INodeNumbers, numINodes, 0, baseDir) == 0) {
		free(INodes);
		free(INodeNumbers);
		free(startFreeMemory);
		free(endFreeMemory);
		free(startPointers);
		free(endPointers);
		return -5;																	// Failed to write INodes
	}
	HAL_Delay(1);


	/* 4.) Write the file type and the name of the new file to the directory file */
	/* TODO: determine the current directory */

	uint8_t *directoryEntry = malloc(1+NAME_LIM);

	// If we made new pointers start from the new start pointer.
	// If we extened an old pointer, start from the end pointer minus len
	uint32_t dirAddr = needNewPointer ? baseDir.addr[j] : (baseDir.addr[j] - 1 - NAME_LIM);

	directoryEntry[0] = newINodeNumber;

	memcpy(directoryEntry+1, newFilename, NAME_LIM);


	if(writeData(directoryEntry, dirAddr, 1+NAME_LIM) == 0) {
		free(directoryEntry);
		free(INodes);
		free(INodeNumbers);
		free(startFreeMemory);
		free(endFreeMemory);
		free(startPointers);
		free(endPointers);
		return 0;
	}

	free(directoryEntry);
	free(INodes);
	free(INodeNumbers);
	free(startFreeMemory);
	free(endFreeMemory);
	free(startPointers);
	free(endPointers);


	HAL_Delay(1);


	/* 5.) Write the file to memory */
	// TODO : Update for case that multiple addr pairs are needed
	if (writeData(file, newFile.addr[0], len) == 0)
		return 0;


	HAL_Delay(1);


	return 1;

}


uint8_t deleteINode(uint8_t INodeNumber) {
	struct INode *INodes = malloc(0);
	uint8_t *INodeNumbers = malloc(0);
	uint8_t numINodes;
	uint8_t INodeIndex;
	int i;

	/* 1.) Read all INodes */
	numINodes = readINodes(&INodes, &INodeNumbers);


	// Check if the file to delete exists
	for (i = 0; i < numINodes; i++) {
		if (INodeNumber == INodeNumbers[i]) {
			break;
		}
	}
	// return if file does not exist
	if (i == numINodes) {
		free(INodeNumbers);
		free(INodes);
		return 0;
	}

	INodeIndex = i;

	/* 2.) Read current directory file and remove entry. save as array to write later (step 8) */
	// TODO : DO current dir instead of base dir
	uint8_t *file = readFile(INodes[0]);
	uint8_t numFiles = INodes[0].fileSize / (NAME_LIM+1);

	for (i = 0; i < numFiles; i++) {
		// find the directory entry that needs to be deleted.
		if (file[i * 64] == INodeNumber) {									// Each directory entry is 64 bytes (1 bytes num, 63 bytes str)
			// (start addr of entry remove, start of next entry, size)
			// size = 64 bytes * number of entries left
			memcpy(file + (i * (NAME_LIM+1)),
				   file + ((i + 1) * (NAME_LIM+1)),
				   (NAME_LIM+1)*(numFiles - 1 - i));
			// case 1
			// three files: 0, 1, 2
			// delete 1 - i = 0
			// size = 128 = 64*(numFiles-1) - 64*i = 64*2-0

			// case 2
			// three files: 0, 1, 2
			// delete 2 - i=1 when match
			// size 64 = 64*(numFiles-1) - 64*i = 64*2-64 = 64

			break;
		}

	}

	uint32_t newLength = (numINodes - 2) * 64;


	/* 3.) clear the file from memory */
	// Clear and rewrite each sector without the directory entries
	uint8_t *sector;
	uint32_t startPointer;
	uint32_t endPointer;
	uint32_t startSector;
	uint32_t endSector;



	for (i = 0; i < 5; i++) {
		startPointer = INodes[INodeIndex].addr[i*2];
		startSector = startPointer & 0x7F000;
		endPointer = INodes[INodeIndex].addr[i*2+1];
		endSector = endPointer & 0x7F000;

		if (startPointer == 0 || endPointer == 0)
			break;

		// Simple case - Clear data in one sector
		if (startSector == endSector) {
			sector = readData(startSector, 0xFFF);

			for (int j = (startPointer & 0xFFF); j <= (endPointer & 0xFFF); j++) {
				sector[j+4] = 0xFF;
			}

			sectorErase(startSector);
			writeData(sector+4, startSector, 0xFFF);
		}

		// Complex Case - Clear data that starts in one sector and ends in another
		else {
			sector = readData(startSector, 0xFFF);

			// Clear from start point to end pointer
			for (int j = (startPointer & 0xFFF); j <= 0xFFF; j++) {
				sector[j+4] = 0xFF;
			}

			sectorErase(startSector);
			writeData(sector+4, startSector, 0xFFF);

			free(sector);

			sector = readData(endSector, 0xFFF);

			// Clear from start point to end pointer
			for (int j = 0; j <= (endPointer & 0xFFF); j++) {
				sector[j+4] = 0xFF;
			}

			sectorErase(endSector);
			writeData(sector+4, endSector, (endPointer & 0xFFF) + 1);
		}

		free(sector);


	}
	sector = readData(RESERVED_ADDR, 526);

	free(sector);



	/* 4.) Delete file INode */
	struct INode node;
	node.fileSize = 0xFFFFFFFF;
	node.fileType = 0xFF;
	memset(node.addr, 0xFF, sizeof(uint32_t) * 10);

	// Write the INode as all 0xFF to delete it
	writeINode(INodes, INodeNumbers, numINodes, INodeNumber, node);


	/* 5.) Remove directory file from memory */
	for (i = 0; i < 5; i++) {
		// TODO : Make this current directory
		startPointer = INodes[0].addr[i*2];
		startSector = startPointer & 0x7F000;
		endPointer = INodes[0].addr[i*2+1];
		endSector = endPointer & 0x7F000;

		if (startPointer == 0 || endPointer == 0)
			break;

		// Simple case - Clear data in one sector
		if (startSector == endSector) {
			sector = readData(startSector, 0xFFF);

			for (int j = (startPointer & 0xFFF); j <= (endPointer & 0xFFF); j++) {
				sector[j+4] = 0xFF;
			}

			sectorErase(startSector);
			writeData(sector+4, startSector, 0xFFF);
		}

		// Complex Case - Clear data that starts in one sector and ends in another
		else {
			sector = readData(startSector, 0xFFF);

			// Clear from start point to end pointer
			for (int j = (startPointer & 0xFFF); j <= 0xFFF; j++) {
				sector[j+4] = 0xFF;
			}

			sectorErase(startSector);
			writeData(sector+4, startSector, 0xFFF);

			free(sector);

			sector = readData(endSector, 0xFFF);

			// Clear from start point to end pointer
			for (int j = 0; j <= (endPointer & 0xFFF); j++) {
				sector[j+4] = 0xFF;
			}

			sectorErase(endSector);
			writeData(sector+4, endSector, (endPointer & 0xFFF) + 1);
		}

		free(sector);
	}


	/* 6.) Write new base dir */
	node.fileSize = 0;
	node.fileType = 0;
	memset(node.addr, 0, sizeof(uint32_t) * 10);

	numINodes = readINodes(&INodes, &INodeNumbers);


	// set the base dir as all empty so we can find open memory
	INodes[0] = node;

	// a.) Find open memory to write directory to
	node.fileSize = newLength;

	uint8_t needNewPointer = 1;
	int j;
	uint16_t numPointers = numINodes*5;
	uint32_t *startFreeMemory = malloc(sizeof(uint32_t) * numPointers);
	uint32_t *endFreeMemory = malloc(sizeof(uint32_t) * numPointers);
	memset(startFreeMemory, 0, numPointers * 4);
	memset(endFreeMemory, 0, numPointers * 4);


	uint32_t *startPointers = malloc(sizeof(uint32_t) * numPointers);
	uint32_t *endPointers = malloc(sizeof(uint32_t) * numPointers);
	uint16_t index;

	// a.) Make lists of starting and ending pointers
	for (i = 0; i < numINodes; i++) {
		for (j = 0; j < 5; j++) {
			startPointers[i*5 +j] = INodes[i].addr[j*2];
			endPointers[i*5 + j] = INodes[i].addr[j*2 +1];
		}
	}

	// b.) Sort startPointers, make identical swaps to endPointers array
	sort(startPointers, endPointers, numPointers);

	// c.) Find empty memory segments between used ones
	memset(startFreeMemory, 0, numPointers * 4);
	memset(endFreeMemory, 0, numPointers * 4);
	index = 0;

	// Check if there is space open at start of mem
	if (startPointers[0] > RESERVED_ADDR) {
	    startFreeMemory[index] = RESERVED_ADDR;
	    endFreeMemory[index] = startPointers[0] - 1;
	    index++;
	}

	// Check if there are open segments between data
	for (i = 0; i < numPointers; i++) {								// Iterate through pointer pairs
		if (startPointers[i] == 0) {								// If the start is 0 then its blank, skip
			continue;
		}

		// Calculate the available memory segment between two used segments
		// If end is less than start it means there is no space between segments
		if ((endPointers[i] + 1) < (startPointers[i + 1] - 1) && (startPointers[i+1] - 1 < END_ADDR)) {
			startFreeMemory[index] = endPointers[i] + 1;
			endFreeMemory[index] = startPointers[i + 1] - 1;
			index++;
		}

	}

	// Check if there is space open at end of mem
	if (endPointers[numPointers - 1] < END_ADDR) {
		if (endPointers[numPointers - 1] == 0) {							// This shoudl indicate that nothing is in memory, so all of it is open
			startFreeMemory[index] = RESERVED_ADDR+1;
		}
		else {
			startFreeMemory[index] = endPointers[numPointers - 1] + 1;
		}

	    endFreeMemory[index] = END_ADDR;
	    index++;
	}

	// Find end of data in pointers
	for (j = 9; j > 0; j -= 2) {
		if (node.addr[j] == 0) {									// If the addr is empty go to next one
			continue;
		}
		else
			break;													// If addr is not empty it is end of data
	}

	if (j == -1) {													// If there are no pointers
		// J needs to be set to 0, do that in needNewPointer if
		needNewPointer = 1;
	}
	else {
		// If there is more room at end add on to the last pointer
		for (i = 0; i < index; i++) {
			// If there is room at the end of the last pointer add on
			if (((node.addr[j] + NAME_LIM+1) > startFreeMemory[i]) &&
				((node.addr[j] + NAME_LIM+1) < endFreeMemory[i])) {
				node.addr[j] += NAME_LIM+1;										// Extend pointer
				needNewPointer = 0;													// Set flag so new pointers arent added
			}
		}
	}

	//TODO : TEST THIS SHIT.. Pretty sure it works
	if (needNewPointer) {														// If the last pointer could not be extended
		if (j == 9) {
			free(INodes);
			free(INodeNumbers);
			free(startFreeMemory);
			free(endFreeMemory);
			free(startPointers);
			free(endPointers);
			return -1;															// And we have used all 5 pointer pairs, we are too fragmented
		}
		j += 1;																	// Increment 1 to move from previous ending pointer to new starting pointer

		// TODO URG: If there is not one segment big enough use multiple
		for (i = 0; i < index; i++) {											// Iterate through open memory segments
			if (endFreeMemory[i] - startFreeMemory[i] > node.fileSize) {		// If the segment is big enough to hold the new data
				if (node.fileSize > 0) {
					node.addr[j]   = startFreeMemory[i];							// Store the data at the start of the open memory
					node.addr[j+1] = startFreeMemory[i] + node.fileSize;
				}
				break;
			}
		}
	}



	/* 7.) Write directory INode to memory */
	if (writeINode(INodes, INodeNumbers, numINodes, 0, node) == 0) {
		free(INodes);
		free(INodeNumbers);
		free(startPointers);
		free(endPointers);
		free(startFreeMemory);
		free(endFreeMemory);
		return -5;																	// Failed to write INodes
	}


	/* 8.) rewrite directory to memory */
	/* TODO: determine the current directory */

	// ----- TODOD: DELETE ME-----
	uint8_t * file__ = readFile(node);
	free(file__);




	// If we made new pointers start from the new start pointer.
	// If we extened an old pointer, start from the end pointer minus len
	uint32_t dirAddr = needNewPointer ? node.addr[j] : (node.addr[j] - 1 - NAME_LIM);

	if (newLength != 0) {
		if(writeData(file, dirAddr, newLength) == 0) {
			free(INodes);
			free(INodeNumbers);
			free(startFreeMemory);
			free(endFreeMemory);
			free(startPointers);
			free(endPointers);
			return 0;
		}
	}


	free(INodes);
	free(INodeNumbers);

	free(startFreeMemory);
	free(endFreeMemory);
	free(startPointers);
	free(endPointers);
	free(file);


	// ----- TODOD: DELETE ME-----
	file__ = readFile(node);
	free(file__);


//
//	struct INode *INodes__ = malloc(0);
//	uint8_t *INodeNumbers__ = malloc(0);
//	uint8_t numINodes__;
//
//	numINodes__ = readINodes(&INodes__, &INodeNumbers__);

	return 1;
}


uint8_t writeINode(struct INode *INodes, uint8_t *INodeNumbers, uint8_t numINodes, uint8_t INodeNumber, struct INode node) {
	uint8_t *INodeArray = malloc(INODE_SIZE+4);
	int i;
	uint32_t INodeAddr = INODE_SIZE * INodeNumber;										// The addr is an offset from 0. multiply number by struct size in bytes


	// Check if the INode already exists
	for (i = 0; i < numINodes; i++) {
		if (INodeNumber == INodeNumbers[i]) {
			break;
		}
	}

	/* If this is a new node */
	if (i == numINodes) {
		structToArray(node, INodeArray);

		if (writeData(INodeArray, INodeAddr, INODE_SIZE) == 0) {
			free(INodeArray);
			return 0;
		}
	}

	/* If this is a modification of a current node */
	else {
		  if (sectorErase(0x000000) == 0)
			  return 0;												// erase INODES sect 1
		  if (sectorErase(0x001000) == 0)
			  return 0;	 											// erase INODES sect 2

		  for (i = 0; i < numINodes; i++) {							// Re write INodes

			  if (INodeNumbers[i] == INodeNumber) {					// If INode is same being modified
				  structToArray(node, INodeArray);					// Use new data not old data
			  }
			  else {
				  structToArray(INodes[i], INodeArray);
			  }

			  if (writeData(INodeArray, INODE_SIZE * INodeNumbers[i] , INODE_SIZE) == 0) {
				  free(INodeArray);	// If this happens you are fucked. Table of contents gone.
				  return 0;
			  }
		  }


	}

	free(INodeArray);
	return 1;
}

uint8_t getINode(uint8_t INodeNumber, struct INode *node) {
	struct INode temp;
	uint8_t *rawINodes = readData(INODE_SIZE * INodeNumber, INODE_SIZE);


	temp.fileType = rawINodes[4+0];

	temp.fileSize = (rawINodes[4+1] << 16)  | (rawINodes[4+2] << 8)  | (rawINodes[4+3]);

	temp.addr[0]  = (rawINodes[4+4] << 16)  | (rawINodes[4+5] << 8)  | (rawINodes[4+6]);
	temp.addr[1]  = (rawINodes[4+7] << 16)  | (rawINodes[4+8] << 8)  | (rawINodes[4+9]);

	temp.addr[2]  = (rawINodes[4+10] << 16) | (rawINodes[4+11] << 8) | (rawINodes[4+12]);
	temp.addr[3]  = (rawINodes[4+13] << 16) | (rawINodes[4+14] << 8) | (rawINodes[4+15]);

	temp.addr[4]  = (rawINodes[4+16] << 16) | (rawINodes[4+17] << 8) | (rawINodes[4+18]);
	temp.addr[5]  = (rawINodes[4+19] << 16) | (rawINodes[4+20] << 8) | (rawINodes[4+21]);

	temp.addr[6]  = (rawINodes[4+22] << 16) | (rawINodes[4+23] << 8) | (rawINodes[4+24]);
	temp.addr[7]  = (rawINodes[4+25] << 16) | (rawINodes[4+26] << 8) | (rawINodes[4+27]);

	temp.addr[8]  = (rawINodes[4+28] << 16) | (rawINodes[4+29] << 8) | (rawINodes[4+30]);
	temp.addr[9]  = (rawINodes[4+31] << 16) | (rawINodes[4+32] << 8) | (rawINodes[4+33]);


	if (checkINodeValidity(temp)) {
		*node = temp;											// Store node in array
		return 1;
	}

	free(rawINodes);
	return 0;
}



uint8_t readINodes(struct INode **INodes, uint8_t **INodeNumbers) {
	// INodes are in sectors 1 and 2 - 0x0000 - 0x1FFF
	uint8_t *rawINodes = readData(0x0000, RESERVED_ADDR);

	struct INode temp;
	uint16_t numValid = 0;

	for (int i = 4; i < RESERVED_ADDR+4; i += INODE_SIZE) {				// Start at 4 and add 4 to limit to ignore where instruction and addr were

		temp.fileType = rawINodes[i];

		temp.fileSize = (rawINodes[i+1] << 16) | (rawINodes[i+2] << 8) | (rawINodes[i+3]);

		temp.addr[0] = (rawINodes[i+4] << 16) | (rawINodes[i+5] << 8) | (rawINodes[i+6]);
		temp.addr[1] = (rawINodes[i+7] << 16) | (rawINodes[i+8] << 8) | (rawINodes[i+9]);

		temp.addr[2] = (rawINodes[i+10] << 16) | (rawINodes[i+11] << 8) | (rawINodes[i+12]);
		temp.addr[3] = (rawINodes[i+13] << 16) | (rawINodes[i+14] << 8) | (rawINodes[i+15]);

		temp.addr[4] = (rawINodes[i+16] << 16) | (rawINodes[i+17] << 8) | (rawINodes[i+18]);
		temp.addr[5] = (rawINodes[i+19] << 16) | (rawINodes[i+20] << 8) | (rawINodes[i+21]);

		temp.addr[6] = (rawINodes[i+22] << 16) | (rawINodes[i+23] << 8) | (rawINodes[i+24]);
		temp.addr[7] = (rawINodes[i+25] << 16) | (rawINodes[i+26] << 8) | (rawINodes[i+27]);

		temp.addr[8] = (rawINodes[i+28] << 16) | (rawINodes[i+29] << 8) | (rawINodes[i+30]);
		temp.addr[9] = (rawINodes[i+31] << 16) | (rawINodes[i+32] << 8) | (rawINodes[i+33]);


		if (checkINodeValidity(temp)) {
			numValid++;

			*INodes = realloc((*INodes), sizeof(struct INode) * numValid);					// Increase array size to hold valid node
			*INodeNumbers = realloc(*INodeNumbers, numValid);									// Increase array size to hold valid node

			(*INodes)[numValid-1] = temp;											// Store node in array
			(*INodeNumbers)[numValid-1] = i / INODE_SIZE;							// Store corresponding INode Number for valid INodes
		}

	}

	free(rawINodes);
	return numValid;
}

uint8_t checkINodeValidity(struct INode node) {
	// INodes are deemed invalid if the data within them is not set
	// The file size is compared to 3 bytes of FF
	// addrs compared to 3 bytes of FF


	if (node.fileSize == 0x00FFFFFF)						// Check if the size was set at all
		return 0;
	else if (node.fileSize > 504*1024)
		return 0;											// If size is greater than total storage

	if (node.fileType != 0 && node.fileType != 1)			// If the file type is invalid
		return 0;

	for (int i = 0; i < 10; i++)
		if (node.addr[i] == 0x00FFFFFF)
			return 0;										// If any of the pointers are unset
		else if (node.addr[i] > 504*1024 || (node.addr[i] < RESERVED_ADDR && node.addr[i] != 0))
			return 0;


	return 1;
}


void structToArray(struct INode node, uint8_t *arr) {
	// Make sure you pass an array that is 34 bytes long
	arr[0] = node.fileType;
	arr[1] = (node.fileSize & 0x00FF0000) >> 16;
	arr[2] = (node.fileSize & 0x0000FF00) >> 8;
	arr[3] = (node.fileSize & 0x000000FF);

	for (int i = 0; i < 10; i++) {
		arr[4+ 3*i]   = (node.addr[i] & 0x00FF0000) >> 16;
		arr[4+ 3*i+1] = (node.addr[i] & 0x0000FF00) >> 8;
		arr[4+ 3*i+2] =	 node.addr[i] & 0x000000FF;
	}
	return;
}


void swap(uint32_t* xp, uint32_t* yp) {
	uint32_t temp = *xp;
    *xp = *yp;
    *yp = temp;
}

// Function to perform Selection Sort
void sort(uint32_t *arr, uint32_t *pairedArr, int n) {
    int i, j, min_idx;

    // One by one move boundary of
    // unsorted subarray
    for (i = 0; i < n - 1; i++) {
        // Find the minimum element in
        // unsorted array
        min_idx = i;
        for (j = i + 1; j < n; j++)
            if (arr[j] < arr[min_idx])
                min_idx = j;

        // Swap the found minimum element
        // with the first element
        swap(&arr[min_idx], &arr[i]);
        swap(&pairedArr[min_idx], &pairedArr[i]);
    }
    return;
}

