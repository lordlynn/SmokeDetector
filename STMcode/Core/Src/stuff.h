#include "main.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"

#define SPI_TIMEOUT 1000 					// WHAT SHOULD THIS BE?? WHAT ARE UNITS??
#define NAME_LIM 63							// filenames are limited to 63 characters
#define INODE_SIZE 34						// Number of bytes to store INode
#define MAX_INODES 0x1FFF / 32				// Size of reserved memory divided by size of struct

#define READ_STATUS_REG 0x05
#define WRITE_ENABLE    0x06
#define WRITE_DISABLE   0x04
#define PAGE_PROGRAM    0x02
#define READ_DATA       0x03
#define SECTOR_ERASE 	0x20
#define CHIP_ERASE 		0x60


#define RESERVED_ADDR 0x1FFF				// From 0 - 4095 are reserved for INodes
#define END_ADDR 0x00080000					// Last addressable byte is 0x7FFFF
#define TOTAL_MEM END_ADDR - RESERVED_ADDR

// From Main file
extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern uint8_t UARTFlag;

struct INode{
	uint8_t fileType;						// 0 - dir, 1 - txt
	uint32_t fileSize;						// size in bytes
	uint32_t addr[10];						// Only need 3 bytes for addr
};


enum states{
	idle,
	store,
	dir,
	mem,
	del,
	read,
	clear,
};




void swap(uint32_t* xp, uint32_t* yp);
void sort(uint32_t *arr, uint32_t *pairedArr, int n);


void getDir();


void swapEndian(uint8_t data[], uint16_t len);
uint8_t writeEnable();

uint8_t sectorErase(uint32_t addr);
uint8_t chipErase(void);

uint8_t getINode(uint8_t INodeNumber, struct INode *node);
uint8_t readINodes(struct INode **INodes, uint8_t **INodeNumbers);
uint8_t checkINodeValidity(struct INode node);

void structToArray(struct INode node, uint8_t *arr);

void profileMemory(void);

uint8_t deleteINode(uint8_t INodeNumber);
uint8_t writeINode(struct INode *INodes, uint8_t *INodeNumbers, uint8_t numINodes, uint8_t INodeNumber, struct INode node);

uint8_t SetupFilesystem();
uint8_t writeFile(uint8_t *file, uint32_t len, char newFilename[NAME_LIM], uint8_t fileType);

uint8_t* readData(uint32_t startAddr, uint32_t bytesToRead);
uint8_t writeData(uint8_t *data, uint32_t startAddr, uint32_t bytesToWrite);

uint8_t* readFile(struct INode node);



