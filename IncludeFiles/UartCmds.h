#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"

#define MAX_CHARS 80
#define MAX_FIELDS 10

typedef struct _USER_DATA {

    char buffer[MAX_CHARS + 1];
    char fieldType[MAX_FIELDS]; //array to store the char type
    uint8_t fieldCounter;
    uint8_t fieldPosition[MAX_FIELDS];


} USER_DATA;


void parseFields(USER_DATA* data);
char* getFieldString(USER_DATA* data, uint8_t fieldNumber);
int32_t getFieldInteger(USER_DATA* data, uint8_t fieldNumber);
bool isCommand(USER_DATA* data, const char strCommand[], uint8_t minArguments);
void getsUart0(USER_DATA* userData);
double getFieldFloat(USER_DATA* data, uint8_t fieldNumber);
