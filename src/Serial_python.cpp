#include "Serial_python.h"
#include "constants.h"

const int MAX_DATA_LENGTH = 10; // Kích thước tối đa của dữ liệu nhận được
float Position_X = 600;
float Position_Y = 0;
float Position_Z = 150;
int EnableRUN = 0;
float offset[3] = {-85, 270, 90};

bool ReceiveSerial()
{
    float nums[3];
    if (Serial.available() > 0)
    {
        // digitalWrite(Knift, HIGH);
        char input[50] = "0 0 0";
        // Read value until \n
        Serial.readBytesUntil('\n', input, sizeof(input));
        // Take the first token
        char *token = strtok(input, " ");
        int index = 0;

        while (token != NULL && index < 4)
        {
            nums[index] = atoi(token);
            token = strtok(NULL, " ");
            index++;
        }

        Position_X = nums[2] + offset[2];        // = 400
        Position_Y = (-1 * nums[0]) + offset[0]; //= 0
        Position_Z = 10; //=0

        EnableRUN = nums[3];
        // return true;
        if ((Position_X >= MinX) && (Position_X <= MaxX) && (Position_Y >= MinY) && (Position_Y <= MaxY) && (Position_Z >= MinZ) && (Position_Z <= MaxZ))
            return true;
        else
            return false;
    }
    else
    {
        return false;
        EnableRUN = 0;
    }
}
