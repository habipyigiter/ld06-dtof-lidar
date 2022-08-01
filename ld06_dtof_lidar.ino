//  Habip Yiğiter Temmuz 2022 Ankara
//  LD06 Lidar Driver v0.2
//  ChangeLog
//  v0.2 Added CRC8 checksum and Intensity value control
//  V0.1 first version
//
//   -----------------------------------------------------------------------------------------------------------
//  |   Header | VerLen | Speed  | Start angle |  Data   | End angle | Timestamp |  CRC check                   |
//  |   1 Byte | 1 Byte | 2 Byte |   2 Byte    | 36 Byte |  2 Byte   |  2 Byte   |   1 Byte   = total 47 byte   |
//  |      0   |    1   |  2,3   |     4,5     |  6-41   |   42,43   |   44,45   |     46                       |
//   -----------------------------------------------------------------------------------------------------------

uint8_t CrcTable[256] =
    {
        0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3,
        0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33,
        0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8,
        0xf5, 0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77,
        0x3a, 0x94, 0xd9, 0x0e, 0x43, 0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55,
        0x18, 0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4,
        0xe9, 0x47, 0x0a, 0xdd, 0x90, 0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f,
        0x62, 0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff,
        0xb2, 0x1c, 0x51, 0x86, 0xcb, 0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2,
        0x8f, 0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12,
        0x5f, 0xf1, 0xbc, 0x6b, 0x26, 0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99,
        0xd4, 0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14,
        0x59, 0xf7, 0xba, 0x6d, 0x20, 0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36,
        0x7b, 0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9,
        0xb4, 0x1a, 0x57, 0x80, 0xcd, 0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72,
        0x3f, 0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2,
        0xef, 0x41, 0x0c, 0xdb, 0x96, 0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1,
        0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71,
        0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa,
        0xb7, 0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35,
        0x78, 0xd6, 0x9b, 0x4c, 0x01, 0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17,
        0x5a, 0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8};

int checksumValue = 0;
unsigned char readBuffer[47]; // 1 frame de  47 karakter var
int indexRxBuffer = 0;
bool dataReady = false;
int frameCounter = 0;

int ld06_distance_mm[360];

#define INTENSITY_VALUE 180
int intensity[360];

#define LENGTHALERTMAP sizeof(alertMap) / sizeof(int) / 2
// alertMap[angle][distance mm]

int alertMap[5][2] = {
    {358, 500}, //358 degree 50cm
    {359, 500},
    {0, 500},
    {1, 500},
    {2, 500}};

void setup()
{
    Serial2.begin(230400);
    Serial.begin(115200);
    Initialize_Array();
    pinMode(19, OUTPUT);
}

void loop()
{

    if (dataReady == true)
    {
        // USER CODE BEGIN/

          Serial.print("0 derece ");
          Serial.print(ld06_distance_mm[0]);  // 0 degree distance mm
          Serial.print(" mm Intensity: ");
          Serial.println(intensity[0]); 

        if (ScanAlertMap(&alertMap[0][0], LENGTHALERTMAP))
        {
            digitalWrite(19, HIGH);
        }
        else
        {
            digitalWrite(19, LOW);
        }

        // END USER CODE/

        dataReady = false;
        Initialize_Array();
    }
}

void serialEvent2()
{
    while (Serial2.available())
    {
        static unsigned char pre_inChar = 0;
        unsigned char inChar;

        inChar = (unsigned char)Serial2.read();

        if (dataReady == false)
        {

            if (inChar == 0x2c && pre_inChar == 0x54) //  2 byte header detect
            {
                readBuffer[0] == 0x54;
                indexRxBuffer = 1;
            }

            pre_inChar = inChar;

            readBuffer[indexRxBuffer] = inChar;
            indexRxBuffer++;

            if (indexRxBuffer >= 47)
            {
                // 1 DATA PAKETI 47 byte
                indexRxBuffer = 0;
                checksumValue = CalCRC8(readBuffer, 46);
                if (checksumValue == readBuffer[46])
                {
                    CalculateArray();
                    frameCounter++;
                }
            }

            if (frameCounter > 41)
            { // 1 turda ~41 frame
                frameCounter = 0;
                dataReady = true;
            }
        }
    }
}

void Initialize_Array()
{
    for (int row = 0; row < 360; row++)
    {
        ld06_distance_mm[row] = -1;
        intensity[row] = -1;
    }
}

void CalculateArray()
{
    if (readBuffer[0] == 0x54 && readBuffer[1] == 0x2C)
    {

        unsigned int startAngle = readBuffer[4] + (readBuffer[5] << 8);
        unsigned int endAngle = readBuffer[42] + (readBuffer[43] << 8);
        unsigned int distance_mm[12];
        unsigned int confidence[12]; // intensity
        float stepAngle;

        for (int i = 0; i < 12; i++)
        {
            distance_mm[i] = readBuffer[i * 3 + 6] + (readBuffer[i * 3 + 7] << 8);
            confidence[i] = readBuffer[i * 3 + 8];
        }

        if (startAngle < endAngle)
        {
            stepAngle = (endAngle - startAngle) / 11;
        }
        else
        {
            stepAngle = ((36000 - startAngle) + endAngle) / 11;
        }

        for (int i = 0; i < 12; i++)
        {

            unsigned int angle = 0;

            angle = (startAngle + ((float)stepAngle * i)) / 100; // 100 value unit conversion

            if (angle >= 360)
            {
                angle = angle - 360;
            }

            ld06_distance_mm[angle] = distance_mm[i];
            intensity[angle] = confidence[i];
        }
    }
}

int ScanAlertMap(int *arr, unsigned int lengthAlertMap)
{

    int alertFlag = 0;
    for (int a = 0; a < 360; a++)
    {
        for (int m = 0; m < lengthAlertMap; m++)
        {
            if (arr[m * 2] == a)
            {
                if (arr[m * 2 + 1] > ld06_distance_mm[a] && ld06_distance_mm[a] > 0 && intensity[a] >= INTENSITY_VALUE)
                {
                    alertFlag = 1;
                }
            }
        }
    }
    return alertFlag;
}

uint8_t CalCRC8(uint8_t *p, uint8_t len)
{
    uint8_t crc = 0;
    uint16_t i;
    for (i = 0; i < len; i++)
    {
        crc = CrcTable[(crc ^ *p++) & 0xff];
    }
    return crc;
}
