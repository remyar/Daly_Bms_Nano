/*
DALY2MQTT Project
https://github.com/softwarecrash/DALY2MQTT
*/
#include "daly.h"

// software serial #1: TX = digital pin 10, RX = digital pin 11
AltSoftSerial myPort;

//----------------------------------------------------------------------
// Public Functions
//----------------------------------------------------------------------

DalyBms::DalyBms()
{
    this->my_serialIntf = &myPort;
}

bool DalyBms::Init()
{
    // Null check the serial interface
    if (this->my_serialIntf == NULL)
    {
        BMS_DEBUG_PRINTLN("<DALY-BMS DEBUG> ERROR: No serial peripheral specificed!");
        get.connectionState = -3;
        return false;
    }

    // Initialize the serial link to 9600 baud with 8 data bits and no parity bits, per the Daly BMS spec
    this->my_serialIntf->begin(9600);
    memset(this->my_txBuffer, 0x00, XFER_BUFFER_LENGTH);

    clearGet();

    return true;
}

bool DalyBms::loop()
{
    if (millis() - previousTime >= DELAYTINME)
    {
        switch (requestCounter)
        {
        case 0:
            // requestCounter = sendCommand() ? (requestCounter + 1) : 0;
            requestCounter++;
            break;
        case 1:
            if (getPackMeasurements())
            {
                get.connectionState = true;
                errorCounter = 0;
                requestCounter++;
            }
            else
            {
                requestCounter = 0;
                if (errorCounter < ERRORCOUNTER)
                {
                    errorCounter++;
                }
                else
                {
                    get.connectionState = false;
                    errorCounter = 0;
                    // requestCallback();
                    // clearGet();
                }
            }
            break;
        case 2:
            requestCounter = getMinMaxCellVoltage() ? (requestCounter + 1) : 0;
            break;
        case 3:
            requestCounter = getPackTemp() ? (requestCounter + 1) : 0;
            break;
        default:
            break;
        }
        previousTime = millis();
    }
    return true;
}

void DalyBms::clearGet(void)
{
    get.chargeDischargeStatus = "offline"; // charge/discharge status (0 stationary ,1 charge ,2 discharge)
}

bool DalyBms::getPackMeasurements() // 0x90
{
    if (!this->requestData(COMMAND::VOUT_IOUT_SOC, 1))
    {
        BMS_DEBUG_PRINTLN("<DALY-BMS DEBUG> Receive failed, V, I, & SOC values won't be modified!\n");
        clearGet();
        return false;
    }
    else
    {
        // check if packCurrent in range
        if (((float)(((this->frameBuff[0][8] << 8) | this->frameBuff[0][9]) - 30000) / 10.0f) == -3000.f)
        {
            BMS_DEBUG_PRINTLN("<DALY-BMS DEBUG> Receive failed, pack Current not in range. values won't be modified!\n");
            return false;
        }
        else
            // check if SOC in range
            if (((float)((this->frameBuff[0][10] << 8) | this->frameBuff[0][11]) / 10.0f) > 100.f)
            {
                BMS_DEBUG_PRINTLN("<DALY-BMS DEBUG> Receive failed,SOC out of range. values won't be modified!\n");
                return false;
            }
    }

    // Pull the relevent values out of the buffer
    get.packVoltage = ((float)((this->frameBuff[0][4] << 8) | this->frameBuff[0][5]) / 10.0f);
    get.packCurrent = ((float)(((this->frameBuff[0][8] << 8) | this->frameBuff[0][9]) - 30000) / 10.0f);
    get.packSOC = ((float)((this->frameBuff[0][10] << 8) | this->frameBuff[0][11]) / 10.0f);
    // BMS_DEBUG_PRINTLN("<DALY-BMS DEBUG> " + (String)get.packVoltage + "V, " + (String)get.packCurrent + "A, " + (String)get.packSOC + "SOC");
    // BMS_DEBUG_WEBLN("<DALY-BMS DEBUG> " + (String)get.packVoltage + "V, " + (String)get.packCurrent + "A, " + (String)get.packSOC + "SOC");
    return true;
}

bool DalyBms::getMinMaxCellVoltage() // 0x91
{
    if (!this->requestData(COMMAND::MIN_MAX_CELL_VOLTAGE, 1))
    {
        BMS_DEBUG_PRINT("<DALY-BMS DEBUG> Receive failed, min/max cell values won't be modified!\n");
        return false;
    }

    get.maxCellmV = (float)((this->frameBuff[0][4] << 8) | this->frameBuff[0][5]);
    get.maxCellVNum = this->frameBuff[0][6];
    get.minCellmV = (float)((this->frameBuff[0][7] << 8) | this->frameBuff[0][8]);
    get.minCellVNum = this->frameBuff[0][9];
    get.cellDiff = (get.maxCellmV - get.minCellmV);

    return true;
}

bool DalyBms::getPackTemp() // 0x92
{
    if (!this->requestData(COMMAND::MIN_MAX_TEMPERATURE, 1))
    {
        BMS_DEBUG_PRINT("<DALY-BMS DEBUG> Receive failed, Temp values won't be modified!\n");
        return false;
    }
    get.tempAverage = ((this->frameBuff[0][4] - 40) + (this->frameBuff[0][6] - 40)) / 2;

    return true;
}


//----------------------------------------------------------------------
// Private Functions
//----------------------------------------------------------------------

bool DalyBms::requestData(COMMAND cmdID, unsigned int frameAmount) // new function to request global data
{
    // Clear out the buffers
    memset(this->my_rxFrameBuffer, 0x00, sizeof(this->my_rxFrameBuffer));
    memset(this->frameBuff, 0x00, sizeof(this->frameBuff));
    memset(this->my_txBuffer, 0x00, XFER_BUFFER_LENGTH);

    //--------------send part--------------------
    uint8_t txChecksum = 0x00;    // transmit checksum buffer
    unsigned int byteCounter = 0; // bytecounter for incomming data
    // prepare the frame with static data and command ID
    this->my_txBuffer[0] = START_BYTE;
    this->my_txBuffer[1] = HOST_ADRESS;
    this->my_txBuffer[2] = cmdID;
    this->my_txBuffer[3] = FRAME_LENGTH;

    // Calculate the checksum
    for (uint8_t i = 0; i <= 11; i++)
    {
        txChecksum += this->my_txBuffer[i];
    }
    // put it on the frame
    this->my_txBuffer[12] = txChecksum;

    // send the packet
    this->my_serialIntf->write(this->my_txBuffer, XFER_BUFFER_LENGTH);

    // first wait for transmission end
    this->my_serialIntf->flush();
    //-------------------------------------------

    //-----------Recive Part---------------------
    /*uint8_t rxByteNum = */ this->my_serialIntf->readBytes(this->my_rxFrameBuffer, XFER_BUFFER_LENGTH * frameAmount);
    for (size_t i = 0; i < frameAmount; i++)
    {
        for (size_t j = 0; j < XFER_BUFFER_LENGTH; j++)
        {
            this->frameBuff[i][j] = this->my_rxFrameBuffer[byteCounter];
            byteCounter++;
        }

        uint8_t rxChecksum = 0x00;
        for (int k = 0; k < XFER_BUFFER_LENGTH - 1; k++)
        {
            rxChecksum += this->frameBuff[i][k];
        }
        char debugBuff[128];
        sprintf(debugBuff, "<UART>[Command: 0x%2X][CRC Rec: %2X][CRC Calc: %2X]", cmdID, rxChecksum, this->frameBuff[i][XFER_BUFFER_LENGTH - 1]);
        BMS_DEBUG_PRINTLN(debugBuff);

        if (rxChecksum != this->frameBuff[i][XFER_BUFFER_LENGTH - 1])
        {
            BMS_DEBUG_PRINTLN("<UART> CRC FAIL");
            return false;
        }
        if (rxChecksum == 0)
        {
            BMS_DEBUG_PRINTLN("<UART> NO DATA");
            return false;
        }
        if (this->frameBuff[i][1] >= 0x20)
        {
            BMS_DEBUG_PRINTLN("<UART> BMS SLEEPING");
            return false;
        }
    }
    return true;
}
