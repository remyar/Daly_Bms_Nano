/*
DALY2MQTT Project
https://github.com/softwarecrash/DALY2MQTT
*/
#ifndef DALY_BMS_UART_H
#define DALY_BMS_UART_H
#include <Arduino.h>
#include <AltSoftSerial.h>

#define XFER_BUFFER_LENGTH 13
#define DELAYTINME 100

#define START_BYTE 0xA5;   // Start byte
#define HOST_ADRESS 0x40;  // Host address
#define FRAME_LENGTH 0x08; // Length

#define ERRORCOUNTER 10 // number of try befor clear data

#define BMS_DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
#define BMS_DEBUG_PRINT(...) Serial.print(__VA_ARGS__)

class DalyBms
{
public:
    unsigned long previousTime = 0;
    uint8_t requestCounter = 0;

    enum COMMAND
    {
        CELL_THRESHOLDS = 0x59,
        PACK_THRESHOLDS = 0x5A,
        VOUT_IOUT_SOC = 0x90,
        MIN_MAX_CELL_VOLTAGE = 0x91,
        MIN_MAX_TEMPERATURE = 0x92,
        DISCHARGE_CHARGE_MOS_STATUS = 0x93,
        STATUS_INFO = 0x94,
        CELL_VOLTAGES = 0x95,
        CELL_TEMPERATURE = 0x96,
        CELL_BALANCE_STATE = 0x97,
        FAILURE_CODES = 0x98,
        DISCHRG_FET = 0xD9,
        CHRG_FET = 0xDA,
        BMS_RESET = 0x00,
        READ_SOC = 0x61, // read the time and soc
        SET_SOC = 0x21,  // set the time and soc
        // END = 0xD8,
        // after request the pc soft hangs a 0xD8 as last request, its empty, dont know what it means?
    };

    /**
     * @brief get struct holds all the data collected from the BMS and is populated using the update() API
     */
    struct
    {
        // data from 0x90
        float packVoltage; // pressure (0.1 V)
        float packCurrent; // acquisition (0.1 V)
        float packSOC;     // State Of Charge

        // data from 0x91
        float maxCellmV; // maximum monomer voltage (mV)
        int maxCellVNum; // Maximum Unit Voltage cell No.
        float minCellmV; // minimum monomer voltage (mV)
        int minCellVNum; // Minimum Unit Voltage cell No.
        int cellDiff;    // difference betwen cells

        // data from 0x92
        int tempAverage; // Avergae Temperature

        // data from 0x93
        const char *chargeDischargeStatus; // charge/discharge status (0 stationary ,1 charge ,2 discharge)
        bool chargeFetState;               // charging MOS tube status
        bool disChargeFetState;            // discharge MOS tube state
        int bmsHeartBeat;                  // BMS life(0~255 cycles)
        float resCapacityAh;               // residual capacity mAH

        // get a state of the connection
        bool connectionState;
    } get;

    /**
     * @brief Construct a new DalyBms object
     *
     * @param serialIntf UART interface BMS is connected to
     */
    DalyBms();

    /**
     * @brief Initializes this driver
     * @details Configures the serial peripheral and pre-loads the transmit buffer with command-independent bytes
     */
    bool Init();
    /**
     * @brief put it in lopp
     *
     */
    bool loop();

    /**
     * @brief Gets Voltage, Current, and SOC measurements from the BMS
     * @return True on successful aquisition, false otherwise
     */
    bool getPackMeasurements();

    /**
     * @brief Returns the highest and lowest individual cell voltage, and which cell is highest/lowest
     * @details Voltages are returned as floats with milliVolt precision (3 decimal places)
     * @return True on successful aquisition, false otherwise
     */
    bool getMinMaxCellVoltage();

    /**
     * @brief Gets the pack temperature from the min and max of all the available temperature sensors
     * @details Populates tempMax, tempMax, and tempAverage in the "get" struct
     * @return True on successful aquisition, false otherwise
     */
    bool getPackTemp();
    
private:
    unsigned int errorCounter = 0;

    /**
     * @brief send the command id, and return true if data complete read or false by crc error
     * @details calculates the checksum and sends the command over the specified serial connection
     */
    bool requestData(COMMAND cmdID, unsigned int frameAmount);

    /**
     * @brief Clear all data from the Get struct
     * @details when wrong or missing data comes in it need sto be cleared
     */
    void clearGet();

    /**
     * @brief Serial interface used for communication
     * @details This is set in the constructor
     */
    AltSoftSerial *my_serialIntf;

    /**
     * @brief Buffer used to transmit data to the BMS
     * @details Populated primarily in the "Init()" function, see the readme for more info
     */
    uint8_t my_txBuffer[XFER_BUFFER_LENGTH];
    uint8_t my_rxFrameBuffer[XFER_BUFFER_LENGTH * 12];
    uint8_t frameBuff[12][XFER_BUFFER_LENGTH];
    unsigned int frameCount;
};
#endif // DALY_BMS_UART_H