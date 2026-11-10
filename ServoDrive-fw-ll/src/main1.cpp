#include <cstring>
#include "common_inc.h"
#include "configurations.h"

Motor motor;
BoardConfig_t boardConfig;


/* Default Entry -------------------------------------------------------*/
void Main()
{
    // Read data from Flash
    EEPROM eeprom;
    eeprom.get(0, boardConfig);
    if (boardConfig.configStatus != CONFIG_OK) // use default settings
    {
        boardConfig = BoardConfig_t{
            .configStatus = CONFIG_OK,
            .nodeId = 12, // 7bit address, has to be even number
            .initPos = 90,
            .toqueLimit = 0.90f, // 90% torque capability to overcome SG90 gear friction
            .velocityLimit = 0,
            .adcValAtAngleMin = 140,  // Clean track start
            .adcValAtAngleMax = 2850, // Clean track end before 3000 breakdown
            .mechanicalAngleMin = 0,
            .mechanicalAngleMax = 180,
            .dceKp = 35.0f,
            .dceKv = 0,
            .dceKi = 0.20f,
            .dceKd = 40.0f,
            .enableMotorOnBoot = false,
            .invertDirection = true
        };
        eeprom.put(0, boardConfig);
    }

    // Safety validation for values loaded from flash (prevents uninitialized flash corruption)
    if (boardConfig.adcValAtAngleMin < 50 || boardConfig.adcValAtAngleMin > 1000)
        boardConfig.adcValAtAngleMin = 140;
    if (boardConfig.adcValAtAngleMax < 2000 || boardConfig.adcValAtAngleMax > 3800)
        boardConfig.adcValAtAngleMax = 2850;
    if (boardConfig.toqueLimit <= 0.10f || boardConfig.toqueLimit > 1.0f)
        boardConfig.toqueLimit = 0.90f;
    if (boardConfig.dceKp < 5.0f || boardConfig.dceKp > 150.0f)
        boardConfig.dceKp = 35.0f;
    if (boardConfig.dceKi < 0.0f || boardConfig.dceKi > 10.0f)
        boardConfig.dceKi = 0.20f;
    if (boardConfig.dceKd < 5.0f || boardConfig.dceKd > 150.0f)
        boardConfig.dceKd = 40.0f;
    if (boardConfig.mechanicalAngleMax < 90.0f || boardConfig.mechanicalAngleMax > 360.0f)
        boardConfig.mechanicalAngleMax = 180.0f;

    motor.invertDirection = boardConfig.invertDirection;
    motor.SetTorqueLimit(boardConfig.toqueLimit);
    motor.mechanicalAngleMin = boardConfig.mechanicalAngleMin;
    motor.mechanicalAngleMax = boardConfig.mechanicalAngleMax;
    motor.adcValAtAngleMin = boardConfig.adcValAtAngleMin;
    motor.adcValAtAngleMax = boardConfig.adcValAtAngleMax;
    motor.dce.kp = boardConfig.dceKp;
    motor.dce.ki = boardConfig.dceKi;
    motor.dce.kv = boardConfig.dceKv;
    motor.dce.kd = boardConfig.dceKd;
    motor.dce.setPointPos = boardConfig.initPos;
    motor.SetEnable(boardConfig.enableMotorOnBoot);
    
    // Initialize default I2C response buffer
    i2cDataTx[0] = 0x11; // GET_ANGLE
    memset(i2cDataTx + 1, 0, 4);

    // Init PWM
    LL_TIM_EnableCounter(TIM3);
    LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1);
    LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH2);
    LL_TIM_OC_SetCompareCH1(TIM3, 0);
    LL_TIM_OC_SetCompareCH2(TIM3, 0);

    // Start receive data
    MY_I2C1_Init(boardConfig.nodeId);
    LL_mDelay(10);
    
    // Start control loop at 200Hz
    LL_TIM_EnableIT_UPDATE(TIM14);
    LL_TIM_EnableCounter(TIM14);

    uint32_t commitDeadline = 0;
    bool commitPending = false;

    while (1)
    {
        if (boardConfig.configStatus == CONFIG_COMMIT)
        {
            boardConfig.configStatus = CONFIG_OK;
            commitDeadline = HAL_GetTick() + 300; // debounce 300ms so bursts of commands write flash once
            commitPending = true;
        }

        if (commitPending && (HAL_GetTick() >= commitDeadline))
        {
            commitPending = false;
            eeprom.put(0, boardConfig);
        }
        else if (boardConfig.configStatus == CONFIG_RESTORE)
        {
            eeprom.put(0, boardConfig);
            HAL_NVIC_SystemReset();
        }
    }
}


/* Callbacks -------------------------------------------------------*/

// Command handler
void I2C_SlaveDMARxCpltCallback()
{
    float valF = *((float*) (i2cDataRx + 1));

    i2cDataTx[0] = i2cDataRx[0];
    switch (i2cDataRx[0])
    {
        case 0x01:  // Set angle
        {
            if (valF < 5.0f) valF = 5.0f;
            if (valF > (motor.mechanicalAngleMax - 5.0f)) valF = motor.mechanicalAngleMax - 5.0f;
            motor.dce.setPointPos = valF;
            motor.dce.integralPos = 0.0f;
            auto* b = (unsigned char*) &(motor.angle);
            for (int i = 0; i < 4; i++)
                i2cDataTx[i + 1] = *(b + i);
            break;
        }
        case 0x02:  // Set velocity
        {
            motor.dce.setPointVel = valF;
            auto* b = (unsigned char*) &(motor.velocity);
            for (int i = 0; i < 4; i++)
                i2cDataTx[i + 1] = *(b + i);
            break;
        }
        case 0x03:  // Set torque
        {
            motor.SetTorqueLimit(valF);
            auto* b = (unsigned char*) &(motor.dce.output);
            for (int i = 0; i < 4; i++)
                i2cDataTx[i + 1] = *(b + i);
            break;
        }
        case 0x11:  // Read angle
        {
            auto* b = (unsigned char*) &(motor.angle);
            for (int i = 0; i < 4; i++)
                i2cDataTx[i + 1] = *(b + i);
            break;
        }
        case 0x12:  // Read velocity
        {
            auto* b = (unsigned char*) &(motor.velocity);
            for (int i = 0; i < 4; i++)
                i2cDataTx[i + 1] = *(b + i);
            break;
        }
        case 0x13:  // Read Raw ADC
        {
            float rawAdc = (float) adcData[0];
            auto* b = (unsigned char*) &rawAdc;
            for (int i = 0; i < 4; i++)
                i2cDataTx[i + 1] = *(b + i);
            break;
        }
        case 0x14:  // Read Invert Direction
        {
            float inv = motor.invertDirection ? 1.0f : 0.0f;
            auto* b = (unsigned char*) &inv;
            for (int i = 0; i < 4; i++)
                i2cDataTx[i + 1] = *(b + i);
            break;
        }
        case 0x15:  // Read Mechanical Max Angle (180 or 360)
        {
            auto* b = (unsigned char*) &(motor.mechanicalAngleMax);
            for (int i = 0; i < 4; i++)
                i2cDataTx[i + 1] = *(b + i);
            break;
        }
        case 0x16:  // Read Kp
        {
            auto* b = (unsigned char*) &(motor.dce.kp);
            for (int i = 0; i < 4; i++)
                i2cDataTx[i + 1] = *(b + i);
            break;
        }
        case 0x17:  // Read Ki
        {
            auto* b = (unsigned char*) &(motor.dce.ki);
            for (int i = 0; i < 4; i++)
                i2cDataTx[i + 1] = *(b + i);
            break;
        }
        case 0x18:  // Read Kv
        {
            auto* b = (unsigned char*) &(motor.dce.kv);
            for (int i = 0; i < 4; i++)
                i2cDataTx[i + 1] = *(b + i);
            break;
        }
        case 0x19:  // Read Kd
        {
            auto* b = (unsigned char*) &(motor.dce.kd);
            for (int i = 0; i < 4; i++)
                i2cDataTx[i + 1] = *(b + i);
            break;
        }
        case 0x21: // Set node ID
        {
            boardConfig.nodeId = (uint8_t) valF;
            boardConfig.configStatus = CONFIG_COMMIT;
            auto* b = (unsigned char*) &(motor.angle);
            for (int i = 0; i < 4; i++)
                i2cDataTx[i + 1] = *(b + i);
            break;
        }
        case 0x22: // Set Kp
        {
            motor.dce.kp = valF;
            boardConfig.dceKp = valF;
            boardConfig.configStatus = CONFIG_COMMIT;
            auto* b = (unsigned char*) &(motor.angle);
            for (int i = 0; i < 4; i++)
                i2cDataTx[i + 1] = *(b + i);
            break;
        }
        case 0x23: // Set Ki
        {
            motor.dce.ki = valF;
            boardConfig.dceKi = valF;
            boardConfig.configStatus = CONFIG_COMMIT;
            auto* b = (unsigned char*) &(motor.angle);
            for (int i = 0; i < 4; i++)
                i2cDataTx[i + 1] = *(b + i);
            break;
        }
        case 0x24: // Set Kv
        {
            motor.dce.kv = valF;
            boardConfig.dceKv = valF;
            boardConfig.configStatus = CONFIG_COMMIT;
            auto* b = (unsigned char*) &(motor.angle);
            for (int i = 0; i < 4; i++)
                i2cDataTx[i + 1] = *(b + i);
            break;
        }
        case 0x25: // Set Kd
        {
            motor.dce.kd = valF;
            boardConfig.dceKd = valF;
            boardConfig.configStatus = CONFIG_COMMIT;
            auto* b = (unsigned char*) &(motor.angle);
            for (int i = 0; i < 4; i++)
                i2cDataTx[i + 1] = *(b + i);
            break;
        }
        case 0x26: // Set torque limit
        {
            motor.SetTorqueLimit(valF);
            boardConfig.toqueLimit = valF;
            boardConfig.configStatus = CONFIG_COMMIT;
            auto* b = (unsigned char*) &(motor.angle);
            for (int i = 0; i < 4; i++)
                i2cDataTx[i + 1] = *(b + i);
            break;
        }
        case 0x27: // Set init pos
        {
            boardConfig.initPos = valF;
            boardConfig.configStatus = CONFIG_COMMIT;
            auto* b = (unsigned char*) &(motor.angle);
            for (int i = 0; i < 4; i++)
                i2cDataTx[i + 1] = *(b + i);
            break;
        }
        case 0x28: // Set ADC min at 0 deg
        {
            boardConfig.adcValAtAngleMin = (uint16_t) valF;
            motor.adcValAtAngleMin = boardConfig.adcValAtAngleMin;
            boardConfig.configStatus = CONFIG_COMMIT;
            auto* b = (unsigned char*) &(motor.angle);
            for (int i = 0; i < 4; i++)
                i2cDataTx[i + 1] = *(b + i);
            break;
        }
        case 0x29: // Set ADC max at 180 deg
        {
            boardConfig.adcValAtAngleMax = (uint16_t) valF;
            motor.adcValAtAngleMax = boardConfig.adcValAtAngleMax;
            boardConfig.configStatus = CONFIG_COMMIT;
            auto* b = (unsigned char*) &(motor.angle);
            for (int i = 0; i < 4; i++)
                i2cDataTx[i + 1] = *(b + i);
            break;
        }
        case 0x2A: // Set Mechanical Max Angle (180 or 360)
        {
            if (valF >= 90.0f && valF <= 360.0f)
            {
                boardConfig.mechanicalAngleMax = valF;
                motor.mechanicalAngleMax = valF;
                boardConfig.configStatus = CONFIG_COMMIT;
            }
            auto* b = (unsigned char*) &(motor.angle);
            for (int i = 0; i < 4; i++)
                i2cDataTx[i + 1] = *(b + i);
            break;
        }
        case 0x30: // Set Invert Direction (0 = Normal, 1 = Inverted)
        {
            motor.invertDirection = (valF != 0);
            boardConfig.invertDirection = motor.invertDirection;
            boardConfig.configStatus = CONFIG_COMMIT;
            auto* b = (unsigned char*) &(motor.angle);
            for (int i = 0; i < 4; i++)
                i2cDataTx[i + 1] = *(b + i);
            break;
        }
        case 0xff:
        {
            bool en = (valF != 0.0f || i2cDataRx[1] != 0);
            if (en && !motor.IsEnabled())
            {
                // Soft enable: hold current horn position gently without jumping
                motor.dce.setPointPos = motor.angle;
                motor.dce.integralPos = 0;
            }
            motor.SetEnable(en);
            auto* b = (unsigned char*) &(motor.angle);
            for (int i = 0; i < 4; i++)
                i2cDataTx[i + 1] = *(b + i);
            break;
        }
        default:
            break;
    }

    LL_I2C_DisableIT_ADDR(I2C1);
    i2cTxReady = 1;
    LL_I2C_EnableIT_ADDR(I2C1);

    if (i2cDataRx[0] == 0x21)
    {
        pendingSetId = 1;
        pendingNewNodeId = boardConfig.nodeId;
    }
}


// 200Hz Closed-Loop Control loop
void TIM14_PeriodElapsedCallback()
{
    uint16_t raw_adc = adcData[0];

    // Gap cutoff protection with 3-sample glitch filter:
    // Wiper valid track is ~130 to ~2950.
    // If a transient glitch or motor inrush brownout reads 0 for 1 cycle,
    // do NOT immediately kill motor PWM; maintain last known good angle.
    // Only kill PWM if reading stays out-of-bounds for 3 consecutive cycles (15ms).
    static uint8_t outOfBoundsCount = 0;
    static uint16_t lastValidRawAdc = 1500;

    if (raw_adc < 100 || raw_adc > 2980)
    {
        outOfBoundsCount++;
        if (outOfBoundsCount >= 3)
        {
            motor.SetPwm(0);
            return;
        }
        raw_adc = lastValidRawAdc;
    }
    else
    {
        outOfBoundsCount = 0;
        lastValidRawAdc = raw_adc;
    }

    if (raw_adc < motor.adcValAtAngleMin)
        raw_adc = motor.adcValAtAngleMin;
    else if (raw_adc > motor.adcValAtAngleMax)
        raw_adc = motor.adcValAtAngleMax;

    motor.angle = motor.mechanicalAngleMin +
                  (motor.mechanicalAngleMax - motor.mechanicalAngleMin) *
                  ((float) raw_adc - (float) motor.adcValAtAngleMin) /
                  ((float) motor.adcValAtAngleMax - (float) motor.adcValAtAngleMin);

    // Calculate PID
    motor.CalcDceOutput(motor.angle, 0);

    // Motor polarity direction (configurable on the fly via I2C opcode 0x30 / UI)
    int16_t pwmVal = (int16_t) motor.dce.output;
    if (motor.invertDirection)
        pwmVal = -pwmVal;

    motor.SetPwm(pwmVal);
}

