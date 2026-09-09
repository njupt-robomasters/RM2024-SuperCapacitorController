/**
 * @file SuperCap.cpp
 * @author JIANG Yicheng RM2023
 * @brief
 * @version 0.1
 * @date 2023-01-22
 *
 * @copyright Copyright (c) 2023
 */

#include "Buzzer.hpp"
#include "Calibration.hpp"
#include "Communication.hpp"
#include "Config.hpp"
#include "PowerManager.hpp"
#include "iwdg.h"
#include "main.h"

namespace SuperCap
{
static void loop()
{
    while (true)
    {
        __HAL_IWDG_RELOAD_COUNTER(&hiwdg);
        HAL_Delay(1);
    }
}

void configIWDG()
{
    hiwdg.Instance = IWDG;
    __HAL_IWDG_START(&hiwdg);
    IWDG_ENABLE_WRITE_ACCESS(&hiwdg);
    hiwdg.Instance->PR  = IWDG_PRESCALER_4;
    hiwdg.Instance->RLR = 1000;
    while ((hiwdg.Instance->SR & (IWDG_SR_WVU | IWDG_SR_RVU | IWDG_SR_PVU)) != 0x00u)
        ;
    __HAL_IWDG_RELOAD_COUNTER(&hiwdg);
}

void init()
{
    HAL_Delay(200);  // wait voltage stabilization

    /* initialize communication */
    Buzzer::init();

    /* check hardware id */
    static volatile uint32_t hardwareID = 0;

    hardwareID = HAL_FLASHEx_OBGetUserData(OB_DATA_ADDRESS_DATA0);

    if (hardwareID != (uint32_t)HARDWARE_ID)
    {
#ifdef CALIBRATION_MODE
        /* 直接寄存器写 Option Byte DATA0 (HAL_FLASHEx_OBProgram 在该 F334 上不生效)。
           只写 DATA0 值 + 取反(半字)，不擦除、不碰 RDP/USER/WRP，避免读保护级别变化 */
        WRITE_REG(FLASH->KEYR, FLASH_KEY1);
        WRITE_REG(FLASH->KEYR, FLASH_KEY2);
        WRITE_REG(FLASH->OPTKEYR, FLASH_OPTKEY1);
        WRITE_REG(FLASH->OPTKEYR, FLASH_OPTKEY2);

        SET_BIT(FLASH->CR, FLASH_CR_OPTPG);
        *(__IO uint16_t *)OB_DATA_ADDRESS_DATA0 = (uint16_t)((HARDWARE_ID & 0xFFU) |
                                                              (((~HARDWARE_ID) & 0xFFU) << 8));
        while (READ_BIT(FLASH->SR, FLASH_SR_BSY) != 0U)
            ;
        CLEAR_BIT(FLASH->CR, FLASH_CR_OPTPG);

        SET_BIT(FLASH->CR, FLASH_CR_LOCK);

        /* OBL_LAUNCH 触发 Option Bytes 重新加载并复位(见 RM0316 FLASH_CR 位13)。
           仅 NVIC_SystemReset() 不会重载 OBR，HAL_FLASHEx_OBGetUserData 仍读到旧值 */
        SET_BIT(FLASH->CR, FLASH_CR_OBL_LAUNCH);
        while (true)
            __ASM volatile("bkpt ");

#endif
        Buzzer::play(1000);
        while (true)
            __ASM volatile("bkpt ");
    }

    /* initialize power manager */
    Buzzer::play(800, 500);

    Communication::init();

    HAL_Delay(600);

    PowerManager::init();

    configIWDG();
}

extern "C"
{
    void systemStart()
    {
        SuperCap::init();
        loop();
    }
}
}  // namespace SuperCap