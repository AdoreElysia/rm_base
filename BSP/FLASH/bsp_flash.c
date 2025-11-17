#include "bsp_flash.h"

/**
 * @description: 擦除指定扇区
 * @param {BSP_FLASH_Sector} sector - 要擦除的扇区
 * @return {BSP_FLASH_Status} 操作状态
 */
BSP_FLASH_Status BSP_FLASH_Erase_Sector(BSP_FLASH_Sector sector)
{
    // 检查参数有效性
    if (sector > BSP_FLASH_SECTOR_11)
    {
        return BSP_FLASH_INVALID_ADDRESS;
    }

    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t               SectorError = 0;
    HAL_StatusTypeDef      status      = HAL_ERROR;

    // 解锁FLASH
    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        return BSP_FLASH_ERROR;
    }

    // 清除所有可能的FLASH错误标志
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                           FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

    // 设置擦除参数
    EraseInitStruct.TypeErase    = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    EraseInitStruct.Sector       = (uint32_t)sector;
    EraseInitStruct.NbSectors    = 1;

    // 执行擦除
    status = HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError);

    // 锁定FLASH
    if (HAL_FLASH_Lock() != HAL_OK)
    {
        return BSP_FLASH_ERROR;
    }

    // 检查擦除结果
    if (status != HAL_OK)
    {
        return BSP_FLASH_ERROR;
    }

    return BSP_FLASH_OK;
}

/**
 * @description: 在指定地址写入数据块
 * @param {uint32_t} address - 写入地址
 * @param {uint8_t*} data - 要写入的数据指针
 * @param {uint32_t} size - 数据大小(字节)
 * @return {BSP_FLASH_Status} 操作状态
 */
BSP_FLASH_Status BSP_FLASH_Write_Buffer(uint32_t address, uint8_t *data, uint32_t size)
{
    // 检查参数有效性
    if (data == NULL || size == 0)
    {
        return BSP_FLASH_INVALID_SIZE;
    }

    // 检查地址有效性
    if (!BSP_FLASH_Is_Address_Valid(address) || !BSP_FLASH_Is_Address_Valid(address + size - 1))
    {
        return BSP_FLASH_INVALID_ADDRESS;
    }

    // 解锁FLASH
    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        return BSP_FLASH_ERROR;
    }

    // 清除所有可能的FLASH错误标志
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                           FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

    // 按字节写入数据
    HAL_StatusTypeDef status = HAL_OK;
    for (uint32_t i = 0; i < size; i++)
    {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, address + i, data[i]);
        if (status != HAL_OK)
        {
            HAL_FLASH_Lock();
            return BSP_FLASH_ERROR;
        }
    }

    // 锁定FLASH
    if (HAL_FLASH_Lock() != HAL_OK)
    {
        return BSP_FLASH_ERROR;
    }

    return BSP_FLASH_OK;
}

/**
 * @description: 从指定地址读取数据块
 * @param {uint32_t} address - 读取地址
 * @param {uint8_t*} data - 读取到的数据指针
 * @param {uint32_t} size - 数据大小(字节)
 * @return {BSP_FLASH_Status} 操作状态
 */
BSP_FLASH_Status BSP_FLASH_Read_Buffer(uint32_t address, uint8_t *data, uint32_t size)
{
    // 检查参数有效性
    if (data == NULL || size == 0)
    {
        return BSP_FLASH_INVALID_SIZE;
    }

    // 检查地址有效性
    if (!BSP_FLASH_Is_Address_Valid(address) || !BSP_FLASH_Is_Address_Valid(address + size - 1))
    {
        return BSP_FLASH_INVALID_ADDRESS;
    }

    // 读取数据
    for (uint32_t i = 0; i < size; i++)
    {
        data[i] = *(volatile uint8_t *)(address + i);
    }

    return BSP_FLASH_OK;
}

/**
 * @description: 获取扇区起始地址
 * @param {BSP_FLASH_Sector} sector - 扇区编号
 * @return {uint32_t} 扇区起始地址
 */
uint32_t BSP_FLASH_Get_Sector_Address(BSP_FLASH_Sector sector)
{
    switch (sector)
    {
    case BSP_FLASH_SECTOR_0:
        return 0x08000000U;
    case BSP_FLASH_SECTOR_1:
        return 0x08004000U;
    case BSP_FLASH_SECTOR_2:
        return 0x08008000U;
    case BSP_FLASH_SECTOR_3:
        return 0x0800C000U;
    case BSP_FLASH_SECTOR_4:
        return 0x08010000U;
    case BSP_FLASH_SECTOR_5:
        return 0x08020000U;
    case BSP_FLASH_SECTOR_6:
        return 0x08040000U;
    case BSP_FLASH_SECTOR_7:
        return 0x08060000U;
    case BSP_FLASH_SECTOR_8:
        return 0x08080000U;
    case BSP_FLASH_SECTOR_9:
        return 0x080A0000U;
    case BSP_FLASH_SECTOR_10:
        return 0x080C0000U;
    case BSP_FLASH_SECTOR_11:
        return 0x080E0000U;
    default:
        return 0xFFFFFFFFU;
    }
}

/**
 * @description: 检查地址是否有效
 * @param {uint32_t} address - 要检查的地址
 * @return {uint8_t} 1表示有效，0表示无效
 */
uint8_t BSP_FLASH_Is_Address_Valid(uint32_t address)
{
    if (address >= BSP_FLASH_BASE_ADDR && address <= BSP_FLASH_END_ADDR)
    {
        return 1;
    }
    return 0;
}