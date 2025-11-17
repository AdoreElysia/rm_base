/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-10-02 14:23:32
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-10-02 19:29:57
 * @FilePath: \rm_base\BSP\FLASH\bsp_flash.h
 * @Description:
 */
#ifndef _BSP_FLASH_H_
#define _BSP_FLASH_H_

#include "osal_def.h"
#include "stm32f4xx_hal.h"

// STM32F4系列Flash扇区定义
typedef enum
{
    BSP_FLASH_SECTOR_0  = 0U,  // 0x08000000 - 0x08003FFF (16KB)
    BSP_FLASH_SECTOR_1  = 1U,  // 0x08004000 - 0x08007FFF (16KB)
    BSP_FLASH_SECTOR_2  = 2U,  // 0x08008000 - 0x0800BFFF (16KB)
    BSP_FLASH_SECTOR_3  = 3U,  // 0x0800C000 - 0x0800FFFF (16KB)
    BSP_FLASH_SECTOR_4  = 4U,  // 0x08010000 - 0x0801FFFF (64KB)
    BSP_FLASH_SECTOR_5  = 5U,  // 0x08020000 - 0x0803FFFF (128KB)
    BSP_FLASH_SECTOR_6  = 6U,  // 0x08040000 - 0x0805FFFF (128KB)
    BSP_FLASH_SECTOR_7  = 7U,  // 0x08060000 - 0x0807FFFF (128KB)
    BSP_FLASH_SECTOR_8  = 8U,  // 0x08080000 - 0x0809FFFF (128KB)
    BSP_FLASH_SECTOR_9  = 9U,  // 0x080A0000 - 0x080BFFFF (128KB)
    BSP_FLASH_SECTOR_10 = 10U, // 0x080C0000 - 0x080DFFFF (128KB)
    BSP_FLASH_SECTOR_11 = 11U  // 0x080E0000 - 0x080FFFFF (128KB)
} BSP_FLASH_Sector;

#define BSP_FLASH_BASE_ADDR 0x08000000U
#define BSP_FLASH_END_ADDR  0x080FFFFFU

// Flash操作状态
typedef enum
{
    BSP_FLASH_OK = 0,
    BSP_FLASH_ERROR,
    BSP_FLASH_INVALID_ADDRESS,
    BSP_FLASH_INVALID_SIZE
} BSP_FLASH_Status;

// 函数声明
/**
 * @description: 擦除指定扇区
 * @param {BSP_FLASH_Sector} sector - 要擦除的扇区
 * @return {BSP_FLASH_Status} 操作状态
 */
BSP_FLASH_Status BSP_FLASH_Erase_Sector(BSP_FLASH_Sector sector);
/**
 * @description: 在指定地址写入数据块
 * @param {uint32_t} address - 写入地址
 * @param {uint8_t*} data - 要写入的数据指针
 * @param {uint32_t} size - 数据大小(字节)
 * @return {BSP_FLASH_Status} 操作状态
 */
BSP_FLASH_Status BSP_FLASH_Write_Buffer(uint32_t address, uint8_t *buffer, uint32_t length);
/**
 * @description: 从指定地址读取数据块
 * @param {uint32_t} address - 读取地址
 * @param {uint8_t*} data - 读取到的数据指针
 * @param {uint32_t} size - 数据大小(字节)
 * @return {BSP_FLASH_Status} 操作状态
 */
BSP_FLASH_Status BSP_FLASH_Read_Buffer(uint32_t address, uint8_t *buffer, uint32_t length);
/**
 * @description: 获取扇区起始地址
 * @param {BSP_FLASH_Sector} sector - 扇区编号
 * @return {uint32_t} 扇区起始地址
 */
uint32_t BSP_FLASH_Get_Sector_Address(BSP_FLASH_Sector sector);
/**
 * @description: 检查地址是否有效
 * @param {uint32_t} address - 要检查的地址
 * @return {uint8_t} 是否有效
 */
uint8_t BSP_FLASH_Is_Address_Valid(uint32_t address);

#endif // _BSP_FLASH_H_