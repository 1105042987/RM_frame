[#ftl]
/**
  ******************************************************************************
  * @file    stm32f4xx_hal_conf.h
  * @brief   HAL configuration file.             
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) ${year} STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F4xx_HAL_CONF_H
#define __STM32F4xx_HAL_CONF_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "main.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* ########################## Module Selection ############################## */
/**
  * @brief This is the list of modules to be used in the HAL driver 
  */
#define HAL_MODULE_ENABLED  

  [#assign allModules = ["ADC","AES","CAN","CRC","CRYP","DAC","DCMI","DMA2D","ETH","NAND","NOR","PCCARD","SRAM","SDRAM","HASH","I2C","I2S","IWDG","LTDC","RNG","RTC","SAI","SD","MMC","SPI","TIM","UART","USART","IRDA","SMARTCARD","WWDG","PCD","HCD", "DSI","QSPI","QUADSPI","CEC","FMPI2C","SPDIFRX", "DFSDM","LPTIM"]]
  [#list allModules as module]
	[#if isModuleUsed(module)]
[#compress]#define HAL_${module?replace("QUADSPI","QSPI")?replace("AES","CRYP")}_MODULE_ENABLED[/#compress]
	[#else]
/* #define HAL_${module?replace("QUADSPI","QSPI")?replace("AES","CRYP")}_MODULE_ENABLED   */
	[/#if]	
  [/#list]
  [#function isModuleUsed moduleName]
	[#assign used=false]
	[#list modules as usedModule]
		[#if usedModule==moduleName]
			[#assign used=true]
			[#return true]
		[/#if]
	[/#list]
	[#return used]
  [/#function]
#define HAL_GPIO_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED
#define HAL_PWR_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED

/* ########################## HSE/HSI Values adaptation ##################### */
/**
  * @brief Adjust the value of External High Speed oscillator (HSE) used in your application.
  *        This value is used by the RCC HAL module to compute the system frequency
  *        (when HSE is used as system clock source, directly or through the PLL).  
  */
#if !defined  (HSE_VALUE) 
  #define HSE_VALUE    ((uint32_t)[#if hse_value??]${hse_value}[#else]25000000[/#if]U) /*!< Value of the External oscillator in Hz */
#endif /* HSE_VALUE */

#if !defined  (HSE_STARTUP_TIMEOUT)
  #define HSE_STARTUP_TIMEOUT    ((uint32_t)[#if HSE_Timout??]${HSE_Timout}[#if HSE_Timout?matches("(0x[0-9]*[a-f]*[A-F]*)|([0-9]*)")]U[/#if][#else]5000U[/#if])   /*!< Time out for HSE start up, in ms */
#endif /* HSE_STARTUP_TIMEOUT */

/**
  * @brief Internal High Speed oscillator (HSI) value.
  *        This value is used by the RCC HAL module to compute the system frequency
  *        (when HSI is used as system clock source, directly or through the PLL). 
  */
#if !defined  (HSI_VALUE)
  #define HSI_VALUE    ((uint32_t)16000000U) /*!< Value of the Internal oscillator in Hz*/
#endif /* HSI_VALUE */

/**
  * @brief Internal Low Speed oscillator (LSI) value.
  */
#if !defined  (LSI_VALUE) 
 #define LSI_VALUE  ((uint32_t)[#if lsi_value??]${lsi_value}[#else]32000[/#if]U)       /*!< LSI Typical Value in Hz*/
#endif /* LSI_VALUE */                      /*!< Value of the Internal Low Speed oscillator in Hz
                                             The real value may vary depending on the variations
                                             in voltage and temperature.*/
/**
  * @brief External Low Speed oscillator (LSE) value.
  */
#if !defined  (LSE_VALUE)
 #define LSE_VALUE  ((uint32_t)[#if lse_value??]${lse_value}[#else]32768[/#if]U)    /*!< Value of the External Low Speed oscillator in Hz */
#endif /* LSE_VALUE */

#if !defined  (LSE_STARTUP_TIMEOUT)
  #define LSE_STARTUP_TIMEOUT    ((uint32_t)[#if LSE_Timout??]${LSE_Timout}[#if LSE_Timout?matches("(0x[0-9]*[a-f]*[A-F]*)|([0-9]*)")]U[/#if][#else]5000U[/#if])   /*!< Time out for LSE start up, in ms */
#endif /* LSE_STARTUP_TIMEOUT */

/**
  * @brief External clock source for I2S peripheral
  *        This value is used by the I2S HAL module to compute the I2S clock source 
  *        frequency, this source is inserted directly through I2S_CKIN pad. 
  */
#if !defined  (EXTERNAL_CLOCK_VALUE)
  #define EXTERNAL_CLOCK_VALUE    ((uint32_t)[#if external_clock_value??]${external_clock_value}[#else]12288000[/#if]U) /*!< Value of the External audio frequency in Hz*/
#endif /* EXTERNAL_CLOCK_VALUE */

/* Tip: To avoid modifying this file each time you need to use different HSE,
   ===  you can define the HSE value in your toolchain compiler preprocessor. */

/* ########################### System Configuration ######################### */
/**
  * @brief This is the HAL system configuration section
  */
[#if advancedSettings??][#assign advancedSettings = advancedSettings[0]][/#if]     
#define  VDD_VALUE		      ((uint32_t)[#if vdd_value??]${vdd_value}[#if vdd_value?matches("(0x[0-9]*[a-f]*[A-F]*)|([0-9]*)")]U[/#if])[#else]3300U)[/#if] /*!< Value of VDD in mv */           
#define  TICK_INT_PRIORITY            ((uint32_t)[#if TICK_INT_PRIORITY??]${TICK_INT_PRIORITY}[#if TICK_INT_PRIORITY?matches("(0x[0-9]*[a-f]*[A-F]*)|([0-9]*)")]U[/#if][#else]0x0FU[/#if])   /*!< tick interrupt priority */            
#define  USE_RTOS                     [#if advancedSettings?? && advancedSettings.USE_RTOS??]${advancedSettings.USE_RTOS}[#else]0[/#if]U     
#define  PREFETCH_ENABLE              [#if PREFETCH_ENABLE??]${PREFETCH_ENABLE}[#else]1[/#if]U
#define  INSTRUCTION_CACHE_ENABLE     [#if INSTRUCTION_CACHE_ENABLE??]${INSTRUCTION_CACHE_ENABLE}[#else]1[/#if]U
#define  DATA_CACHE_ENABLE            [#if DATA_CACHE_ENABLE??]${DATA_CACHE_ENABLE}[#else]1[/#if]U

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
[#if !fullAssert??]/*[/#if] #define USE_FULL_ASSERT    1U [#if !fullAssert??]*/[/#if]

/* ################## Ethernet peripheral configuration ##################### */

/* Section 1 : Ethernet peripheral configuration */

/* MAC ADDRESS: MAC_ADDR0:MAC_ADDR1:MAC_ADDR2:MAC_ADDR3:MAC_ADDR4:MAC_ADDR5 */
#define MAC_ADDR0   2U
#define MAC_ADDR1   0U
#define MAC_ADDR2   0U
#define MAC_ADDR3   0U
#define MAC_ADDR4   0U
#define MAC_ADDR5   0U

/* Definition of the Ethernet driver buffers size and count */   
#define ETH_RX_BUF_SIZE                ETH_MAX_PACKET_SIZE /* buffer size for receive               */
#define ETH_TX_BUF_SIZE                ETH_MAX_PACKET_SIZE /* buffer size for transmit              */
#define ETH_RXBUFNB                    ((uint32_t)[#if ETH_RXBUFNB??]${ETH_RXBUFNB}[#if ETH_RXBUFNB?matches("(0x[0-9]*[a-f]*[A-F]*)|([0-9]*)")]U[/#if][#else]4U[/#if])       /* 4 Rx buffers of size ETH_RX_BUF_SIZE  */
#define ETH_TXBUFNB                    ((uint32_t)[#if ETH_TXBUFNB??]${ETH_TXBUFNB}[#if ETH_TXBUFNB?matches("(0x[0-9]*[a-f]*[A-F]*)|([0-9]*)")]U[/#if][#else]4U[/#if])       /* 4 Tx buffers of size ETH_TX_BUF_SIZE  */

/* Section 2: PHY configuration section */

/* [#if PHY_Name??]${PHY_Name}[#else]DP83848_PHY_ADDRESS[/#if] Address*/ 
#define [#if PHY_Name??]${PHY_Name}[#else]DP83848_PHY_ADDRESS[/#if]           [#if PHY_Value??]${PHY_Value}[#if PHY_Value?matches("(0x[0-9]*[a-f]*[A-F]*)|([0-9]*)")]U[/#if][#else]0x01U[/#if]
/* PHY Reset delay these values are based on a 1 ms Systick interrupt*/ 
#define PHY_RESET_DELAY                 ((uint32_t)[#if PHY_RESET_DELAY??]${PHY_RESET_DELAY}[#if PHY_RESET_DELAY?matches("(0x[0-9]*[a-f]*[A-F]*)|([0-9]*)")]U[/#if][#else]0x000000FFU[/#if])
/* PHY Configuration delay */
#define PHY_CONFIG_DELAY                ((uint32_t)[#if PHY_CONFIG_DELAY??]${PHY_CONFIG_DELAY}[#if PHY_CONFIG_DELAY?matches("(0x[0-9]*[a-f]*[A-F]*)|([0-9]*)")]U[/#if][#else]0x00000FFFU[/#if])

#define PHY_READ_TO                     ((uint32_t)[#if PHY_READ_TO??]${PHY_READ_TO}[#if PHY_READ_TO?matches("(0x[0-9]*[a-f]*[A-F]*)|([0-9]*)")]U[/#if][#else]0x0000FFFFU[/#if])
#define PHY_WRITE_TO                    ((uint32_t)[#if PHY_WRITE_TO??]${PHY_WRITE_TO}[#if PHY_WRITE_TO?matches("(0x[0-9]*[a-f]*[A-F]*)|([0-9]*)")]U[/#if][#else]0x0000FFFFU[/#if])

/* Section 3: Common PHY Registers */

#define PHY_BCR                         ((uint16_t)[#if PHY_BCR??]${PHY_BCR}[#if PHY_BCR?matches("(0x[0-9]*[a-f]*[A-F]*)|([0-9]*)")]U[/#if][#else]0x0000U[/#if])    /*!< Transceiver Basic Control Register   */
#define PHY_BSR                         ((uint16_t)[#if PHY_BSR??]${PHY_BSR}[#if PHY_BSR?matches("(0x[0-9]*[a-f]*[A-F]*)|([0-9]*)")]U[/#if][#else]0x0001U[/#if])    /*!< Transceiver Basic Status Register    */
 
#define PHY_RESET                       ((uint16_t)[#if PHY_RESET??]${PHY_RESET}[#if PHY_RESET?matches("(0x[0-9]*[a-f]*[A-F]*)|([0-9]*)")]U[/#if][#else]0x8000U[/#if])  /*!< PHY Reset */
#define PHY_LOOPBACK                    ((uint16_t)[#if PHY_LOOPBACK??]${PHY_LOOPBACK}[#if PHY_LOOPBACK?matches("(0x[0-9]*[a-f]*[A-F]*)|([0-9]*)")]U[/#if][#else]0x4000U[/#if])  /*!< Select loop-back mode */
#define PHY_FULLDUPLEX_100M             ((uint16_t)[#if PHY_FULLDUPLEX_100M??]${PHY_FULLDUPLEX_100M}[#if PHY_FULLDUPLEX_100M?matches("(0x[0-9]*[a-f]*[A-F]*)|([0-9]*)")]U[/#if][#else]0x2100U[/#if])  /*!< Set the full-duplex mode at 100 Mb/s */
#define PHY_HALFDUPLEX_100M             ((uint16_t)[#if PHY_HALFDUPLEX_100M??]${PHY_HALFDUPLEX_100M}[#if PHY_HALFDUPLEX_100M?matches("(0x[0-9]*[a-f]*[A-F]*)|([0-9]*)")]U[/#if][#else]0x2000U[/#if])  /*!< Set the half-duplex mode at 100 Mb/s */
#define PHY_FULLDUPLEX_10M              ((uint16_t)[#if PHY_FULLDUPLEX_10M??]${PHY_FULLDUPLEX_10M}[#if PHY_FULLDUPLEX_10M?matches("(0x[0-9]*[a-f]*[A-F]*)|([0-9]*)")]U[/#if][#else]0x0100U[/#if])  /*!< Set the full-duplex mode at 10 Mb/s  */
#define PHY_HALFDUPLEX_10M              ((uint16_t)[#if PHY_HALFDUPLEX_10M??]${PHY_HALFDUPLEX_10M}[#if PHY_HALFDUPLEX_10M?matches("(0x[0-9]*[a-f]*[A-F]*)|([0-9]*)")]U[/#if][#else]0x0000U[/#if])  /*!< Set the half-duplex mode at 10 Mb/s  */
#define PHY_AUTONEGOTIATION             ((uint16_t)[#if PHY_AUTONEGOTIATION??]${PHY_AUTONEGOTIATION}[#if PHY_AUTONEGOTIATION?matches("(0x[0-9]*[a-f]*[A-F]*)|([0-9]*)")]U[/#if][#else]0x1000U[/#if])  /*!< Enable auto-negotiation function     */
#define PHY_RESTART_AUTONEGOTIATION     ((uint16_t)[#if PHY_RESTART_AUTONEGOTIATION??]${PHY_RESTART_AUTONEGOTIATION}[#if PHY_RESTART_AUTONEGOTIATION?matches("(0x[0-9]*[a-f]*[A-F]*)|([0-9]*)")]U[/#if][#else]0x0200U[/#if])  /*!< Restart auto-negotiation function    */
#define PHY_POWERDOWN                   ((uint16_t)[#if PHY_POWERDOWN??]${PHY_POWERDOWN}[#if PHY_POWERDOWN?matches("(0x[0-9]*[a-f]*[A-F]*)|([0-9]*)")]U[/#if][#else]0x0800U[/#if])  /*!< Select the power down mode           */
#define PHY_ISOLATE                     ((uint16_t)[#if PHY_ISOLATE??]${PHY_ISOLATE}[#if PHY_ISOLATE?matches("(0x[0-9]*[a-f]*[A-F]*)|([0-9]*)")]U[/#if][#else]0x0400U[/#if])  /*!< Isolate PHY from MII                 */

#define PHY_AUTONEGO_COMPLETE           ((uint16_t)[#if PHY_AUTONEGO_COMPLETE??]${PHY_AUTONEGO_COMPLETE}[#if PHY_AUTONEGO_COMPLETE?matches("(0x[0-9]*[a-f]*[A-F]*)|([0-9]*)")]U[/#if][#else]0x0020U[/#if])  /*!< Auto-Negotiation process completed   */
#define PHY_LINKED_STATUS               ((uint16_t)[#if PHY_LINKED_STATUS??]${PHY_LINKED_STATUS}[#if PHY_LINKED_STATUS?matches("(0x[0-9]*[a-f]*[A-F]*)|([0-9]*)")]U[/#if][#else]0x0004U[/#if])  /*!< Valid link established               */
#define PHY_JABBER_DETECTION            ((uint16_t)[#if PHY_JABBER_DETECTION??]${PHY_JABBER_DETECTION}[#if PHY_JABBER_DETECTION?matches("(0x[0-9]*[a-f]*[A-F]*)|([0-9]*)")]U[/#if][#else]0x0002U[/#if])  /*!< Jabber condition detected            */
  
/* Section 4: Extended PHY Registers */
[#--Common define--]
#define PHY_SR                          ((uint16_t)[#if PHY_SR??]${PHY_SR}[#if PHY_SR?matches("(0x[0-9]*[a-f]*[A-F]*)|([0-9]*)")]U[/#if][#else]0x10U[/#if])    /*!< PHY status register Offset                      */
[#if PHY_Name?? && (PHY_Name=="DP83848_PHY_ADDRESS"||PHY_Name=="DP83848")] 
#define PHY_MICR                        ((uint16_t)[#if PHY_MICR??]${PHY_MICR}[#if PHY_MICR?matches("(0x[0-9]*[a-f]*[A-F]*)|([0-9]*)")]U[/#if][#else]0x11U[/#if])    /*!< MII Interrupt Control Register                  */
#define PHY_MISR                        ((uint16_t)[#if PHY_MISR??]${PHY_MISR}[#if PHY_MISR?matches("(0x[0-9]*[a-f]*[A-F]*)|([0-9]*)")]U[/#if][#else]0x12U[/#if])    /*!< MII Interrupt Status and Misc. Control Register */
#n#define PHY_LINK_STATUS                 ((uint16_t)[#if PHY_LINK_STATUS??]${PHY_LINK_STATUS}[#if PHY_LINK_STATUS?matches("(0x[0-9]*[a-f]*[A-F]*)|([0-9]*)")]U[/#if][#else]0x0001U[/#if])  /*!< PHY Link mask                                   */
[#else]
#n
[/#if]
[#--Common define--]
#define PHY_SPEED_STATUS                ((uint16_t)[#if PHY_SPEED_STATUS??]${PHY_SPEED_STATUS}[#if PHY_SPEED_STATUS?matches("(0x[0-9]*[a-f]*[A-F]*)|([0-9]*)")]U[/#if][#else]0x0002U[/#if])  /*!< PHY Speed mask                                  */
#define PHY_DUPLEX_STATUS               ((uint16_t)[#if PHY_DUPLEX_STATUS??]${PHY_DUPLEX_STATUS}[#if PHY_DUPLEX_STATUS?matches("(0x[0-9]*[a-f]*[A-F]*)|([0-9]*)")]U[/#if][#else]0x0004U[/#if])  /*!< PHY Duplex mask                                 */
[#if PHY_Name?? && (PHY_Name=="DP83848_PHY_ADDRESS"||PHY_Name=="DP83848")]
#n#define PHY_MICR_INT_EN                 ((uint16_t)[#if PHY_MICR_INT_EN??]${PHY_MICR_INT_EN}[#if PHY_MICR_INT_EN?matches("(0x[0-9]*[a-f]*[A-F]*)|([0-9]*)")]U[/#if][#else]0x0002U[/#if])  /*!< PHY Enable interrupts                           */
#define PHY_MICR_INT_OE                 ((uint16_t)[#if PHY_MICR_INT_OE??]${PHY_MICR_INT_OE}[#if PHY_MICR_INT_OE?matches("(0x[0-9]*[a-f]*[A-F]*)|([0-9]*)")]U[/#if][#else]0x0001U[/#if])  /*!< PHY Enable output interrupt events              */
#n#define PHY_MISR_LINK_INT_EN            ((uint16_t)[#if PHY_MISR_LINK_INT_EN??]${PHY_MISR_LINK_INT_EN}[#if PHY_MISR_LINK_INT_EN?matches("(0x[0-9]*[a-f]*[A-F]*)|([0-9]*)")]U[/#if][#else]0x0020U[/#if])  /*!< Enable Interrupt on change of link status       */
#define PHY_LINK_INTERRUPT              ((uint16_t)[#if PHY_LINK_INTERRUPT??]${PHY_LINK_INTERRUPT}[#if PHY_LINK_INTERRUPT?matches("(0x[0-9]*[a-f]*[A-F]*)|([0-9]*)")]U[/#if][#else]0x2000U[/#if])  /*!< PHY link status interrupt mask                  */
[/#if]
[#if PHY_Name?? && (PHY_Name=="LAN8742A_PHY_ADDRESS")]
#n#define PHY_ISFR                        ((uint16_t)[#if PHY_ISFR??]${PHY_ISFR}[#if PHY_ISFR?matches("(0x[0-9]*[a-f]*[A-F]*)|([0-9]*)")]U[/#if][#else]0x1DU[/#if])    /*!< PHY Interrupt Source Flag register Offset   */
#define PHY_ISFR_INT4                   ((uint16_t)[#if PHY_ISFR_INT4??]${PHY_ISFR_INT4}[#if PHY_ISFR_INT4?matches("(0x[0-9]*[a-f]*[A-F]*)|([0-9]*)")]U[/#if][#else]0x0010U[/#if])  /*!< PHY Link down inturrupt       */  
[/#if]

/* ################## SPI peripheral configuration ########################## */

/* CRC FEATURE: Use to activate CRC feature inside HAL SPI Driver
* Activated: CRC code is present inside driver
* Deactivated: CRC code cleaned from driver
*/

#define USE_SPI_CRC                     [#if CRC_SPI??]${CRC_SPI}[#else]1U[/#if]

/* Includes ------------------------------------------------------------------*/
/**
  * @brief Include module's header file 
  */

#ifdef HAL_RCC_MODULE_ENABLED
  #include "stm32f4xx_hal_rcc.h"
#endif /* HAL_RCC_MODULE_ENABLED */

#ifdef HAL_GPIO_MODULE_ENABLED
  #include "stm32f4xx_hal_gpio.h"
#endif /* HAL_GPIO_MODULE_ENABLED */

#ifdef HAL_DMA_MODULE_ENABLED
  #include "stm32f4xx_hal_dma.h"
#endif /* HAL_DMA_MODULE_ENABLED */
   
#ifdef HAL_CORTEX_MODULE_ENABLED
  #include "stm32f4xx_hal_cortex.h"
#endif /* HAL_CORTEX_MODULE_ENABLED */

#ifdef HAL_ADC_MODULE_ENABLED
  #include "stm32f4xx_hal_adc.h"
#endif /* HAL_ADC_MODULE_ENABLED */

#ifdef HAL_CAN_MODULE_ENABLED
  #include "stm32f4xx_hal_can.h"
#endif /* HAL_CAN_MODULE_ENABLED */

#ifdef HAL_CRC_MODULE_ENABLED
  #include "stm32f4xx_hal_crc.h"
#endif /* HAL_CRC_MODULE_ENABLED */

#ifdef HAL_CRYP_MODULE_ENABLED
  #include "stm32f4xx_hal_cryp.h" 
#endif /* HAL_CRYP_MODULE_ENABLED */

#ifdef HAL_DMA2D_MODULE_ENABLED
  #include "stm32f4xx_hal_dma2d.h"
#endif /* HAL_DMA2D_MODULE_ENABLED */

#ifdef HAL_DAC_MODULE_ENABLED
  #include "stm32f4xx_hal_dac.h"
#endif /* HAL_DAC_MODULE_ENABLED */

#ifdef HAL_DCMI_MODULE_ENABLED
  #include "stm32f4xx_hal_dcmi.h"
#endif /* HAL_DCMI_MODULE_ENABLED */

#ifdef HAL_ETH_MODULE_ENABLED
  #include "stm32f4xx_hal_eth.h"
#endif /* HAL_ETH_MODULE_ENABLED */

#ifdef HAL_FLASH_MODULE_ENABLED
  #include "stm32f4xx_hal_flash.h"
#endif /* HAL_FLASH_MODULE_ENABLED */
 
#ifdef HAL_SRAM_MODULE_ENABLED
  #include "stm32f4xx_hal_sram.h"
#endif /* HAL_SRAM_MODULE_ENABLED */

#ifdef HAL_NOR_MODULE_ENABLED
  #include "stm32f4xx_hal_nor.h"
#endif /* HAL_NOR_MODULE_ENABLED */

#ifdef HAL_NAND_MODULE_ENABLED
  #include "stm32f4xx_hal_nand.h"
#endif /* HAL_NAND_MODULE_ENABLED */

#ifdef HAL_PCCARD_MODULE_ENABLED
  #include "stm32f4xx_hal_pccard.h"
#endif /* HAL_PCCARD_MODULE_ENABLED */ 
  
#ifdef HAL_SDRAM_MODULE_ENABLED
  #include "stm32f4xx_hal_sdram.h"
#endif /* HAL_SDRAM_MODULE_ENABLED */      

#ifdef HAL_HASH_MODULE_ENABLED
 #include "stm32f4xx_hal_hash.h"
#endif /* HAL_HASH_MODULE_ENABLED */

#ifdef HAL_I2C_MODULE_ENABLED
 #include "stm32f4xx_hal_i2c.h"
#endif /* HAL_I2C_MODULE_ENABLED */

#ifdef HAL_I2S_MODULE_ENABLED
 #include "stm32f4xx_hal_i2s.h"
#endif /* HAL_I2S_MODULE_ENABLED */

#ifdef HAL_IWDG_MODULE_ENABLED
 #include "stm32f4xx_hal_iwdg.h"
#endif /* HAL_IWDG_MODULE_ENABLED */

#ifdef HAL_LTDC_MODULE_ENABLED
 #include "stm32f4xx_hal_ltdc.h"
#endif /* HAL_LTDC_MODULE_ENABLED */

#ifdef HAL_PWR_MODULE_ENABLED
 #include "stm32f4xx_hal_pwr.h"
#endif /* HAL_PWR_MODULE_ENABLED */

#ifdef HAL_RNG_MODULE_ENABLED
 #include "stm32f4xx_hal_rng.h"
#endif /* HAL_RNG_MODULE_ENABLED */

#ifdef HAL_RTC_MODULE_ENABLED
 #include "stm32f4xx_hal_rtc.h"
#endif /* HAL_RTC_MODULE_ENABLED */

#ifdef HAL_SAI_MODULE_ENABLED
 #include "stm32f4xx_hal_sai.h"
#endif /* HAL_SAI_MODULE_ENABLED */

#ifdef HAL_SD_MODULE_ENABLED
 #include "stm32f4xx_hal_sd.h"
#endif /* HAL_SD_MODULE_ENABLED */

#ifdef HAL_MMC_MODULE_ENABLED
 #include "stm32f4xx_hal_mmc.h"
#endif /* HAL_MMC_MODULE_ENABLED */

#ifdef HAL_SPI_MODULE_ENABLED
 #include "stm32f4xx_hal_spi.h"
#endif /* HAL_SPI_MODULE_ENABLED */

#ifdef HAL_TIM_MODULE_ENABLED
 #include "stm32f4xx_hal_tim.h"
#endif /* HAL_TIM_MODULE_ENABLED */

#ifdef HAL_UART_MODULE_ENABLED
 #include "stm32f4xx_hal_uart.h"
#endif /* HAL_UART_MODULE_ENABLED */

#ifdef HAL_USART_MODULE_ENABLED
 #include "stm32f4xx_hal_usart.h"
#endif /* HAL_USART_MODULE_ENABLED */

#ifdef HAL_IRDA_MODULE_ENABLED
 #include "stm32f4xx_hal_irda.h"
#endif /* HAL_IRDA_MODULE_ENABLED */

#ifdef HAL_SMARTCARD_MODULE_ENABLED
 #include "stm32f4xx_hal_smartcard.h"
#endif /* HAL_SMARTCARD_MODULE_ENABLED */

#ifdef HAL_WWDG_MODULE_ENABLED
 #include "stm32f4xx_hal_wwdg.h"
#endif /* HAL_WWDG_MODULE_ENABLED */

#ifdef HAL_PCD_MODULE_ENABLED
 #include "stm32f4xx_hal_pcd.h"
#endif /* HAL_PCD_MODULE_ENABLED */

#ifdef HAL_HCD_MODULE_ENABLED
 #include "stm32f4xx_hal_hcd.h"
#endif /* HAL_HCD_MODULE_ENABLED */
   
#ifdef HAL_DSI_MODULE_ENABLED
 #include "stm32f4xx_hal_dsi.h"
#endif /* HAL_DSI_MODULE_ENABLED */

#ifdef HAL_QSPI_MODULE_ENABLED
 #include "stm32f4xx_hal_qspi.h"
#endif /* HAL_QSPI_MODULE_ENABLED */

#ifdef HAL_CEC_MODULE_ENABLED
 #include "stm32f4xx_hal_cec.h"
#endif /* HAL_CEC_MODULE_ENABLED */

#ifdef HAL_FMPI2C_MODULE_ENABLED
 #include "stm32f4xx_hal_fmpi2c.h"
#endif /* HAL_FMPI2C_MODULE_ENABLED */

#ifdef HAL_SPDIFRX_MODULE_ENABLED
 #include "stm32f4xx_hal_spdifrx.h"
#endif /* HAL_SPDIFRX_MODULE_ENABLED */

#ifdef HAL_DFSDM_MODULE_ENABLED
 #include "stm32f4xx_hal_dfsdm.h"
#endif /* HAL_DFSDM_MODULE_ENABLED */

#ifdef HAL_LPTIM_MODULE_ENABLED
 #include "stm32f4xx_hal_lptim.h"
#endif /* HAL_LPTIM_MODULE_ENABLED */
   
/* Exported macro ------------------------------------------------------------*/
#ifdef  USE_FULL_ASSERT
/**
  * @brief  The assert_param macro is used for function's parameters check.
  * @param  expr: If expr is false, it calls assert_failed function
  *         which reports the name of the source file and the source
  *         line number of the call that failed. 
  *         If expr is true, it returns no value.
  * @retval None
  */
  #define assert_param(expr) ((expr) ? (void)0U : assert_failed((uint8_t *)__FILE__, __LINE__))
/* Exported functions ------------------------------------------------------- */
  void assert_failed(uint8_t* file, uint32_t line);
#else
  #define assert_param(expr) ((void)0U)
#endif /* USE_FULL_ASSERT */    

#ifdef __cplusplus
}
#endif

#endif /* __STM32F4xx_HAL_CONF_H */
 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
