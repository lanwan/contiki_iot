/*
 * Copyright (c) 2017, Lanven Jiaxing China
 * http://www.scadai.com
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 */

#ifndef SX1278_ARCH_H
#define SX1278_ARCH_H

#include <stdint.h>
/*---------------------------------------------------------------------------*/
/*
 * Initialize SPI module & Pins.
 *
 * The function has to accomplish the following tasks:
 * - Enable SPI and configure SPI (CPOL = 0, CPHA = 0)
 * - Configure MISO, MOSI, SCLK accordingly
 * - Configure GPIOx (input)
 * - Configure RESET_N (output high)
 * - Configure CSn (output high)
 */
void
sx1278_arch_init(void);
/*---------------------------------------------------------------------------*/
/* Select SI4460 (pull down CSn pin). */
void
sx1278_arch_spi_select(void);
/*---------------------------------------------------------------------------*/
/* De-select SI4460 (release CSn pin). */
void
sx1278_arch_spi_deselect(void);
/*---------------------------------------------------------------------------*/
/*
 * Configure port GPIO port IRQ.
 * If rising == 1: configure IRQ for rising edge, else falling edge
 * Interrupt has to call sx1278_rx_interrupt()!
 */
void
sx1278_arch_setup_irq(int rising);
/*---------------------------------------------------------------------------*/
/* Reset interrupt flag and enable GPIO port IRQ. */
void
sx1278_arch_enable_irq(void);
/*---------------------------------------------------------------------------*/
/* Disable GPIO port IRQ. */
void
sx1278_arch_disable_irq(void);
/*---------------------------------------------------------------------------*/
/*
 * Read back the status of the GPIO pin.
 * Returns 0 if the pin is low, otherwise 1
 */
int
sx1278_arch_gpio_read_pin(void);

void
sx1278_arch_spi_readffer( uint8_t addr, uint8_t *buffer, uint8_t size )
/*---------------------------------------------------------------------------*/
/* Write a single byte via SPI, return response. */
int
sx1278_arch_spi_rw_byte(uint8_t c);
/*---------------------------------------------------------------------------*/
/*
 * Write a sequence of bytes while reading back the response.
 * Either read_buf or write_buf can be NULL.
 */
int
sx1278_arch_spi_rw(uint8_t *read_buf,
                   const uint8_t *write_buf,
                   uint16_t len);
/*---------------------------------------------------------------------------*/
/*
 * The SI4460 interrupt handler exported from the sx1278_ driver.
 *
 * To be called by the hardware interrupt handler(s),
 * which are defined as part of the sx1278_-arch interface.
 */
int
sx1278_rx_interrupt(void);

#endif /* SX1278_ARCH_H */
