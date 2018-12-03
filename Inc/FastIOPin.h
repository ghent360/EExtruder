#pragma once

#include <stdint.h>
#include "gpio.h"

/**
 * Fast I/O interfaces for STM32F0
 * These use GPIO functions instead of Direct Port Manipulation, as on AVR.
 */

enum PortNumber {
    PORTA = 0,
    PORTB = 1,
    PORTC = 2,
    PORTD = 3,
    PORTE = 4,
    PORTF = 5,
};

enum PinMode {
    INPUT = 0,
    INPUT_PULLUP,
    INPUT_PULLDOWN,
    OUTPUT
};

#define FAST_IO_PIN(port, pin_no) (0x7000 | ((port) << 4) | (pin_no & 0xf))
#define IS_FAST_IO_PIN(pin) (((pin) & 0xf000) == 0x7000)
#define GET_PORT(fio_pin) ((fio_pin >> 4) & 0xf)
#define GET_PIN_IDX(fio_pin) (fio_pin & 0xf)

struct FastIOPin {
    const intptr_t port_addr_;
    const uint16_t pin_;
    const uint16_t port_num_;

    constexpr FastIOPin(uint32_t fast_io_pin)
        : port_addr_(AHB2PERIPH_BASE + GET_PORT(fast_io_pin) * 0x400),
          pin_(1 << GET_PIN_IDX(fast_io_pin)),
          port_num_(GET_PORT(fast_io_pin)) {}

    void set_mode(uint32_t ulMode) const {
        GPIO_InitTypeDef GPIO_InitStructure;
        GPIO_InitStructure.Pin = pin_;
        GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;

        switch ( ulMode ) {
        case INPUT:
            GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
            GPIO_InitStructure.Pull = GPIO_NOPULL;
            break;
        case INPUT_PULLUP:
            GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
            GPIO_InitStructure.Pull = GPIO_PULLUP;
            break;
        case INPUT_PULLDOWN:
            GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
            GPIO_InitStructure.Pull = GPIO_PULLDOWN;
            break;
        case OUTPUT:
            GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
            GPIO_InitStructure.Pull = GPIO_NOPULL;
            break;
        default:
            return;
        }
        set_GPIO_Port_Clock(port_num_);
        HAL_GPIO_Init(
            reinterpret_cast<GPIO_TypeDef*>(port_addr_),
            &GPIO_InitStructure);
    }

    void set() const {
        volatile GPIO_TypeDef* gpio_port = reinterpret_cast<GPIO_TypeDef*>(port_addr_);
        gpio_port->BSRR = pin_;
    }

    void reset() const {
        volatile GPIO_TypeDef* const gpio_port = reinterpret_cast<GPIO_TypeDef*>(port_addr_);
        gpio_port->BRR = pin_;
    }

    void toggle() const {
        volatile GPIO_TypeDef* const gpio_port = reinterpret_cast<GPIO_TypeDef*>(port_addr_);
        gpio_port->ODR ^= pin_;
    }

    void write(uint8_t val) const {
        if (val) {
            set();
        } else {
            reset();
        }
    }

    uint8_t read() const {
        volatile  GPIO_TypeDef* gpio_port = reinterpret_cast<GPIO_TypeDef*>(port_addr_);
        return (gpio_port->IDR & pin_) ? 1 : 0;
    }
private:
    void set_GPIO_Port_Clock(uint8_t port_num) const {
        switch(port_num) {
#ifdef GPIOA_BASE
        case PORTA:
            __HAL_RCC_GPIOA_CLK_ENABLE();
            break;
#endif            
#ifdef GPIOB_BASE
        case PORTB:
            __HAL_RCC_GPIOB_CLK_ENABLE();
            break;
#endif            
#ifdef GPIOC_BASE
        case PORTC:
            __HAL_RCC_GPIOC_CLK_ENABLE();
            break;
#endif            
#ifdef GPIOD_BASE
        case PORTD:
            __HAL_RCC_GPIOD_CLK_ENABLE();
            break;
#endif            
#ifdef GPIOE_BASE
        case PORTE:
            __HAL_RCC_GPIOE_CLK_ENABLE();
            break;
#endif            
#ifdef GPIOF_BASE
        case PORTF:
            __HAL_RCC_GPIOF_CLK_ENABLE();
            break;
#endif            
        }
    }
};
