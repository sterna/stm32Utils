# stm32Utils
Various useful extra utility functions for (primarily for stm32)

This repo contains various extra functions useful for any MCU. Some are specifically targeted to STM32, using the old stdperiph library (the new ST HAL sucks >:( ).
Functions include:
* Time and delay functions (using the ARM cortex-M Systick)
* Abstracted setting functions for stdperiph
* Various conversion functions (explicit data conversion, ADC-conversions)
* Simple math functions (various straight line functions)
* Specific counter functions

It also includes the whole stdperiph v3.5, newlib, and core funtionality.
