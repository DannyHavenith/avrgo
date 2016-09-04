#AVRGo, fast AVR core emulator

This is an experiment. This software uses C++ template metaprogramming to create a very flexible
emulator of the Atmel AVR processor core.

The central idea is to employ meta programming to create optimal emulator code and to use static
polymorphism to avoid the cost of run-time polymorphism.

Much of the template metaprogramming is in the instruction decoder. AVR core instructions are all
implemented as overloads of the `execute()` member function of the avr_core class.
