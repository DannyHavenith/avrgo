//
//  Copyright (C) 2016 Danny Havenith
//
//  Distributed under the Boost Software License, Version 1.0. (See
//  accompanying file LICENSE_1_0.txt or copy at
//  http://www.boost.org/LICENSE_1_0.txt)
//
//  Functions to pre-compile AVR machine code instructions into
//  structs with decoded arguments and function pointers.
//
//  An optimized emulator will first pre-compile a complete flash
//  image into such structs, so that it can quickly execute each instruction
//  without having to decode the instruction every time.
//
//  The precompiler uses the exact same instruction decoder that is also used to
//  execute AVR instructions on-the-fly.
//
//
#ifndef AVRSIM_PRECOMPILER_HPP_
#define AVRSIM_PRECOMPILER_HPP_
#include <cstdint>
#include "avr_instruction_set.hpp"
#include "instruction.hpp"
#include "decoder.hpp"

namespace avrsim
{
    template<typename Core>
    using precompiled_function = void (Core &core, uint16_t arg1, uint16_t arg2);

    template<typename Core>
    struct precompiled_instruction
    {
        precompiled_function<Core> *f;
        uint16_t arg1;
        uint16_t arg2;
    };

    template<
        typename Core,
        typename Instruction,
        int size = mpl::size<
            typename operands<
                Instruction,
                instructions::avr_operands
            >::type
         >::type::value
    >
    struct precompiled_function_implementation
    {
    };

    template< typename Core, typename Instruction>
    struct precompiled_function_implementation< Core, Instruction, 0>
    {
        static void execute(Core &core, uint16_t, uint16_t)
        {
            core.execute( Instruction{});
        }
    };

    template< typename Core, typename Instruction>
    struct precompiled_function_implementation< Core, Instruction, 1>
    {
        static void execute(Core &core, uint16_t arg1, uint16_t)
        {
            core.execute( Instruction{}, arg1);
        }
    };

    template< typename Core, typename Instruction>
    struct precompiled_function_implementation< Core, Instruction, 2>
    {
        static void execute(Core &core, uint16_t arg1, uint16_t arg2)
        {
            core.execute( Instruction{}, arg1, arg2);
        }
    };

    template< typename Core>
    class precompiler
    {
    public:
        template< typename Instruction>
        void execute( Instruction, uint16_t arg1 = 0, uint16_t arg2 = 0)
        {
            result = {
                &precompiled_function_implementation< Core, Instruction>::execute,
                arg1,
                arg2
            };
        }

        precompiled_instruction<Core> get_result() const
        {
            return result;
        }

    private:
        precompiled_instruction<Core> result;
    };

    /**
     * The actual pre-compilation function.
     *
     *  Given an instruction word and
     *  a AVR Core implementation type, this will return a struct with
     *  the decoded arguments and a pointer to a function that will call
     *  the right member function of the AVR Core implementation object.
     */
    template< typename Core>
    precompiled_instruction<Core> precompile( uint16_t instruction)
    {
        // the trick used here is that a precompiler is also a Core implementation
        // (it implements execute() for all Instruction types).
        // The decoder will call the execute member functions of the decoder
        // instead of those of an actual AVR core.
        // The decoder will generate a struct with function pointer and arguments
        // in its execute() member function, which is then harvested and
        // returned.
        using decoder
            = typename find_decoder< precompiler<Core>, instructions::list>::type;
        precompiler<Core> compiler;
        decoder::decode_and_execute( compiler, instruction);
        return compiler.get_result();
    }

}
#endif /* AVRSIM_PRECOMPILER_HPP_ */
