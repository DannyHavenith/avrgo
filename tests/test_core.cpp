//
//  Copyright (C) 2016 Danny Havenith
//
//  Distributed under the Boost Software License, Version 1.0. (See
//  accompanying file LICENSE_1_0.txt or copy at
//  http://www.boost.org/LICENSE_1_0.txt)
//
#include "avrsim/avr_core.hpp"
#include "avrsim/avr_instruction_set.hpp"
#include "avrsim/decoder.hpp"
#include "avrsim/list_parser.hpp"
#include <gtest/gtest.h>

using namespace avrsim;

class CoreTest : public ::testing::Test
{
protected:
    avr_core core{512};
    using decoder = typename find_decoder< avr_core,instructions::list>::type;

    void Execute( const std::string &instruction)
    {
        std::stringstream stream{instruction};
        auto listing = parse_listing( stream);
        core.rom = listing.rom;
        core.pc = 0;
        decoder::decode_and_execute( core, core.fetch_instruction_word());
    }

    void Execute(uint16_t instruction)
    {
        core.rom.resize(2);
        core.rom[0] = instruction;
        core.pc = 0;
        decoder::decode_and_execute( core, core.fetch_instruction_word());
    }
};

TEST_F( CoreTest, skip_skips_2_words)
{
    core.r[19] = 0x80;

    Execute(
            "00:   37 ff           sbrs    r19, 7\n"
            "02:   30 93 96 01     sts 0x0196, r19"
            );

    ASSERT_EQ( 3, core.pc);

    Execute(
            "00:   37 ff           sbrs    r19, 7\n"
            "02:   48 e0           ldi r20, 0x08"
            );

    ASSERT_EQ( 2, core.pc);
}

TEST_F( CoreTest, st_x_inc)
{
    core.r[1] = 42;
    core.r[26] = 255;
    core.r[27] = 1;
    core.ram[0x1FF] = 1;
    core.pc = 0;
    core.flags = flags_t{};

    Execute( "00:   1d 92           st  X+, r1");

    EXPECT_EQ( 0, core.r[26]);
    EXPECT_EQ( 2, core.r[27]);
    EXPECT_EQ( 42, core.ram[0x1FF]);
    EXPECT_EQ( flags_t{}, core.flags);
}

TEST_F( CoreTest, LPM_Z_inc)
{
    core.r[0] = 1;
    core.r[30] = 255;
    core.r[31] = 1;
    core.rom.resize( 0x200);
    core.rom[0x1ff/2] = 0x4241;
    core.flags = flags_t{};

    Execute("00:  05 90           lpm r0, Z+");

    EXPECT_EQ( 0, core.r[30]);
    EXPECT_EQ( 2, core.r[31]);
    EXPECT_EQ( 0x42, core.r[0]);
    EXPECT_EQ( flags_t{}, core.flags);
}

TEST_F( CoreTest, LPM_Z)
{
    core.r[30] = 255;
    core.r[31] = 1;
    core.rom.resize( 0x200);
    core.rom[0x1ff/2] = 0x4241;
    core.flags = flags_t{};

    Execute("00:   f4 91           lpm r31, Z");

    EXPECT_EQ( 255, core.r[30]);
    EXPECT_EQ( 0x42, core.r[31]);
    EXPECT_EQ( flags_t{}, core.flags);

}

TEST_F( CoreTest, SBIW_happy_days)
{
    core.r[28] = 0x04;
    core.r[29] = 0x01;

    Execute("00:    22 97           sbiw    r28, 0x02");

    EXPECT_EQ( 0x02, core.r[28]);
    EXPECT_EQ( 0x01, core.r[29]);

    EXPECT_FALSE( !!core.flags.Z);
    EXPECT_FALSE( !!core.flags.C);

}

TEST_F( CoreTest, BREAK_throws_exception)
{
    ASSERT_THROW( Execute( 0b1001010110011000), avr_core::break_exception);
}

TEST_F( CoreTest, SBIW_byte_overflow)
{
    core.r[28] = 0x01;
    core.r[29] = 0x01;

    Execute("00:    22 97           sbiw    r28, 0x02");


    EXPECT_EQ( 0x00, core.r[27]);
    EXPECT_EQ( 0xff, core.r[28]);
    EXPECT_EQ( 0x00, core.r[29]);
    EXPECT_EQ( 0x00, core.r[30]);

    EXPECT_FALSE( !!core.flags.Z);
    EXPECT_FALSE( !!core.flags.C);
}

TEST_F( CoreTest, CPI_happy_days)
{
    core.r[26] = 0xDB;
    core.flags = flags_t{};

    Execute("00:    ab 3d           cpi r26, 0xDB");

    EXPECT_TRUE ( !!core.flags.Z);
    EXPECT_FALSE( !!core.flags.C);
    EXPECT_FALSE( !!core.flags.H);
}

TEST_F( CoreTest, CPR_halfcary)
{

    core.r[27] = 0x20;
    core.r[17] = 0x1F;
    core.flags = flags_t{};
    Execute("00:   b1 07           cpc r27, r17");

    EXPECT_TRUE(  !!core.flags.H);
    EXPECT_FALSE( !!core.flags.Z );
    EXPECT_FALSE( !!core.flags.S );
    EXPECT_FALSE( !!core.flags.N );
    EXPECT_FALSE( !!core.flags.C );
}

TEST_F( CoreTest, CPR_nozero_if_not_zero)
{

    core.r[27] = 0x20;
    core.r[17] = 0x1F;
    core.flags = flags_t{};
    core.flags.C = true;

    Execute("00:   b1 07           cpc r27, r17");

    EXPECT_FALSE( !!core.flags.H);
    EXPECT_FALSE( !!core.flags.Z);
    EXPECT_FALSE( !!core.flags.S);
    EXPECT_FALSE( !!core.flags.N);
    EXPECT_FALSE( !!core.flags.C);
}

TEST_F( CoreTest, CPR_zero_if_zero)
{

    core.r[27] = 0x20;
    core.r[17] = 0x1F;
    core.flags = flags_t{};
    core.flags.C = true;
    core.flags.Z = true;

    Execute("00:   b1 07           cpc r27, r17");

    EXPECT_FALSE( !!core.flags.H);
    EXPECT_TRUE(  !!core.flags.Z);
    EXPECT_FALSE( !!core.flags.S);
    EXPECT_FALSE( !!core.flags.N);
    EXPECT_FALSE( !!core.flags.C);
}

TEST_F( CoreTest, CPR_cary_if_cary)
{

    core.r[27] = 0x20;
    core.r[17] = 0x20;
    core.flags = flags_t{};
    core.flags.C = true;
    core.flags.Z = true;

    Execute("00:   b1 07           cpc r27, r17");

    EXPECT_TRUE ( !!core.flags.H);
    EXPECT_FALSE( !!core.flags.Z);
    EXPECT_FALSE( !!core.flags.S);
    EXPECT_TRUE ( !!core.flags.N);
    EXPECT_TRUE ( !!core.flags.C);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
