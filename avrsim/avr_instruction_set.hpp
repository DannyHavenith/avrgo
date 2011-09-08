/*
 * avr_instruction_set.hpp
 *
 * List of AVR instructions with their associated bit patterns.
 * Derived from a text file: AVR opcodes by Jeremy Brandon, as found
 * on avrfreaks.
 *
 *  Created on: Jul 25, 2011
 *      Author: danny
 */

#ifndef AVR_INSTRUCTION_SET_HPP_
#define AVR_INSTRUCTION_SET_HPP_
#include <boost/mpl/joint_view.hpp>
#include <boost/mpl/vector.hpp>
#include "instruction.hpp"
#include "avr_operand_types.hpp"

namespace avrsim { namespace instructions {

// below are the individual AVR instructions and their bit patterns.
// note that these bit patterns are used by the instruction decoder and that
// the decoder uses no other knowledge of AVR instruction bit patterns.
struct NOP      : instruction< 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0> {};
struct MOVW     : instruction< 0,0,0,0,0,0,0,1,d,d,d,d,r,r,r,r> {};
struct MULS     : instruction< 0,0,0,0,0,0,1,0,d,d,d,d,r,r,r,r> {};
struct MULSU    : instruction< 0,0,0,0,0,0,1,1,0,d,d,d,0,r,r,r> {};
struct FMUL     : instruction< 0,0,0,0,0,0,1,1,0,d,d,d,1,r,r,r> {};
struct FMULS    : instruction< 0,0,0,0,0,0,1,1,1,d,d,d,0,r,r,r> {};
struct FMULSU   : instruction< 0,0,0,0,0,0,1,1,1,d,d,d,1,r,r,r> {};
struct CPC      : instruction< 0,0,0,0,0,1,r,d,d,d,d,d,r,r,r,r> {};
struct SBC      : instruction< 0,0,0,0,1,0,r,d,d,d,d,d,r,r,r,r> {};
struct ADD      : instruction< 0,0,0,0,1,1,r,d,d,d,d,d,r,r,r,r> {};
struct CPSE     : instruction< 0,0,0,1,0,0,r,d,d,d,d,d,r,r,r,r> {};
struct CP       : instruction< 0,0,0,1,0,1,r,d,d,d,d,d,r,r,r,r> {};
struct SUB      : instruction< 0,0,0,1,1,0,r,d,d,d,d,d,r,r,r,r> {};
struct ADC      : instruction< 0,0,0,1,1,1,r,d,d,d,d,d,r,r,r,r> {};
struct AND      : instruction< 0,0,1,0,0,0,r,d,d,d,d,d,r,r,r,r> {};
struct EOR      : instruction< 0,0,1,0,0,1,r,d,d,d,d,d,r,r,r,r> {};
struct OR       : instruction< 0,0,1,0,1,0,r,d,d,d,d,d,r,r,r,r> {};
struct MOV      : instruction< 0,0,1,0,1,1,r,d,d,d,d,d,r,r,r,r> {};
struct CPI      : instruction< 0,0,1,1,k,k,k,k,d,d,d,d,k,k,k,k> {};
struct SBCI     : instruction< 0,1,0,0,k,k,k,k,d,d,d,d,k,k,k,k> {};
struct SUBI     : instruction< 0,1,0,1,k,k,k,k,d,d,d,d,k,k,k,k> {};
struct ORI 		: instruction< 0,1,1,0,k,k,k,k,d,d,d,d,k,k,k,k> {};
struct ANDI 	: instruction< 0,1,1,1,k,k,k,k,d,d,d,d,k,k,k,k> {};
struct LDD_Y    : instruction< 1,0,q,0,q,q,0,d,d,d,d,d,1,q,q,q> {};
struct LDD_Z 	: instruction< 1,0,q,0,q,q,0,d,d,d,d,d,0,q,q,q> {};
struct STD_Z 	: instruction< 1,0,q,0,q,q,1,d,d,d,d,d,0,q,q,q> {};
struct STD_Y 	: instruction< 1,0,q,0,q,q,1,d,d,d,d,d,1,q,q,q> {};
struct LDS 	    : instruction< 1,0,0,1,0,0,0,d,d,d,d,d,0,0,0,0> {};
struct LD_Z_inc : instruction< 1,0,0,1,0,0,0,d,d,d,d,d,0,0,0,1> {};
struct LD_Z_dec : instruction< 1,0,0,1,0,0,0,d,d,d,d,d,0,0,1,0> {};
struct LD_Z_min : instruction< 1,0,0,1,0,0,0,d,d,d,d,d,0,0,1,1> {};
struct LPM_Z    : instruction< 1,0,0,1,0,0,0,d,d,d,d,d,0,1,0,0> {};
struct LPM_Z_inc: instruction< 1,0,0,1,0,0,0,d,d,d,d,d,0,1,0,1> {};
struct ELPM_Z   : instruction< 1,0,0,1,0,0,0,d,d,d,d,d,0,1,1,0> {};
struct ELPM_Z_inc:instruction< 1,0,0,1,0,0,0,d,d,d,d,d,0,1,1,1> {};
struct LD_Y_inc : instruction< 1,0,0,1,0,0,0,d,d,d,d,d,1,0,0,1> {};
struct LD_Y_dec : instruction< 1,0,0,1,0,0,0,d,d,d,d,d,1,0,1,0> {};
struct LD_Y_min : instruction< 1,0,0,1,0,0,0,d,d,d,d,d,1,0,1,1> {};
struct LD_Y     : instruction< 1,0,0,1,0,0,0,d,d,d,d,d,1,0,0,0> {};
struct LD_X_inc : instruction< 1,0,0,1,0,0,0,d,d,d,d,d,1,1,0,1> {};
struct LD_X_dec : instruction< 1,0,0,1,0,0,0,d,d,d,d,d,1,1,1,0> {};
struct LD_X_min : instruction< 1,0,0,1,0,0,0,d,d,d,d,d,1,1,1,1> {};
struct LD_X     : instruction< 1,0,0,1,0,0,0,d,d,d,d,d,1,1,0,0> {};
struct POP 		: instruction< 1,0,0,1,0,0,0,d,d,d,d,d,1,1,1,1> {};
struct STS   	: instruction< 1,0,0,1,0,0,1,r,r,r,r,r,0,0,0,0> {};
struct ST_Z_inc : instruction< 1,0,0,1,0,0,1,r,r,r,r,r,0,0,0,1> {};
struct ST_Z_dec : instruction< 1,0,0,1,0,0,1,r,r,r,r,r,0,0,1,0> {};
struct ST_Z_min : instruction< 1,0,0,1,0,0,1,r,r,r,r,r,0,0,1,1> {};
struct unk1		: instruction< 1,0,0,1,0,0,1,r,r,r,r,r,0,1,x,x> {};
struct ST_Y_inc : instruction< 1,0,0,1,0,0,1,r,r,r,r,r,1,0,0,1> {};
struct ST_Y_dec : instruction< 1,0,0,1,0,0,1,r,r,r,r,r,1,0,1,0> {};
struct ST_Y_min : instruction< 1,0,0,1,0,0,1,r,r,r,r,r,1,0,1,1> {};
struct ST_Y     : instruction< 1,0,0,1,0,0,1,r,r,r,r,r,1,0,0,0> {};
struct ST_X_inc : instruction< 1,0,0,1,0,0,1,r,r,r,r,r,1,1,0,1> {};
struct ST_X_dec : instruction< 1,0,0,1,0,0,1,r,r,r,r,r,1,1,1,0> {};
struct ST_X_min : instruction< 1,0,0,1,0,0,1,r,r,r,r,r,1,1,1,1> {};
struct ST_X     : instruction< 1,0,0,1,0,0,1,r,r,r,r,r,1,1,0,0> {};
struct PUSH 	: instruction< 1,0,0,1,0,0,1,d,d,d,d,d,1,1,1,1> {};
struct COM 		: instruction< 1,0,0,1,0,1,0,d,d,d,d,d,0,0,0,0> {};
struct NEG 		: instruction< 1,0,0,1,0,1,0,d,d,d,d,d,0,0,0,1> {};
struct SWAP 	: instruction< 1,0,0,1,0,1,0,d,d,d,d,d,0,0,1,0> {};
struct INC 		: instruction< 1,0,0,1,0,1,0,d,d,d,d,d,0,0,1,1> {};
struct unk2		: instruction< 1,0,0,1,0,1,0,d,d,d,d,d,0,1,0,0> {};
struct ASR 		: instruction< 1,0,0,1,0,1,0,d,d,d,d,d,0,1,0,1> {};
struct LSR 		: instruction< 1,0,0,1,0,1,0,d,d,d,d,d,0,1,1,0> {};
struct ROR 		: instruction< 1,0,0,1,0,1,0,d,d,d,d,d,0,1,1,1> {};
struct DEC 		: instruction< 1,0,0,1,0,1,0,d,d,d,d,d,1,0,1,0> {};
struct BSET 	: instruction< 1,0,0,1,0,1,0,0,0,s,s,s,1,0,0,0> {};
struct BCLR 	: instruction< 1,0,0,1,0,1,0,0,1,s,s,s,1,0,0,0> {};
struct IJMP 	: instruction< 1,0,0,1,0,1,0,0,0,0,0,0,1,0,0,1> {};
struct EIJMP 	: instruction< 1,0,0,1,0,1,0,0,0,0,0,1,1,0,0,1> {};
struct RET 		: instruction< 1,0,0,1,0,1,0,1,0,0,0,0,1,0,0,0> {};
struct ICALL 	: instruction< 1,0,0,1,0,1,0,1,0,0,0,0,1,0,0,1> {};
struct RETI 	: instruction< 1,0,0,1,0,1,0,1,0,0,0,1,1,0,0,0> {};
struct EICALL 	: instruction< 1,0,0,1,0,1,0,1,0,0,0,1,1,0,0,1> {};
struct SLEEP 	: instruction< 1,0,0,1,0,1,0,1,1,0,0,0,1,0,0,0> {};
struct BREAK 	: instruction< 1,0,0,1,0,1,0,1,1,0,0,1,1,0,0,0> {};
struct WDR 		: instruction< 1,0,0,1,0,1,0,1,1,0,1,0,1,0,0,0> {};
struct LPM 		: instruction< 1,0,0,1,0,1,0,1,1,1,0,0,1,0,0,0> {};
struct ELPM 	: instruction< 1,0,0,1,0,1,0,1,1,1,0,1,1,0,0,0> {};
struct SPM 		: instruction< 1,0,0,1,0,1,0,1,1,1,1,0,1,0,0,0> {};
struct ESPM 	: instruction< 1,0,0,1,0,1,0,1,1,1,1,1,1,0,0,0> {};
struct JMP   	: instruction< 1,0,0,1,0,1,0,k,k,k,k,k,1,1,0,k> {};
struct CALL   	: instruction< 1,0,0,1,0,1,0,k,k,k,k,k,1,1,1,k> {};
struct ADIW 	: instruction< 1,0,0,1,0,1,1,0,k,k,d,d,k,k,k,k> {};
struct SBIW 	: instruction< 1,0,0,1,0,1,1,1,k,k,d,d,k,k,k,k> {};
struct CBI 		: instruction< 1,0,0,1,1,0,0,0,A,A,A,A,A,b,b,b> {};
struct SBIC 	: instruction< 1,0,0,1,1,0,0,1,A,A,A,A,A,b,b,b> {};
struct SBI 		: instruction< 1,0,0,1,1,0,1,0,A,A,A,A,A,b,b,b> {};
struct SBIS 	: instruction< 1,0,0,1,1,0,1,1,A,A,A,A,A,b,b,b> {};
struct MUL 		: instruction< 1,0,0,1,1,1,r,d,d,d,d,d,r,r,r,r> {};
struct IN 		: instruction< 1,0,1,1,0,A,A,d,d,d,d,d,A,A,A,A> {};
struct OUT 		: instruction< 1,0,1,1,1,A,A,r,r,r,r,r,A,A,A,A> {};
struct RJMP 	: instruction< 1,1,0,0,k,k,k,k,k,k,k,k,k,k,k,k> {};
struct RCALL 	: instruction< 1,1,0,1,k,k,k,k,k,k,k,k,k,k,k,k> {};
struct LDI 		: instruction< 1,1,1,0,k,k,k,k,d,d,d,d,k,k,k,k> {};
struct BRBS 	: instruction< 1,1,1,1,0,0,k,k,k,k,k,k,k,s,s,s> {};
struct BRBC 	: instruction< 1,1,1,1,0,1,k,k,k,k,k,k,k,s,s,s> {};
struct BLD 		: instruction< 1,1,1,1,1,0,0,d,d,d,d,d,0,b,b,b> {};
struct BST 		: instruction< 1,1,1,1,1,0,1,d,d,d,d,d,0,b,b,b> {};
struct SBRC 	: instruction< 1,1,1,1,1,1,0,r,r,r,r,r,0,b,b,b> {};
struct SBRS  	: instruction< 1,1,1,1,1,1,1,r,r,r,r,r,0,b,b,b> {};

// extra instruction tags, for instructions that require an extra word
struct LDS_direct {};
struct STS_direct {};

// create a list that enumerates all avr instructions
typedef boost::mpl::vector<
        NOP,
        MOVW,
        MULS,
        MULSU,
        FMUL,
        FMULS,
        FMULSU,
        CPC,
        SBC,
        ADD,
        CPSE,
        CP,
        SUB,
        ADC,
        AND,
        EOR,
        OR,
        MOV,
        CPI,
        SBCI,
        SUBI,
        ORI,
        ANDI,
        LDD_Z,
        LDD_Y,
        STD_Z,
        STD_Y,
        LDS,
        LD_Z_inc,
        LD_Z_dec,
        LD_Z_min,
        LPM_Z,
        ELPM_Z,
        LD_Y_inc,
        LD_Y_dec,
        LD_Y_min,
        LD_Y,
        LD_X,
        POP,
        STS,
        ST_Z_inc,
        ST_Z_dec,
        ST_Z_min
        > list1;

typedef boost::mpl::vector<
        ST_Y_inc,
        ST_Y_dec,
        ST_Y_min,
        ST_Y,
        ST_X,
        PUSH,
        COM,
        NEG,
        SWAP,
        INC,
        ASR,
        LSR,
        ROR,
        DEC,
        BSET,
        BCLR,
        IJMP,
        EIJMP,
        RET,
        ICALL,
        RETI,
        EICALL,
        SLEEP,
        BREAK,
        WDR,
        LPM,
        ELPM,
        SPM,
        ESPM,
        JMP,
        CALL,
        ADIW,
        SBIW,
        CBI
        > list2;

typedef boost::mpl::vector<
        SBIC,
        SBI,
        SBIS,
        MUL,
        IN,
        OUT,
        RJMP,
        RCALL,
        LDI,
        BRBS,
        BRBC,
        BLD,
        BST,
        SBRC,
        SBRS
        > list3;


// todo: balance this
/// list of all known avr instructions.
/// This is a type-sequence, in which each element represents an AVR instruction.
typedef boost::mpl::joint_view< list1, mpl::joint_view<list2, list3> > list;

}}


#endif /* AVR_INSTRUCTION_SET_HPP_ */
