/*
 * avr_operand_types.hpp
 *
 *  Created on: Jul 31, 2011
 *      Author: danny
 */

#ifndef AVR_OPERAND_TYPES_HPP_
#define AVR_OPERAND_TYPES_HPP_
#include <boost/mpl/vector_c.hpp>
namespace avrsim {
namespace instructions {
// the operand codes are used to encode the different operand bits
// in an instruction word. Their actual value does not matter, as long as
// they're all different and are not 0 or 1.
const int d = 2;
const int r = 3;
const int k = 4;
const int q = 5;
const int A = 6;
const int s = 7;
const int x = 8;
const int b = 9;

/// Sorted list of possible AVR operands.
/// The order of operands in this list also determines the order of operand
/// arguments to the execute() methods.
typedef boost::mpl::vector_c< int, d, r, k, q, A, s, b, x> avr_operands;
}
}


#endif /* AVR_OPERAND_TYPES_HPP_ */
