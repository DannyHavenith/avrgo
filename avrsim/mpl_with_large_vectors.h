//
//  Copyright (C) 2016 Danny Havenith
//
//  Distributed under the Boost Software License, Version 1.0. (See
//  accompanying file LICENSE_1_0.txt or copy at
//  http://www.boost.org/LICENSE_1_0.txt)
//


/**
 * This file will include the necessary mpl headers, but first
 * sets the right preprocessor symbols so that boost.mpl will not use
 * preprocessed headers and will allow large vectors.
 *
 * This is done because the preprocessor symbols need to be set before including
 * any mpl header file and it is quite tedious having to do this in each source file.
 */
#ifndef AVRSIM_MPL_WITH_LARGE_VECTORS_H_
#define AVRSIM_MPL_WITH_LARGE_VECTORS_H_

#define BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS
#define BOOST_MPL_LIMIT_VECTOR_SIZE 50

#include <boost/mpl/size.hpp>
#include <boost/mpl/front.hpp>
#include <boost/mpl/copy.hpp>
#include <boost/mpl/copy_if.hpp>
#include <boost/mpl/front_inserter.hpp>
#include <boost/mpl/back_inserter.hpp>
#include <boost/mpl/pop_front.hpp>
#include <boost/mpl/push_back.hpp>
#include <boost/mpl/list.hpp>
#include <boost/mpl/eval_if.hpp>
#include <boost/mpl/empty.hpp>
#include <boost/mpl/or.hpp>
#include <boost/mpl/count_if.hpp>
#include <boost/mpl/vector_c.hpp>
#include <boost/mpl/at.hpp>
#include <boost/mpl/int.hpp>
#include <boost/mpl/greater.hpp>
#include <boost/mpl/contains.hpp>
#include <boost/mpl/vector.hpp>






#endif /* AVRSIM_MPL_WITH_LARGE_VECTORS_H_ */
