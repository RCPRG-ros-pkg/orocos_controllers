/*
 * controller_common.cpp
 *
 *  Created on: 25 sty 2014
 *      Author: konrad
 */
#include <rtt/Component.hpp>

#include "controller_common/vector_concate.h"
#include "controller_common/vector_split.h"
#include "controller_common/cartesian_impedance.h"


typedef VectorConcate<2> VectorConcate2;
typedef VectorConcate<3> VectorConcate3;
typedef VectorConcate<4> VectorConcate4;

ORO_LIST_COMPONENT_TYPE(VectorConcate2)
ORO_LIST_COMPONENT_TYPE(VectorConcate3)
ORO_LIST_COMPONENT_TYPE(VectorConcate4)

typedef VectorSplit<2> VectorSplit2;
typedef VectorSplit<3> VectorSplit3;
typedef VectorSplit<4> VectorSplit4;

ORO_LIST_COMPONENT_TYPE(VectorSplit2)
ORO_LIST_COMPONENT_TYPE(VectorSplit3)
ORO_LIST_COMPONENT_TYPE(VectorSplit4)

typedef CartesianImpedance<6, 1> CartesianImpedance6;
typedef CartesianImpedance<7, 1> CartesianImpedance7;

ORO_LIST_COMPONENT_TYPE(CartesianImpedance6)
ORO_LIST_COMPONENT_TYPE(CartesianImpedance7)

ORO_CREATE_COMPONENT_LIBRARY()

