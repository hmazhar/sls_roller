#ifndef CHRONOMODELS_COMMON_H
#define CHRONOMODELS_COMMON_H


//Core level includes
#include "core/ChLinearAlgebra.h"
#include "core/ChRealtimeStep.h"

//System level Includes
#include "physics/ChSystem.h"
#include "physics/ChSystemOpenMP.h"
#include "physics/ChApidll.h"
#include "physics/ChContactContainer.h"

//Include different LCP solvers
#include "lcp/ChLcpVariablesGeneric.h"
#include "lcp/ChLcpVariablesBody.h"
#include "lcp/ChLcpConstraintTwoGeneric.h"
#include "lcp/ChLcpSystemDescriptor.h"
#include "lcp/ChLcpIterativeSOR.h"
#include "lcp/ChLcpIterativePMINRES.h"
#include "lcp/ChLcpIterativeAPGD.h"
#include "lcp/ChLcpIterativeBB.h"
#include "lcp/ChLcpSimplexSolver.h"
#include "lcp/ChLcpIterativeSOR.h"
#include "lcp/ChLcpIterativeJacobi.h"
#include "lcp/ChLcpIterativeMINRES.h"

//Include bullet collision model
#include "collision/ChCModelBullet.h"
#include "collision/ChCCollisionSystemBullet.h"

//Include Parallel system and descriptor
#include "ChSystemParallel.h"
#include "ChLcpSystemDescriptorParallel.h"


//Enable OpenGL visualizer
#include "ChOpenGL.h"

//Enable namespaces
using namespace chrono;
using namespace geometry;
using namespace std;

//Enable Mitsuba Render postprocessor
//#include "unit_POSTPROCESS/ChMitsubaRender.h"
//using namespace postprocess;


#endif
