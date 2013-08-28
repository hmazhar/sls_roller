#ifndef HAMMADMODELS_COMMON_H
#define HAMMADMODELS_COMMON_H
#include <vector>
#include "lcp/CHlcpVariablesGeneric.h"
#include "lcp/CHlcpVariablesBody.h"
#include "lcp/CHlcpConstraintTwoGeneric.h"
#include "lcp/CHlcpSystemDescriptor.h"
#include "lcp/CHlcpIterativeSOR.h"
#include "lcp/CHlcpIterativePMINRES.h"
#include "lcp/CHlcpIterativeAPGD.h"
#include "lcp/CHlcpIterativeBB.h"
#include "lcp/CHlcpSimplexSolver.h"
#include "core/CHlinearAlgebra.h"
#include "physics/CHsystemOpenMP.h"
#include "assets/CHsphereShape.h"
#include "physics/CHapidll.h"
#include "physics/CHsystem.h"
#include "lcp/CHlcpIterativeMINRES.h"
#include "core/CHrealtimeStep.h"
#include "lcp/CHlcpIterativeSOR.h"
#include "lcp/CHlcpIterativeJacobi.h"
#include "collision/CHcModelBullet.h"
#include "collision/CHcCollisionSystemBullet.h"
#include "physics/CHcontactContainer.h"
#include "unit_OPENGL/CHopenGL.h"
#include "unit_GPU/CHsystemGPU.h"
#include "unit_GPU/CHlcpSystemDescriptorGPU.h"
#include "unit_POSTPROCESS/CHmitsubaRender.h"
using namespace chrono;
using namespace postprocess;
using namespace geometry;
using namespace std;

#define PI 3.14159265359

#endif
