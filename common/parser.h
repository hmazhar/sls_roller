#include "common.h"

////Example Input File
/*
 timestep:				.01
 solver:					APGD
 solver_max_iterations:	100
 solver_tolerance:		1e-3
 compliance:				0 0 0
 ----------------------------
 contact_recovery_speed: .6
 contact_envelope:		.005
 collision_BPA:			40 15 40
 collision_BPB:			100 50

 */

void setSolverGPU(string solver, ChSystemParallel* m_system) {
	if (solver == "APGD") {
		((ChLcpSolverParallel *) (m_system->GetLcpSolverSpeed()))->SetSolverType(ACCELERATED_PROJECTED_GRADIENT_DESCENT);
	} else if (solver == "JACOBI") {
		((ChLcpSolverParallel *) (m_system->GetLcpSolverSpeed()))->SetSolverType(BLOCK_JACOBI);
	} else if (solver == "CG") {
		((ChLcpSolverParallel *) (m_system->GetLcpSolverSpeed()))->SetSolverType(CONJUGATE_GRADIENT);
	} else if (solver == "CGS") {
		((ChLcpSolverParallel *) (m_system->GetLcpSolverSpeed()))->SetSolverType(CONJUGATE_GRADIENT_SQUARED);
	} else if (solver == "BICG") {
		((ChLcpSolverParallel *) (m_system->GetLcpSolverSpeed()))->SetSolverType(BICONJUGATE_GRADIENT);
	} else if (solver == "BICGSTAB") {
		((ChLcpSolverParallel *) (m_system->GetLcpSolverSpeed()))->SetSolverType(BICONJUGATE_GRADIENT_STAB);
	} else if (solver == "GD") {
		((ChLcpSolverParallel *) (m_system->GetLcpSolverSpeed()))->SetSolverType(GRADIENT_DESCENT);
	} else if (solver == "SD") {
		((ChLcpSolverParallel *) (m_system->GetLcpSolverSpeed()))->SetSolverType(STEEPEST_DESCENT);
	} else if (solver == "MINRES") {
		((ChLcpSolverParallel *) (m_system->GetLcpSolverSpeed()))->SetSolverType(MINIMUM_RESIDUAL);
	}
}


void ReadInputFile(string input, ChSystemParallel* m_system) {
	ifstream ifile(input.c_str());
	if (ifile.fail()) {
		cout << "CONFIG FILE FAIL" << endl;
	}

	string token, value_string;
	real value_real;
	int value_int;
	Vector value_vect;
	while (ifile.fail() == false) {

		ifile >> token;
		cout << "token " << token;
//--------------------------------------------------------------------------------
		if (token == "timestep:") {
			ifile >> value_real;
			m_system->SetStep(value_real);
		}
//--------------------------------------------------------------------------------
		if (token == "solver:") {
			ifile >> value_string;
			setSolverGPU(value_string,m_system);
		}
//--------------------------------------------------------------------------------
		if (token == "solver_max_iterations:") {
			ifile >> value_int;
			((ChLcpSolverParallel *) (m_system->GetLcpSolverSpeed()))->SetMaxIteration(value_int);
			m_system->SetMaxiter(value_int);
			m_system->SetIterLCPmaxItersSpeed(value_int);
		}
//--------------------------------------------------------------------------------
		if (token == "solver_tolerance:") {
			ifile >> value_real;
			m_system->SetTol(value_real);
			m_system->SetTolSpeeds(value_real);
			((ChLcpSolverParallel *) (m_system->GetLcpSolverSpeed()))->SetTolerance(value_real);
		}
//--------------------------------------------------------------------------------
		if (token == "compliance:") {
			real compliance, compliance_T, alpha;
			ifile >> compliance >> compliance_T >> alpha;
			((ChLcpSolverParallel *) (m_system->GetLcpSolverSpeed()))->SetCompliance(alpha);
		}
//--------------------------------------------------------------------------------
		if (token == "contact_recovery_speed:") {
			ifile >> value_real;
			((ChLcpSolverParallel *) (m_system->GetLcpSolverSpeed()))->SetContactRecoverySpeed(value_real);
		}
//--------------------------------------------------------------------------------
		if (token == "contact_envelope:") {
			ifile >> value_real;
			((ChCollisionSystemParallel *) (m_system->GetCollisionSystem()))->SetCollisionEnvelope(value_real);
		}
//--------------------------------------------------------------------------------
		if (token == "collision_BPA:") {
			int x, y, z;
			ifile >> x >> y >> z;
			((ChCollisionSystemParallel *) (m_system->GetCollisionSystem()))->setBinsPerAxis(I3(x, y, z));
		}
//--------------------------------------------------------------------------------
		if (token == "collision_BPB:") {
			int min_b, max_b;
			ifile >> max_b >> min_b;
			((ChCollisionSystemParallel *) (m_system->GetCollisionSystem()))->setBodyPerBin(max_b, min_b);
		}
//--------------------------------------------------------------------------------
	}
}


