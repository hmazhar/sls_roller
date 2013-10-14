#include "common/common.h"
#include "common/input_output.h"
#include "common/parser.h"
#include "common/generation.h"
ChVector<> lpos(0, 0, 0);
ChQuaternion<> quat(1, 0, 0, 0);

//all dimensions are in millimeters, milligrams
real plate_height = -5;
real plate_thickness = .1;
real plate_radius =3;
real plate_friction = 1;

real particle_radius = .058 / 2.0;
real particle_std_dev = .015 / 2.0;
real particle_density = .93;
real particle_friction = 1;
real particle_rolling_friction = 1;
real particle_spinning_friction = 1;
real particle_cohesion = 0;
Vector particle_initial_vel = Vector(0, 0, 0);     //initial velocity

real container_width = plate_radius;
real container_thickness = .25;
real container_height = 1.0;
real wscale = 1;

real gravity = -9810;
real timestep = .00001;
real time_to_run = 6;
real current_time = 0;
int num_steps = time_to_run / timestep;

int particle_grid_x = 14;
int particle_grid_z = 14;

int particles_every = 100 * 4;     //add particles every n steps
int max_iteration = 30;
real tolerance = 0;

int particle_configuration = 0;
//0: single sphere
//1: two spheres joined together
//2: single ellipsoid

bool create_particle_plate = 0;
real particle_plate_dim = 6;
real plate_particle_radius = .075;
bool all_three_kinds = true;
GPUSOLVERTYPE solver = ACCELERATED_PROJECTED_GRADIENT_DESCENT;

string data_folder = "data";
real start_height = -1.5;
ParticleGenerator *layer_gen;

template<class T>
void RunTimeStep(T* mSys, const int frame) {
//&& frame * timestep < 5.5 && frame * timestep > .02
	if (frame % particles_every == 0) {
		layer_gen->addPerturbedVolumeMixture(
				R3(0, start_height, 0),
				I3(particle_grid_x, 1, particle_grid_z),
				R3(.1, .1, .1),
				R3(particle_initial_vel.x, particle_initial_vel.y, particle_initial_vel.z));
	}

}
int main(int argc, char* argv[]) {
	int threads = 8;

	if (argc == 10) {
		threads = (atoi(argv[1]));
		particle_density = atof(argv[2]);
		particle_radius = atof(argv[3]);
		particle_friction = atof(argv[4]);
		particle_rolling_friction = atof(argv[5]);
		particle_spinning_friction = atof(argv[6]);
		particle_cohesion = atof(argv[7]);
		plate_friction = atof(argv[8]);
		data_folder = argv[9];
		//all_three_kinds = atoi(argv[7]);
		//particle_configuration = atoi(argv[8]);
	}
	omp_set_num_threads(threads);


	cout << particle_density << " " << particle_radius << " " << particle_friction << " " << particle_rolling_friction << " " << particle_spinning_friction << " " << particle_cohesion << " "
			<< plate_friction << " " << data_folder << " " << endl;
	//=========================================================================================================
	//=========================================================================================================
	ChSystemParallel * system_gpu = new ChSystemParallel;
	ChCollisionSystemParallel *mcollisionengine = new ChCollisionSystemParallel();
	system_gpu->SetIntegrationType(ChSystem::INT_ANITESCU);
	//system_gpu->SetAABB(R3(-2,-5,-2), R3(2,5,2));

	//=========================================================================================================
	system_gpu->SetParallelThreadNumber(threads);
	system_gpu->SetMaxiter(max_iteration);
	system_gpu->SetIterLCPmaxItersSpeed(max_iteration);
	((ChLcpSolverParallel *) (system_gpu->GetLcpSolverSpeed()))->SetMaxIteration(max_iteration);
	system_gpu->SetTol(tolerance);
	system_gpu->SetTolSpeeds(tolerance);
	((ChLcpSolverParallel *) (system_gpu->GetLcpSolverSpeed()))->SetTolerance(tolerance);
	((ChLcpSolverParallel *) (system_gpu->GetLcpSolverSpeed()))->SetCompliance(0, 0, 0);
	((ChLcpSolverParallel *) (system_gpu->GetLcpSolverSpeed()))->SetContactRecoverySpeed(10);
	//setSolverGPU(solver_string, system_gpu);     //reads a string and sets the solver
	((ChLcpSolverParallel *) (system_gpu->GetLcpSolverSpeed()))->SetWarmStart(false);
	((ChLcpSolverParallel *) (system_gpu->GetLcpSolverSpeed()))->SetSolverType(solver);
	((ChCollisionSystemParallel *) (system_gpu->GetCollisionSystem()))->SetCollisionEnvelope(particle_radius * .06);
	mcollisionengine->setBinsPerAxis(R3(50, 50, 50));
	mcollisionengine->setBodyPerBin(100, 50);
	((ChSystemParallel*) system_gpu)->SetAABB(R3(-6, -6, -6), R3(6, 4, 6));

	//=========================================================================================================
	system_gpu->Set_G_acc(ChVector<>(0, gravity, 0));
	system_gpu->SetStep(timestep);
	//=========================================================================================================

	int num_particle = particle_plate_dim / plate_particle_radius;

	ChSharedPtr<ChMaterialSurface> material;
	material = ChSharedPtr<ChMaterialSurface>(new ChMaterialSurface);
	material->SetFriction(2-particle_friction);
	material->SetSpinningFriction(2-particle_spinning_friction);
	material->SetRollingFriction(2-particle_rolling_friction);

	if (create_particle_plate) {

		ChSharedBodyPtr sphere;
		sphere = ChSharedBodyPtr(new ChBody(new ChCollisionModelParallel));
		InitObject(sphere, 1, Vector(0, 0, 0), quat, material, true, true, -20, -20);
		for (int i = 0; i < num_particle; i++) {
			for (int j = 0; j < num_particle; j++) {

				ChVector<> position(
						i * plate_particle_radius * 1.2 - num_particle * plate_particle_radius * .6,
						plate_height + plate_thickness + plate_particle_radius,
						j * plate_particle_radius * 1.2 - num_particle * plate_particle_radius * .6);

				position.x += rand() % 10000 / 10000.0 * plate_particle_radius * .25 - plate_particle_radius * .25 * .5;
				position.y += rand() % 10000 / 10000.0 * plate_particle_radius * .5 - plate_particle_radius * .5 * .5;
				position.z += rand() % 10000 / 10000.0 * plate_particle_radius * .25 - plate_particle_radius * .25 * .5;

				AddCollisionGeometry(sphere, ELLIPSOID, ChVector<>(plate_particle_radius, plate_particle_radius * .6, plate_particle_radius), position, quat);
			}
		}
		FinalizeObject(sphere, (ChSystemParallel *) system_gpu);
	}
	ChSharedBodyPtr PLATE = ChSharedBodyPtr(new ChBody(new ChCollisionModelParallel));
	InitObject(PLATE, 1, ChVector<>(0, plate_height, 0), quat, material, true, true, -1000, -20000);
	AddCollisionGeometry(PLATE, BOX, ChVector<>(plate_radius, plate_thickness, plate_radius), lpos, quat);
	FinalizeObject(PLATE, (ChSystemParallel *) system_gpu);

	ChSharedBodyPtr L = ChSharedBodyPtr(new ChBody(new ChCollisionModelParallel));
	ChSharedBodyPtr R = ChSharedBodyPtr(new ChBody(new ChCollisionModelParallel));
	ChSharedBodyPtr F = ChSharedBodyPtr(new ChBody(new ChCollisionModelParallel));
	ChSharedBodyPtr B = ChSharedBodyPtr(new ChBody(new ChCollisionModelParallel));

	InitObject(L, 100000, Vector(-container_width + container_thickness, plate_height + container_height, 0), quat, material, true, true, -20, -20);
	InitObject(R, 100000, Vector(container_width - container_thickness, plate_height + container_height, 0), quat, material, true, true, -20, -20);
	InitObject(F, 100000, Vector(0, plate_height + container_height, -container_width + container_thickness), quat, material, true, true, -20, -20);
	InitObject(B, 100000, Vector(0, plate_height + container_height, container_width - container_thickness), quat, material, true, true, -20, -20);

	AddCollisionGeometry(L, BOX, Vector(container_thickness, container_height, container_width), lpos, quat);
	AddCollisionGeometry(R, BOX, Vector(container_thickness, container_height, container_width), lpos, quat);
	AddCollisionGeometry(F, BOX, Vector(container_width * wscale, container_height, container_thickness), lpos, quat);
	AddCollisionGeometry(B, BOX, Vector(container_width * wscale, container_height, container_thickness), lpos, quat);

	//FinalizeObject(L, (ChSystemParallel *) system_gpu);
	//FinalizeObject(R, (ChSystemParallel *) system_gpu);
	//FinalizeObject(F, (ChSystemParallel *) system_gpu);
	//FinalizeObject(B, (ChSystemParallel *) system_gpu);

	ChSharedBodyPtr Ring;

		for (int i = 0; i < 360; i += 5) {

			real angle = i * PI / 180.0;
			real x = cos(angle) * 1.5;
			real z = sin(angle) * 1.5;
			Quaternion q;
			q.Q_from_AngAxis(-angle, Vector(0, 1, 0));

			Ring = ChSharedBodyPtr(new ChBody(new ChCollisionModelParallel));
			InitObject(Ring, 100000, Vector(x, 2 +plate_height, z), q, material, true, true, -20, -20);

			AddCollisionGeometry(Ring, BOX, Vector(.1, 2, .062), Vector(0, 0, 0), Quaternion(1, 0, 0, 0));

			FinalizeObject(Ring, (ChSystemParallel *) system_gpu);
		}



	//==========

	real funnel_thickness = .05;
	real funnel_height = 4;
	real funnel_width = 1;
	real funnel_h = 2;
	real funnel_offset = funnel_width;     //.5*sqrt(2)*funnel_width-funnel_thickness*6+particle_radius*3;
	ChSharedBodyPtr Lt = ChSharedBodyPtr(new ChBody(new ChCollisionModelParallel));
	ChSharedBodyPtr Rt = ChSharedBodyPtr(new ChBody(new ChCollisionModelParallel));
	ChSharedBodyPtr Ft = ChSharedBodyPtr(new ChBody(new ChCollisionModelParallel));
	ChSharedBodyPtr Bt = ChSharedBodyPtr(new ChBody(new ChCollisionModelParallel));

	ChSharedBodyPtr F1 = ChSharedBodyPtr(new ChBody(new ChCollisionModelParallel));
	ChSharedBodyPtr F2 = ChSharedBodyPtr(new ChBody(new ChCollisionModelParallel));
	ChSharedBodyPtr F3 = ChSharedBodyPtr(new ChBody(new ChCollisionModelParallel));
	ChSharedBodyPtr F4 = ChSharedBodyPtr(new ChBody(new ChCollisionModelParallel));

	ChSharedPtr<ChMaterialSurface> material_funnel;
	material_funnel = ChSharedPtr<ChMaterialSurface>(new ChMaterialSurface);
	material_funnel->SetFriction(0);
	material_funnel->SetRollingFriction(0);
	material_funnel->SetSpinningFriction(0);

	material_funnel->SetCohesion(-10000);

	//ChSharedBodyPtr Ring;

//	for (int i = 0; i < 360; i += 5) {
//
//		real angle = i * PI / 180.0;
//		real x = cos(angle) * 1;
//		real z = sin(angle) * 1;
//		Quaternion q1, q2, q;
//		q1.Q_from_AngAxis(-angle, Vector(0, 1, 0));
//		q2.Q_from_AngAxis(-45 * PI / 180.0, Vector(0, 0, 1));
//		q = q1 % q2;
//
//		Ring = ChSharedBodyPtr(new ChBody(new ChCollisionModelParallel));
//		InitObject(Ring, 100000, Vector(x, plate_height + funnel_height - funnel_width, z), q, material_funnel, true, true, -20, -20);
//
//		AddCollisionGeometry(Ring, BOX, Vector(funnel_thickness, funnel_width * .9, funnel_thickness * 2), Vector(0, 0, 0), Quaternion(1, 0, 0, 0));
//		AddCollisionGeometry(Ring, SPHERE, Vector(funnel_thickness * 1.2, funnel_width * .9, funnel_thickness * 2), Vector(0, -1 * .9, 0), Quaternion(1, 0, 0, 0));
//
//		FinalizeObject(Ring, (ChSystemParallel *) system_gpu);
//	}

	InitObject(Lt, 100000, Vector(-funnel_width + funnel_thickness, plate_height + funnel_height + funnel_h / 2.0, 0), quat, material_funnel, true, true, -20, -20);
	InitObject(Rt, 100000, Vector(funnel_width - funnel_thickness, plate_height + funnel_height + funnel_h / 2.0, 0), quat, material_funnel, true, true, -20, -20);
	InitObject(Ft, 100000, Vector(0, plate_height + funnel_height + funnel_h / 2.0, -funnel_width + funnel_thickness), quat, material_funnel, true, true, -20, -20);
	InitObject(Bt, 100000, Vector(0, plate_height + funnel_height + funnel_h / 2.0, funnel_width - funnel_thickness), quat, material_funnel, true, true, -20, -20);

	AddCollisionGeometry(Lt, BOX, Vector(funnel_thickness, funnel_h, funnel_width), lpos, quat);
	AddCollisionGeometry(Rt, BOX, Vector(funnel_thickness, funnel_h, funnel_width), lpos, quat);
	AddCollisionGeometry(Ft, BOX, Vector(funnel_width, funnel_h, funnel_thickness), lpos, quat);
	AddCollisionGeometry(Bt, BOX, Vector(funnel_width, funnel_h, funnel_thickness), lpos, quat);
//
//	FinalizeObject(Lt, (ChSystemParallel *) system_gpu);
//	FinalizeObject(Rt, (ChSystemParallel *) system_gpu);
//	FinalizeObject(Ft, (ChSystemParallel *) system_gpu);
//	FinalizeObject(Bt, (ChSystemParallel *) system_gpu);

//	ChSharedBodyPtr TUBE = ChSharedBodyPtr(new ChBody(new ChCollisionModelParallel));
//	InitObject(TUBE, 100000, Vector(0, plate_height + funnel_height - funnel_h+.7 , 0), quat, material_funnel, true, true, -20, -20);
//	AddCollisionGeometry(TUBE, BOX, Vector(2,.05,2), Vector(0, 0, 0), quat);
//	FinalizeObject(TUBE, (ChSystemParallel *) system_gpu);

	layer_gen = new ParticleGenerator(((ChSystemParallel*) system_gpu));
	layer_gen->SetDensity(particle_density);
	layer_gen->SetRadius(R3(particle_radius));
	layer_gen->SetNormalDistribution(particle_radius, particle_std_dev);
	layer_gen->material->SetFriction(particle_friction);
	layer_gen->material->SetRollingFriction(particle_rolling_friction);
	layer_gen->material->SetSpinningFriction(particle_spinning_friction);
	layer_gen->material->SetCohesion(particle_cohesion);

	if (all_three_kinds) {
		layer_gen->AddMixtureType(MIX_TYPE1);
		layer_gen->AddMixtureType(MIX_TYPE2);
		layer_gen->AddMixtureType(MIX_TYPE3);
		layer_gen->AddMixtureType(MIX_TYPE4);
		layer_gen->AddMixtureType(MIX_SPHERE);
		layer_gen->AddMixtureType(MIX_ELLIPSOID);
		layer_gen->AddMixtureType(MIX_DOUBLESPHERE);
	} else if (particle_configuration == 0) {
		layer_gen->AddMixtureType(MIX_SPHERE);
	} else if (particle_configuration == 1) {
		layer_gen->AddMixtureType(MIX_DOUBLESPHERE);
	} else if (particle_configuration == 2) {
		layer_gen->AddMixtureType(MIX_ELLIPSOID);
	}

	//layer_gen->addPerturbedVolumeMixture(R3(0, 0, 0), I3(particle_grid_x, 30, particle_grid_z), R3(.1, .1, .1), R3(0, 0, 0));

	//=========================================================================================================
	//////Rendering specific stuff:
//	ChOpenGLManager * window_manager = new ChOpenGLManager();
//	ChOpenGL openGLView(window_manager, system_gpu, 800, 600, 0, 0, "Test_Solvers");
//	openGLView.render_camera->camera_pos = Vector(0, -5, -40);
//	openGLView.render_camera->look_at = Vector(0, -5, 0);
//	openGLView.render_camera->mScale=.4;
//	openGLView.SetCustomCallback(RunTimeStep);
//	openGLView.StartSpinning(window_manager);
//	window_manager->CallGlutMainLoop();
	//=========================================================================================================
	stringstream ss_m;
	ss_m << data_folder << "/" << "timing.txt";
	string timing_file_name = ss_m.str();
	ofstream ofile(timing_file_name.c_str());
	ofile.close();
	int file = 0;
	for (int i = 0; i < num_steps; i++) {
		cout << "step " << i;
		cout << " Residual: " << ((ChLcpSolverParallel *) (system_gpu->GetLcpSolverSpeed()))->GetResidual();
		cout << " ITER: " << ((ChLcpSolverParallel *) (system_gpu->GetLcpSolverSpeed()))->GetTotalIterations();
		cout << " OUTPUT STEP: Time= " << current_time << " bodies= " << system_gpu->GetNbodies() << " contacts= " << system_gpu->GetNcontacts() << " step time=" << system_gpu->GetTimerStep()
				<< " lcp time=" << system_gpu->GetTimerLcp() << " CDbroad time=" << system_gpu->GetTimerCollisionBroad() << " CDnarrow time=" << system_gpu->GetTimerCollisionNarrow() << " Iterations="
				<< ((ChLcpSolverParallel*) (system_gpu->GetLcpSolverSpeed()))->GetTotalIterations() << "\n";
		//TimingFile(system_gpu, timing_file_name, current_time);
		system_gpu->DoStepDynamics(timestep);
		int save_every = 1.0 / timestep / 30.0;     //save data every n steps
		RunTimeStep(system_gpu, i);
		if (i % save_every == 0) {
			stringstream ss;
			cout << "Frame: " << file << endl;
			ss << data_folder << "/" << file << ".txt";
			DumpAllObjectsWithGeometryPovray(system_gpu, ss.str());
			file++;
		}
		current_time += timestep;

	}
	//stringstream ss;
	//ss << data_folder << "/" << particle_friction << "_" << particle_cohesion << "_" << plate_friction << ".txt";
	//DumpAllObjectsWithGeometry(system_gpu, ss.str());
	return 0;
}

