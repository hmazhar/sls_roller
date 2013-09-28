#include "common/common.h"
#include "common/input_output.h"
#include "common/parser.h"
#include "common/generation.h"
ChVector<> lpos(0, 0, 0);
ChQuaternion<> quat(1, 0, 0, 0);

//all dimensions are in millimeters, milligrams
real plate_height = -5;
real plate_thickness = .25;
real plate_radius = 5;
real plate_friction = 1;

real particle_radius = .058 / 2.0;
real particle_std_dev = .015 / 2.0;
real particle_density = .446;
real particle_friction = 1;
real particle_rolling_friction = 1;
real particle_spinning_friction = 1;
real particle_cohesion = 0;
Vector particle_initial_vel = Vector(0, -20, 0);     //initial velocity

real container_width = 5.0;
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

int particles_every = 300;     //add particles every n steps
int max_iteration = 45;
int tolerance = 1;

int particle_configuration = 0;
//0: single sphere
//1: two spheres joined together
//2: single ellipsoid

bool create_particle_plate = false;
real particle_plate_dim = 8;
real plate_particle_radius = .075;
bool all_three_kinds = true;
GPUSOLVERTYPE solver = ACCELERATED_PROJECTED_GRADIENT_DESCENT;

string data_folder = "data";
real start_height = 2;
ParticleGenerator *layer_gen;

template<class T>
void RunTimeStep(T* mSys, const int frame) {

	if (frame % particles_every == 0 && frame * timestep < 5.5 && frame * timestep > .02) {
		layer_gen->addPerturbedVolumeMixture(
				R3(0, start_height, 0),
				I3(particle_grid_x, 1, particle_grid_z),
				R3(.1, .1, .1),
				R3(particle_initial_vel.x, particle_initial_vel.y, particle_initial_vel.z));
	}

}
int main(int argc, char* argv[]) {
	omp_set_num_threads(4);
	srand(1);

	if (argc == 2) {
		data_folder = argv[1];
	}

	cout << "Density, Radius, Friction_Sphere,Cohesion_Sphere,  Friction_Plate, Data Folder" << endl;
	if (argc == 9) {
		particle_density = atof(argv[1]);
		particle_radius = atof(argv[2]);
		particle_friction = atof(argv[3]);
		particle_rolling_friction = atof(argv[4]);
		particle_spinning_friction = atof(argv[5]);
		particle_cohesion = atof(argv[6]);
		plate_friction = atof(argv[7]);
		data_folder = argv[8];
		//all_three_kinds = atoi(argv[7]);
		//particle_configuration = atoi(argv[8]);
	}

	cout << particle_density << " " << particle_radius << " " << particle_friction <<" "<<particle_rolling_friction<<" "<<particle_spinning_friction<< " " << particle_cohesion << " " << plate_friction << " " << data_folder << " " << endl;
	//=========================================================================================================
	//=========================================================================================================
	ChSystemGPU * system_gpu = new ChSystemGPU;
	ChCollisionSystemGPU *mcollisionengine = new ChCollisionSystemGPU();
	system_gpu->SetIntegrationType(ChSystem::INT_ANITESCU);
	//system_gpu->SetAABB(R3(-2,-5,-2), R3(2,5,2));

	//=========================================================================================================
	system_gpu->SetMaxiter(max_iteration);
	system_gpu->SetIterLCPmaxItersSpeed(max_iteration);
	((ChLcpSolverGPU *) (system_gpu->GetLcpSolverSpeed()))->SetMaxIteration(max_iteration);
	system_gpu->SetTol(tolerance);
	system_gpu->SetTolSpeeds(tolerance);
	((ChLcpSolverGPU *) (system_gpu->GetLcpSolverSpeed()))->SetTolerance(tolerance);
	((ChLcpSolverGPU *) (system_gpu->GetLcpSolverSpeed()))->SetCompliance(0, 0, 0);
	((ChLcpSolverGPU *) (system_gpu->GetLcpSolverSpeed()))->SetContactRecoverySpeed(40);
	//setSolverGPU(solver_string, system_gpu);     //reads a string and sets the solver
	((ChLcpSolverGPU *) (system_gpu->GetLcpSolverSpeed()))->SetWarmStart(false);
	((ChLcpSolverGPU *) (system_gpu->GetLcpSolverSpeed()))->SetSolverType(solver);
	((ChCollisionSystemGPU *) (system_gpu->GetCollisionSystem()))->SetCollisionEnvelope(particle_radius * .05);
	mcollisionengine->setBinsPerAxis(R3(50, 50, 50));
	mcollisionengine->setBodyPerBin(100, 50);
	//((ChSystemGPU*) system_gpu)->SetAABB(R3(-6, -6, -6), R3(6, 4, 6));

	//=========================================================================================================
	system_gpu->Set_G_acc(ChVector<>(0, gravity, 0));
	system_gpu->SetStep(timestep);
	//=========================================================================================================

	int num_particle = particle_plate_dim / plate_particle_radius;

	ChSharedPtr<ChMaterialSurface> material;
	material = ChSharedPtr<ChMaterialSurface>(new ChMaterialSurface);
	material->SetFriction(plate_friction);
	material->SetSpinningFriction(plate_friction);
	material->SetRollingFriction(plate_friction);

	if (create_particle_plate) {

		ChSharedBodyPtr sphere;
		sphere = ChSharedBodyPtr(new ChBody(new ChCollisionModelGPU));
		InitObject(sphere, 1, Vector(0, 0, 0), quat, material, true, true, -20, -20);
		for (int i = 0; i < num_particle; i++) {
			for (int j = 0; j < num_particle; j++) {

				ChVector<> position(
						i * plate_particle_radius * 1.2 - num_particle * plate_particle_radius * .6,
						plate_height + plate_thickness + plate_particle_radius,
						j * plate_particle_radius * 1.2 - num_particle * plate_particle_radius * .6);

				position.x += rand() % 10000 / 10000.0 * plate_particle_radius * .25 - plate_particle_radius * .25 * .5;
				position.z += rand() % 10000 / 10000.0 * plate_particle_radius * .25 - plate_particle_radius * .25 * .5;

				AddCollisionGeometry(sphere, ELLIPSOID, ChVector<>(plate_particle_radius, plate_particle_radius * .2, plate_particle_radius), position, quat);
			}
		}
		FinalizeObject(sphere, (ChSystemGPU *) system_gpu);
	}
	ChSharedBodyPtr PLATE = ChSharedBodyPtr(new ChBody(new ChCollisionModelGPU));
	InitObject(PLATE, 1, ChVector<>(0, plate_height, 0), quat, material, true, true, -1000, -20000);
	AddCollisionGeometry(PLATE, BOX, ChVector<>(plate_radius, plate_thickness, plate_radius), lpos, quat);
	FinalizeObject(PLATE, (ChSystemGPU *) system_gpu);

	ChSharedBodyPtr L = ChSharedBodyPtr(new ChBody(new ChCollisionModelGPU));
	ChSharedBodyPtr R = ChSharedBodyPtr(new ChBody(new ChCollisionModelGPU));
	ChSharedBodyPtr F = ChSharedBodyPtr(new ChBody(new ChCollisionModelGPU));
	ChSharedBodyPtr B = ChSharedBodyPtr(new ChBody(new ChCollisionModelGPU));

	InitObject(L, 100000, Vector(-container_width + container_thickness, plate_height + container_height, 0), quat, material, true, true, -20, -20);
	InitObject(R, 100000, Vector(container_width - container_thickness, plate_height + container_height, 0), quat, material, true, true, -20, -20);
	InitObject(F, 100000, Vector(0, plate_height + container_height, -container_width + container_thickness), quat, material, true, true, -20, -20);
	InitObject(B, 100000, Vector(0, plate_height + container_height, container_width - container_thickness), quat, material, true, true, -20, -20);

	AddCollisionGeometry(L, BOX, Vector(container_thickness, container_height, container_width), lpos, quat);
	AddCollisionGeometry(R, BOX, Vector(container_thickness, container_height, container_width), lpos, quat);
	AddCollisionGeometry(F, BOX, Vector(container_width * wscale, container_height, container_thickness), lpos, quat);
	AddCollisionGeometry(B, BOX, Vector(container_width * wscale, container_height, container_thickness), lpos, quat);

	FinalizeObject(L, (ChSystemGPU *) system_gpu);
	FinalizeObject(R, (ChSystemGPU *) system_gpu);
	FinalizeObject(F, (ChSystemGPU *) system_gpu);
	FinalizeObject(B, (ChSystemGPU *) system_gpu);

	//==========

	real funnel_thickness = .05;
	real funnel_height = 5;
	real funnel_width = 1;
	real funnel_h = 2;
	real funnel_offset = funnel_width * .8;     //.5*sqrt(2)*funnel_width-funnel_thickness*6+particle_radius*3;
	ChSharedBodyPtr Lt = ChSharedBodyPtr(new ChBody(new ChCollisionModelGPU));
	ChSharedBodyPtr Rt = ChSharedBodyPtr(new ChBody(new ChCollisionModelGPU));
	ChSharedBodyPtr Ft = ChSharedBodyPtr(new ChBody(new ChCollisionModelGPU));
	ChSharedBodyPtr Bt = ChSharedBodyPtr(new ChBody(new ChCollisionModelGPU));

	ChSharedBodyPtr F1 = ChSharedBodyPtr(new ChBody(new ChCollisionModelGPU));
	ChSharedBodyPtr F2 = ChSharedBodyPtr(new ChBody(new ChCollisionModelGPU));
	ChSharedBodyPtr F3 = ChSharedBodyPtr(new ChBody(new ChCollisionModelGPU));
	ChSharedBodyPtr F4 = ChSharedBodyPtr(new ChBody(new ChCollisionModelGPU));

	ChSharedPtr<ChMaterialSurface> material_funnel;
	material_funnel = ChSharedPtr<ChMaterialSurface>(new ChMaterialSurface);
	material_funnel->SetFriction(plate_friction);
	material_funnel->SetRollingFriction(plate_friction);
	material_funnel->SetSpinningFriction(plate_friction);

	material_funnel->SetCohesion(-10000);

	ChQuaternion<> quat_r;
	quat_r.Q_from_AngX(45);
	InitObject(F1, 100000, Vector(0, plate_height + funnel_height - funnel_width, -funnel_offset), quat_r, material_funnel, true, true, -20, -20);
	quat_r.Q_from_AngX(-45);
	InitObject(F2, 100000, Vector(0, plate_height + funnel_height - funnel_width, funnel_offset), quat_r, material_funnel, true, true, -20, -20);
	quat_r.Q_from_AngZ(45);
	InitObject(F3, 100000, Vector(funnel_offset, plate_height + funnel_height - funnel_width, 0), quat_r, material_funnel, true, true, -20, -20);
	quat_r.Q_from_AngZ(-45);
	InitObject(F4, 100000, Vector(-funnel_offset, plate_height + funnel_height - funnel_width, 0), quat_r, material_funnel, true, true, -20, -20);

	AddCollisionGeometry(F1, BOX, Vector(funnel_width, funnel_thickness, funnel_width), lpos, quat);
	AddCollisionGeometry(F2, BOX, Vector(funnel_width, funnel_thickness, funnel_width), lpos, quat);
	AddCollisionGeometry(F3, BOX, Vector(funnel_width, funnel_thickness, funnel_width), lpos, quat);
	AddCollisionGeometry(F4, BOX, Vector(funnel_width, funnel_thickness, funnel_width), lpos, quat);

	FinalizeObject(F1, (ChSystemGPU *) system_gpu);
	FinalizeObject(F2, (ChSystemGPU *) system_gpu);
	FinalizeObject(F3, (ChSystemGPU *) system_gpu);
	FinalizeObject(F4, (ChSystemGPU *) system_gpu);

	InitObject(Lt, 100000, Vector(-funnel_width + funnel_thickness, plate_height + funnel_height + funnel_h / 2.0, 0), quat, material_funnel, true, true, -20, -20);
	InitObject(Rt, 100000, Vector(funnel_width - funnel_thickness, plate_height + funnel_height + funnel_h / 2.0, 0), quat, material_funnel, true, true, -20, -20);
	InitObject(Ft, 100000, Vector(0, plate_height + funnel_height + funnel_h / 2.0, -funnel_width + funnel_thickness), quat, material_funnel, true, true, -20, -20);
	InitObject(Bt, 100000, Vector(0, plate_height + funnel_height + funnel_h / 2.0, funnel_width - funnel_thickness), quat, material_funnel, true, true, -20, -20);

	AddCollisionGeometry(Lt, BOX, Vector(funnel_thickness, funnel_h, funnel_width), lpos, quat);
	AddCollisionGeometry(Rt, BOX, Vector(funnel_thickness, funnel_h, funnel_width), lpos, quat);
	AddCollisionGeometry(Ft, BOX, Vector(funnel_width, funnel_h, funnel_thickness), lpos, quat);
	AddCollisionGeometry(Bt, BOX, Vector(funnel_width, funnel_h, funnel_thickness), lpos, quat);

	FinalizeObject(Lt, (ChSystemGPU *) system_gpu);
	FinalizeObject(Rt, (ChSystemGPU *) system_gpu);
	FinalizeObject(Ft, (ChSystemGPU *) system_gpu);
	FinalizeObject(Bt, (ChSystemGPU *) system_gpu);

	ChSharedBodyPtr TUBE = ChSharedBodyPtr(new ChBody(new ChCollisionModelGPU));
	InitObject(TUBE, 100000, Vector(0, plate_height + funnel_height - funnel_h - funnel_h / 9.0, 0), quat, material_funnel, true, true, -20, -20);
	AddCollisionGeometry(TUBE, BOX, Vector(funnel_thickness, funnel_h / 6.0, funnel_width / 3.0), Vector(-funnel_width / 3.0, 0, 0), quat);
	AddCollisionGeometry(TUBE, BOX, Vector(funnel_thickness, funnel_h / 6.0, funnel_width / 3.0), Vector(funnel_width / 3.0, 0, 0), quat);
	AddCollisionGeometry(TUBE, BOX, Vector(funnel_width / 3.0, funnel_h / 6.0, funnel_thickness), Vector(0, 0, -funnel_width / 3.0), quat);
	AddCollisionGeometry(TUBE, BOX, Vector(funnel_width / 3.0, funnel_h / 6.0, funnel_thickness), Vector(0, 0, funnel_width / 3.0), quat);
	FinalizeObject(TUBE, (ChSystemGPU *) system_gpu);

	layer_gen = new ParticleGenerator(((ChSystemGPU*) system_gpu));
	layer_gen->SetDensity(particle_density);
	layer_gen->SetRadius(R3(particle_radius));
	layer_gen->SetNormalDistribution(particle_radius, particle_std_dev);
	layer_gen->material->SetFriction(particle_friction);
	layer_gen->material->SetRollingFriction(particle_rolling_friction);
	layer_gen->material->SetSpinningFriction(particle_spinning_friction);
	layer_gen->material->SetCohesion(particle_cohesion);

	if (all_three_kinds) {
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

	layer_gen->addPerturbedVolumeMixture(R3(0, 1, 0), I3(particle_grid_x, 30, particle_grid_z), R3(.1, .1, .1), R3(0, 0, 0));

	//=========================================================================================================
	//////Rendering specific stuff:
//	ChOpenGLManager * window_manager = new ChOpenGLManager();
//	ChOpenGL openGLView(window_manager, system_gpu, 800, 600, 0, 0, "Test_Solvers");
//	openGLView.render_camera->camera_pos = Vector(0, -5, -40);
//	openGLView.render_camera->look_at = Vector(0, -5, 0);
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
		cout << " Residual: " << ((ChLcpSolverGPU *) (system_gpu->GetLcpSolverSpeed()))->GetResidual();
		cout << " ITER: " << ((ChLcpSolverGPU *) (system_gpu->GetLcpSolverSpeed()))->GetTotalIterations();
		cout << " OUTPUT STEP: Time= " << current_time << " bodies= " << system_gpu->GetNbodies() << " contacts= " << system_gpu->GetNcontacts() << " step time=" << system_gpu->GetTimerStep()
				<< " lcp time=" << system_gpu->GetTimerLcp() << " CDbroad time=" << system_gpu->GetTimerCollisionBroad() << " CDnarrow time=" << system_gpu->GetTimerCollisionNarrow() << " Iterations="
				<< ((ChLcpSolverGPU*) (system_gpu->GetLcpSolverSpeed()))->GetTotalIterations() << "\n";
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

