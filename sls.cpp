#include "common/common.h"
#include "common/input_output.h"
#include "common/parser.h"
#include "common/generation.h"
ChVector<> lpos(0, 0, 0);
ChQuaternion<> quat(1, 0, 0, 0);

//all dimensions are in millimeters, milligrams
real container_width = 5;		//width of area with particles
real container_length = 25;		//length of area that roller will go over		1194mm maximum
real container_thickness = .25;     //thickness of container walls
real container_height = 2;		//height of the outer walls
real container_friction = 0;
real floor_friction = .2;
real spacer_width = 1;
real spacer_height = 1;

real roller_overlap = 1; 		//amount that roller goes over the container area
real roller_length = 2.5 - .25;			//length of the roller
real roller_radius = 76.2 / 2.0;			//radius of roller
real roller_omega = 0;
real roller_velocity = -127;
real roller_mass = 1;
real roller_friction = .2;
real roller_cohesion = 0;
real particle_radius = .058 / 2.0;
real particle_std_dev = .015 / 2.0;
real particle_mass = .05;
real particle_density = 0.93;
real particle_layer_thickness = particle_radius * 12;
real particle_friction = .1;
real gravity = -9810;			//acceleration due to gravity
real timestep = .00001;			//step size
real time_to_run = 1;			//length of simulation
real current_time = 0;

int num_steps = time_to_run / timestep;
int max_iteration = 15;
int tolerance = 0;

string data_folder = "data/sls";
ChSharedBodyPtr ROLLER;
real ang = 0;

template<class T>
void RunTimeStep(T* mSys, const int frame) {
	ChVector<> roller_pos = ROLLER->GetPos();

	ROLLER->SetPos(ChVector<>(0, roller_radius + particle_layer_thickness + container_thickness, roller_pos.z + roller_velocity * timestep));
	ROLLER->SetPos_dt(ChVector<>(0, 0, roller_velocity));
	roller_omega = roller_velocity / roller_radius;
	ang += roller_omega * timestep;
	if (ang >= 2 * CH_C_PI) {
		ang = 0;
	}

	Quaternion q1;
	q1.Q_from_AngY(ang);
	Quaternion q2;
	q1 = Q_from_AngX(-ang);

	ChQuaternion<> roller_quat;
	roller_quat.Q_from_AngAxis(PI / 2.0, ChVector<>(0, 0, 1));

	ROLLER->SetRot(q1 % roller_quat);
	ROLLER->SetWvel_loc(Vector(0, roller_omega, 0));

}
int main(int argc, char* argv[]) {
	bool visualize = false;
	int threads = 8;

	if (argc > 1) {
		threads = atoi(argv[1]);

		visualize = atoi(argv[2]);
		//visualize
		//distribution_type
		//min_radius
		//max_radius
		//mass
		//friction
		//cohesion
		//layer_thickness
		//roller_velocity
		//roller_omega
		//roller_friction
		//roller_cohesion

	}
	omp_set_num_threads(threads);
	//cout << "Mass, Radius, Friction_Sphere, Friction_Plate, Data Folder, create_particle_plate all_three_kinds, particle configuration" << endl;
	//string solver_string = "ACCELERATED_PROJECTED_GRADIENT_DESCENT";
	//=========================================================================================================
	ChSystemParallel * system_gpu = new ChSystemParallel;
	system_gpu->SetIntegrationType(ChSystem::INT_ANITESCU);

	//=========================================================================================================
	system_gpu->SetParallelThreadNumber(threads);
	system_gpu->SetMaxiter(max_iteration);
	system_gpu->SetIterLCPmaxItersSpeed(max_iteration);
	((ChLcpSolverParallel *) (system_gpu->GetLcpSolverSpeed()))->SetMaxIteration(max_iteration);
	system_gpu->SetTol(0);
	system_gpu->SetTolSpeeds(0);
	((ChLcpSolverParallel *) (system_gpu->GetLcpSolverSpeed()))->SetTolerance(0);
	((ChLcpSolverParallel *) (system_gpu->GetLcpSolverSpeed()))->SetCompliance(0, 0, 0);
	((ChLcpSolverParallel *) (system_gpu->GetLcpSolverSpeed()))->SetContactRecoverySpeed(300);
	((ChLcpSolverParallel *) (system_gpu->GetLcpSolverSpeed()))->SetSolverType(ACCELERATED_PROJECTED_GRADIENT_DESCENT);
	((ChCollisionSystemParallel *) (system_gpu->GetCollisionSystem()))->SetCollisionEnvelope(particle_radius * .05);
	((ChCollisionSystemParallel *) (system_gpu->GetCollisionSystem()))->setBinsPerAxis(R3(40, 20, 200));
	((ChCollisionSystemParallel *) (system_gpu->GetCollisionSystem()))->setBodyPerBin(100, 50);
	system_gpu->Set_G_acc(ChVector<>(0, gravity, 0));
	system_gpu->SetStep(timestep);

	//=========================================================================================================
	system_gpu->Set_G_acc(ChVector<>(0, gravity, 0));
	system_gpu->SetStep(timestep);
	//=========================================================================================================
	((ChSystemParallel*) system_gpu)->SetAABB(R3(-6, -3, -30), R3(6, 6, 30));

	ChSharedBodyPtr PLATE = ChSharedBodyPtr(new ChBody(new ChCollisionModelParallel));
	ChSharedBodyPtr L = ChSharedBodyPtr(new ChBody(new ChCollisionModelParallel));
	ChSharedBodyPtr R = ChSharedBodyPtr(new ChBody(new ChCollisionModelParallel));
	ChSharedBodyPtr F = ChSharedBodyPtr(new ChBody(new ChCollisionModelParallel));
	ChSharedBodyPtr B = ChSharedBodyPtr(new ChBody(new ChCollisionModelParallel));
	ChSharedBodyPtr SPACER_L = ChSharedBodyPtr(new ChBody(new ChCollisionModelParallel));
	ChSharedBodyPtr SPACER_R = ChSharedBodyPtr(new ChBody(new ChCollisionModelParallel));

	ChSharedPtr<ChMaterialSurface> material_plate;
	material_plate = ChSharedPtr<ChMaterialSurface>(new ChMaterialSurface);
	material_plate->SetFriction(floor_friction);

	InitObject(PLATE, 100000, ChVector<>(0, 0, 0), quat, material_plate, true, true, -20, -20);

	ChSharedPtr<ChMaterialSurface> material;
	material = ChSharedPtr<ChMaterialSurface>(new ChMaterialSurface);
	material->SetFriction(container_friction);

	//InitObject(L, 100000, Vector(-container_width + container_thickness, container_height, 0), quat, material, true, true, -20, -20);
	//InitObject(R, 100000, Vector(container_width - container_thickness, container_height, 0), quat, material, true, true, -20, -20);
	//InitObject(F, 100000, Vector(0, container_height, -container_length + container_thickness), quat, material, true, true, -20, -20);
	//InitObject(B, 100000, Vector(0, container_height, container_length - container_thickness), quat, material, true, true, -20, -20);
	//InitObject(SPACER_L, 100000, Vector(-container_width + container_thickness * 2 + spacer_width, container_thickness + spacer_height, 0), quat, material, true, true, -20, -20);
	//InitObject(SPACER_R, 100000, Vector(container_width - container_thickness * 2 - spacer_width, container_thickness + spacer_height, 0), quat, material, true, true, -20, -20);

	AddCollisionGeometry(PLATE, BOX, ChVector<>(container_width, container_thickness, container_length), lpos, quat);
	AddCollisionGeometry(PLATE, BOX, Vector(container_thickness, container_height, container_length), Vector(-container_width + container_thickness, container_height, 0), quat);
	AddCollisionGeometry(PLATE, BOX, Vector(container_thickness, container_height, container_length), Vector(container_width - container_thickness, container_height, 0), quat);
	AddCollisionGeometry(PLATE, BOX, Vector(container_width, container_height, container_thickness), Vector(0, container_height, -container_length + container_thickness), quat);
	AddCollisionGeometry(PLATE, BOX, Vector(container_width, container_height, container_thickness), Vector(0, container_height, container_length - container_thickness), quat);
	//AddCollisionGeometry(SPACER_L, BOX, Vector(spacer_width, spacer_height, container_length), lpos, quat);
	//AddCollisionGeometry(SPACER_R, BOX, Vector(spacer_width, spacer_height, container_length), lpos, quat);

	FinalizeObject(PLATE, (ChSystemParallel *) system_gpu);
	//FinalizeObject(L, (ChSystemParallel *) system_gpu);
	//FinalizeObject(R, (ChSystemParallel *) system_gpu);
	//FinalizeObject(F, (ChSystemParallel *) system_gpu);
	//FinalizeObject(B, (ChSystemParallel *) system_gpu);
	//FinalizeObject(SPACER_L, (ChSystemParallel *) system_gpu);
	//FinalizeObject(SPACER_R, (ChSystemParallel *) system_gpu);

	ROLLER = ChSharedBodyPtr(new ChBody(new ChCollisionModelParallel));
	ChQuaternion<> roller_quat;
	roller_quat.Q_from_AngAxis(PI / 2.0, ChVector<>(0, 0, 1));

	ChSharedPtr<ChMaterialSurface> material_roller;
	material_roller = ChSharedPtr<ChMaterialSurface>(new ChMaterialSurface);
	material_roller->SetFriction(roller_friction);

	InitObject(
			ROLLER,
			1000,
			ChVector<>(0, roller_radius + particle_layer_thickness + container_thickness, container_length + roller_radius / 3.0),
			roller_quat,
			material_roller,
			true,
			false,
			-20,
			-20);
	AddCollisionGeometry(ROLLER, CYLINDER, ChVector<>(roller_radius, roller_length * 2, roller_radius), lpos, quat);
	FinalizeObject(ROLLER, (ChSystemParallel *) system_gpu);
	//68
	int3 num_per_dir = I3(68 * 2, 6, 540 * 2);
	//num_per_dir = I3(1, 16, 440);
	num_per_dir = I3(74, 16, 440);
	ParticleGenerator layer_gen(system_gpu);
	layer_gen.SetDensity(particle_density);
	layer_gen.SetRadius(R3(particle_radius));
	layer_gen.SetNormalDistribution(particle_radius, particle_std_dev);
	layer_gen.material->SetFriction(particle_friction);
	layer_gen.AddMixtureType(MIX_TYPE1);
		layer_gen.AddMixtureType(MIX_TYPE2);
		layer_gen.AddMixtureType(MIX_TYPE3);
		layer_gen.AddMixtureType(MIX_TYPE4);
		layer_gen.AddMixtureType(MIX_SPHERE);
		layer_gen.AddMixtureType(MIX_ELLIPSOID);
		layer_gen.AddMixtureType(MIX_DOUBLESPHERE);
	layer_gen.addPerturbedVolumeMixture(R3(0, 1.2, 0), num_per_dir, R3(.1, .1, .1), R3(0));
	num_per_dir = I3(74, 30, 50);
	//num_per_dir = I3(1, 30, 50);

	layer_gen.addPerturbedVolumeMixture(R3(0, 3.7, 15), num_per_dir, R3(.1, .1, .1), R3(0));

	//=========================================================================================================
	//////Rendering specific stuff:
	if (visualize) {
		ChOpenGLManager * window_manager = new ChOpenGLManager();
		ChOpenGL openGLView(window_manager, system_gpu, 800, 600, 0, 0, "Test_Solvers");
		openGLView.render_camera->camera_pos = Vector(-50, 0, -50);
		openGLView.render_camera->look_at = Vector(0, 0, 0);
		openGLView.SetCustomCallback(RunTimeStep);
		openGLView.StartSpinning(window_manager);
		window_manager->CallGlutMainLoop();
	}
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
		RunTimeStep(system_gpu, i);
		int save_every = 1.0 / timestep / 6000.0;     //save data every n steps
		if (i % save_every == 0) {
			stringstream ss;
			cout << "Frame: " << file << endl;
			ss << data_folder << "/" << file << ".txt";
			DumpAllObjectsWithGeometryPovray(system_gpu, ss.str());
			//output.ExportData(ss.str());
			file++;
		}
		current_time += timestep;
	}
	stringstream ss;
	//ss << data_folder << "/" << particle_friction << "_" << plate_friction << "_" << create_particle_plate << ".txt";

	//DumpAllObjectsWithGeometry(system_gpu, ss.str());
	return 0;
}

