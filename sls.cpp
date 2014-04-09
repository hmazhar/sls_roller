#include "common/common.h"
#include "common/generation.h"
#include "common/parser.h"
#include "common/input_output.h"
ChVector<> lpos(0, 0, 0);
ChQuaternion<> quat(1, 0, 0, 0);

//all dimensions are in millimeters, milligrams
real container_width = 5;     //width of area with particles
real container_length = 25;     //length of area that roller will go over		1194mm maximum
real container_thickness = .25;     //thickness of container walls
real container_height = 2;     //height of the outer walls
real container_friction = 0;
real floor_friction = .2;
real spacer_width = 1;
real spacer_height = 1;

real roller_overlap = 1;     //amount that roller goes over the container area
real roller_length = 2.5 - .25;     //length of the roller
real roller_radius = 76.2 / 2.0;     //radius of roller
real roller_omega = 0;
real roller_velocity = -127;
real roller_mass = 1;
real roller_friction = .1;
real roller_cohesion = 0;
real particle_radius = .058 / 2.0;
real particle_std_dev = .015 / 2.0;
real particle_mass = .05;
real particle_density = 0.93;
real particle_layer_thickness = particle_radius * 12;
real particle_friction = .52;
real rolling_friction = .1;
real spinning_friction = .1;
real gravity = -9810;     //acceleration due to gravity
real timestep = .00001;     //step size
real time_to_run = 1;     //length of simulation
real current_time = 0;

int num_steps = time_to_run / timestep;
int max_iteration = 20;
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

	cout << ROLLER->GetPos().x << " " << ROLLER->GetPos().y << " " << ROLLER->GetPos().z << endl;
	;

}
int main(int argc, char* argv[]) {
	bool visualize = true;
	int config = 0;
	bool full_mix = true;
	if (argc > 1) {
		visualize = atoi(argv[1]);
		particle_friction = atof(argv[2]);
		rolling_friction = atof(argv[3]);
		spinning_friction = atof(argv[4]);
		roller_velocity = -atof(argv[5]);
		particle_layer_thickness = atof(argv[6]);
		full_mix = atoi(argv[7]);
		data_folder = argv[8];
	}
	//cout << "Mass, Radius, Friction_Sphere, Friction_Plate, Data Folder, create_particle_plate all_three_kinds, particle configuration" << endl;
	//=========================================================================================================
	ChSystemParallelDVI * system_gpu = new ChSystemParallelDVI;
	system_gpu->SetIntegrationType(ChSystem::INT_ANITESCU);
	//=========================================================================================================
	//system_gpu->SetMaxiter(max_iteration);

	//system_gpu->SetIterLCPmaxItersSpeed(max_iteration);
	//((ChLcpSolverParallelDVI *) (system_gpu->GetLcpSolverSpeed()))->SetMaxIteration(max_iter);
	((ChLcpSolverParallelDVI *) (system_gpu->GetLcpSolverSpeed()))->SetMaxIterationNormal(max_iteration * 2);
	((ChLcpSolverParallelDVI *) (system_gpu->GetLcpSolverSpeed()))->SetMaxIterationSliding(max_iteration);
	((ChLcpSolverParallelDVI *) (system_gpu->GetLcpSolverSpeed()))->SetMaxIterationSpinning(max_iteration);
	system_gpu->SetTol(0);
	system_gpu->SetTolSpeeds(0);
	((ChLcpSolverParallelDVI *) (system_gpu->GetLcpSolverSpeed()))->SetTolerance(0);
	((ChLcpSolverParallelDVI *) (system_gpu->GetLcpSolverSpeed()))->SetCompliance(0);
	((ChLcpSolverParallelDVI *) (system_gpu->GetLcpSolverSpeed()))->SetContactRecoverySpeed(300);
	((ChLcpSolverParallelDVI *) (system_gpu->GetLcpSolverSpeed()))->SetSolverType(APGDRS);
	((ChCollisionSystemParallel *) (system_gpu->GetCollisionSystem()))->SetCollisionEnvelope(particle_radius * .05);
	((ChCollisionSystemParallel *) (system_gpu->GetCollisionSystem()))->setBinsPerAxis(I3(40, 200, 100));
	((ChCollisionSystemParallel *) (system_gpu->GetCollisionSystem()))->setBodyPerBin(100, 50);
	system_gpu->DoThreadTuning(false);
	system_gpu->SetMinThreads(32);
	system_gpu->Set_G_acc(ChVector<>(0, gravity, 0));
	system_gpu->SetStep(timestep);

	//=========================================================================================================
	((ChSystemParallel*) system_gpu)->SetAABB(R3(-6, -3, -30), R3(6, 6, 30));

	ChSharedBodyPtr PLATE = ChSharedBodyPtr(new ChBody(new ChCollisionModelParallel));

	ChSharedPtr<ChMaterialSurface> material_plate;
	material_plate = ChSharedPtr<ChMaterialSurface>(new ChMaterialSurface);
	material_plate->SetFriction(0);

	InitObject(PLATE, 100000, ChVector<>(0, 0, 0), quat, material_plate, true, true, -20, -20);

	ChSharedPtr<ChMaterialSurface> material;
	material = ChSharedPtr<ChMaterialSurface>(new ChMaterialSurface);
	material->SetFriction(container_friction);

	AddCollisionGeometry(PLATE, BOX, Vector(container_thickness, container_height, container_length), Vector(-container_width + container_thickness, container_height, 0), quat);
	AddCollisionGeometry(PLATE, BOX, Vector(container_thickness, container_height, container_length), Vector(container_width - container_thickness, container_height, 0), quat);
	AddCollisionGeometry(PLATE, BOX, Vector(container_width, container_height, container_thickness), Vector(0, container_height, -container_length + container_thickness), quat);
	AddCollisionGeometry(PLATE, BOX, Vector(container_width, container_height, container_thickness), Vector(0, container_height, container_length - container_thickness), quat);
	AddCollisionGeometry(PLATE, BOX, Vector(container_width, container_thickness, container_length), Vector(0, container_height*2,0), quat);
	FinalizeObject(PLATE, (ChSystemParallel *) system_gpu);

	ChSharedPtr<ChMaterialSurface> material_bottom;
	material_bottom = ChSharedPtr<ChMaterialSurface>(new ChMaterialSurface);
	material_bottom->SetFriction(floor_friction);

	ChSharedBodyPtr BOTTOM = ChSharedBodyPtr(new ChBody(new ChCollisionModelParallel));
	InitObject(BOTTOM, 100000, ChVector<>(0, 0, 0), quat, material_bottom, true, true, -20, -20);
	AddCollisionGeometry(BOTTOM, BOX, ChVector<>(container_width, container_thickness, container_length), lpos, quat);

	FinalizeObject(BOTTOM, (ChSystemParallel *) system_gpu);

	ROLLER = ChSharedBodyPtr(new ChBody(new ChCollisionModelParallel));
	ChQuaternion<> roller_quat;
	roller_quat.Q_from_AngAxis(PI / 2.0, ChVector<>(0, 0, 1));

	ChSharedPtr<ChMaterialSurface> material_roller;
	material_roller = ChSharedPtr<ChMaterialSurface>(new ChMaterialSurface);
	material_roller->SetFriction(roller_friction);

	InitObject(ROLLER, 1000, ChVector<>(0, roller_radius + particle_layer_thickness + container_thickness, container_length + roller_radius / 3.0), roller_quat, material_roller,
			true, false, -20, -20);
	AddCollisionGeometry(ROLLER, CYLINDER, ChVector<>(roller_radius, roller_length * 2, roller_radius), lpos, quat);
	FinalizeObject(ROLLER, (ChSystemParallel *) system_gpu);
	//68
	int3 num_per_dir = I3(1, 1, 1);
	//num_per_dir = I3(90, 16, 1);

	num_per_dir = I3(90, 16*particle_layer_thickness/.2032, 540);
	//num_per_dir = I3(90, 1, 100);
	ParticleGenerator < ChSystemParallelDVI > layer_gen(system_gpu);
	layer_gen.SetDensity(particle_density);
	layer_gen.SetRadius(R3(particle_radius));
	layer_gen.SetNormalDistribution(particle_radius, particle_std_dev);
	layer_gen.material->SetFriction(particle_friction);
	layer_gen.material->SetRollingFriction(rolling_friction);
	layer_gen.material->SetSpinningFriction(spinning_friction);

	if (full_mix) {
		layer_gen.AddMixtureType(MIX_SPHERE);
		layer_gen.AddMixtureType(MIX_TYPE1);
		layer_gen.AddMixtureType(MIX_TYPE2);
		layer_gen.AddMixtureType(MIX_TYPE3);
		layer_gen.AddMixtureType(MIX_TYPE4);
		layer_gen.AddMixtureType(MIX_ELLIPSOID);
		layer_gen.AddMixtureType(MIX_DOUBLESPHERE);
	}else{
		layer_gen.AddMixtureType(MIX_SPHERE);
	}

	//layer_gen.addPerturbedVolume(R3(0, 1.2, 0), SPHERE, num_per_dir, R3(.1, .1, .1), R3(0));

	layer_gen.addPerturbedVolumeMixture(R3(0, 1.3, 0), num_per_dir, R3(.1, .1, .1), R3(0));
	//num_per_dir = I3(90, 15, 100);
	//num_per_dir = I3(90, 15, 50);
	//num_per_dir = I3(1, 15, 100);
	//num_per_dir = I3(90, 30, 50);
	//layer_gen.addPerturbedVolumeMixture(R3(0, 3.2, 15), num_per_dir, R3(.1, .1, .1), R3(0));

	//=========================================================================================================
	//////Rendering specific stuff:
	if (visualize) {
		ChOpenGLManager * window_manager = new ChOpenGLManager();
		ChOpenGL openGLView(window_manager, system_gpu, 800, 600, 0, 0, "Test_Solvers");
		openGLView.render_camera->camera_position = glm::vec3(-50, 0, -50);
		openGLView.render_camera->camera_look_at = glm::vec3(0, 0, 0);
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
		cout << " OUTPUT STEP: Time= " << current_time << " bodies= " << system_gpu->GetNbodies() << " contacts= " << system_gpu->GetNcontacts() << " step time="
				<< system_gpu->GetTimerStep() << " lcp time=" << system_gpu->GetTimerLcp() << " CDbroad time=" << system_gpu->GetTimerCollisionBroad() << " CDnarrow time="
				<< system_gpu->GetTimerCollisionNarrow() << " Iterations=" << ((ChLcpSolverParallel*) (system_gpu->GetLcpSolverSpeed()))->GetTotalIterations() << " "
				<< ROLLER->GetPos().z << "\n";
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
		if (ROLLER->GetPos().z < -15) {
			break;
		}
		current_time += timestep;
	}
	stringstream ss;
	return 0;
}

