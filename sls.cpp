#include "common/common.h"
#include "common/input_output.h"
#include "common/parser.h"

ChVector<> lpos(0, 0, 0);
ChQuaternion<> quat(1, 0, 0, 0);

//all dimensions are in millimeters, milligrams
real container_width = 25;		//width of area with particles
real container_length = 200;		//length of area that roller will go over		1194mm maximum
real container_thickness = 1;	//thickness of container walls
real container_height = 100;		//height of the outer walls
real container_friction = 1;
real spacer_width = 1;
real spacer_height = 5;

real roller_overlap = 1; 		//amount that roller goes over the container area
real roller_length = 10;			//length of the roller
real roller_radius = 76.2/2.0;			//radius of roller
real roller_omega = 1;
real roller_mass = 1;
real roller_friction = .2;
real roller_cohesion = 0;
real particle_radius = .1;

real gravity = -9810;			//acceleration due to gravity
real timestep = .0001;			//step size
real time_to_run = .6;			//length of simulation
real current_time = 0;

int num_steps = time_to_run / timestep;
int max_iteration = 50;
int tolerance = 1e-3;

GPUSOLVERTYPE solver = ACCELERATED_PROJECTED_GRADIENT_DESCENT;
string data_folder = "data";
ChSharedBodyPtr ROLLER;

template<class T>
void RunTimeStep(T* mSys, const int frame) {

}
int main(int argc, char* argv[]) {
	omp_set_num_threads(4);
	cout << "Mass, Radius, Friction_Sphere, Friction_Plate, Data Folder, create_particle_plate all_three_kinds, particle configuration" << endl;
	string solver_string = "ACCELERATED_PROJECTED_GRADIENT_DESCENT";
	//=========================================================================================================
	ChSystemGPU * system_gpu = new ChSystemGPU;
	ChLcpSystemDescriptorGPU *mdescriptor = new ChLcpSystemDescriptorGPU();
	ChContactContainerGPU *mcontactcontainer = new ChContactContainerGPU();
	//ChCollisionSystemBulletGPU *mcollisionengine = new ChCollisionSystemBulletGPU();
	ChCollisionSystemGPU *mcollisionengine = new ChCollisionSystemGPU();
	system_gpu->ChangeLcpSystemDescriptor(mdescriptor);
	system_gpu->ChangeContactContainer(mcontactcontainer);
	system_gpu->ChangeCollisionSystem(mcollisionengine);
	system_gpu->SetIntegrationType(ChSystem::INT_ANITESCU);

	//=========================================================================================================

	system_gpu->SetMaxiter(max_iteration);
	system_gpu->SetIterLCPmaxItersSpeed(max_iteration);
	((ChLcpSolverGPU *) (system_gpu->GetLcpSolverSpeed()))->SetMaxIteration(max_iteration);
	system_gpu->SetTol(tolerance);
	system_gpu->SetTolSpeeds(tolerance);
	((ChLcpSolverGPU *) (system_gpu->GetLcpSolverSpeed()))->SetTolerance(tolerance);
	((ChLcpSolverGPU *) (system_gpu->GetLcpSolverSpeed()))->SetCompliance(0, 0, 0);
	((ChLcpSolverGPU *) (system_gpu->GetLcpSolverSpeed()))->SetContactRecoverySpeed(10);
	setSolverGPU(solver_string, system_gpu); //reads a string and sets the solver
	//((ChLcpSolverGPU *) (system_gpu->GetLcpSolverSpeed()))->SetSolverType(solver);
	((ChCollisionSystemGPU *) (system_gpu->GetCollisionSystem()))->SetCollisionEnvelope(particle_radius * .1);
	mcollisionengine->setBinsPerAxis(R3(40, 15, 40));
	mcollisionengine->setBodyPerBin(100, 50);

	//=========================================================================================================
	ChMitsubaRender output(system_gpu);
	output.SetIntegrator("photonmapper");
	output.SetIntegratorOption("integer", "maxDepth", "32");
	output.SetFilm("ldrfilm");
	output.SetFilmOption("integer", "height", "1200");
	output.SetFilmOption("integer", "width", "1920");
	output.camera_target = ChVector<>(0, -5, 0);
	output.camera_origin = ChVector<>(0, -5, -40);
	output.camera_up = ChVector<>(0, 1, 0);
	output.SetDataFolder(data_folder);
	output.ExportScript("test.xml");
	//=========================================================================================================
	system_gpu->Set_G_acc(ChVector<>(0, gravity, 0));
	system_gpu->SetStep(timestep);
	//=========================================================================================================

	ChSharedBodyPtr PLATE = ChSharedBodyPtr(new ChBody(new ChCollisionModelGPU));
	ChSharedBodyPtr L = ChSharedBodyPtr(new ChBody(new ChCollisionModelGPU));
	ChSharedBodyPtr R = ChSharedBodyPtr(new ChBody(new ChCollisionModelGPU));
	ChSharedBodyPtr F = ChSharedBodyPtr(new ChBody(new ChCollisionModelGPU));
	ChSharedBodyPtr B = ChSharedBodyPtr(new ChBody(new ChCollisionModelGPU));
	ChSharedBodyPtr SPACER_L = ChSharedBodyPtr(new ChBody(new ChCollisionModelGPU));
	ChSharedBodyPtr SPACER_R = ChSharedBodyPtr(new ChBody(new ChCollisionModelGPU));

	InitObject(PLATE, 1, ChVector<>(0, 0, 0), quat, container_friction, container_friction, 0, true, true, -1000, -20000);

	InitObject(L, 100000, Vector(-container_width + container_thickness, container_height, 0), quat, container_friction, container_friction, 0, true, true, -20, -20);
	InitObject(R, 100000, Vector(container_width - container_thickness, container_height, 0), quat, container_friction, container_friction, 0, true, true, -20, -20);
	InitObject(F, 100000, Vector(0, container_height, -container_length + container_thickness), quat, container_friction, container_friction, 0, true, true, -20, -20);
	InitObject(B, 100000, Vector(0, container_height, container_length - container_thickness), quat, container_friction, container_friction, 0, true, true, -20, -20);
	InitObject(
			SPACER_L,
			100000,
			Vector(-container_width + container_thickness * 2 + spacer_width, container_thickness + spacer_height, 0),
			quat,
			container_friction,
			container_friction,
			0,
			true,
			true,
			-20,
			-20);
	InitObject(
			SPACER_R,
			100000,
			Vector(container_width - container_thickness * 2 - spacer_width, container_thickness + spacer_height, 0),
			quat,
			container_friction,
			container_friction,
			0,
			true,
			true,
			-20,
			-20);

	AddCollisionGeometry(PLATE, BOX, ChVector<>(container_width, container_thickness, container_length), lpos, quat);
	AddCollisionGeometry(L, BOX, Vector(container_thickness, container_height, container_length), lpos, quat);
	AddCollisionGeometry(R, BOX, Vector(container_thickness, container_height, container_length), lpos, quat);
	AddCollisionGeometry(F, BOX, Vector(container_width, container_height, container_thickness), lpos, quat);
	AddCollisionGeometry(B, BOX, Vector(container_width, container_height, container_thickness), lpos, quat);
	AddCollisionGeometry(SPACER_L, BOX, Vector(spacer_width, spacer_height, container_length), lpos, quat);
	AddCollisionGeometry(SPACER_R, BOX, Vector(spacer_width, spacer_height, container_length), lpos, quat);

	FinalizeObject(PLATE, (ChSystemGPU *) system_gpu);
	FinalizeObject(L, (ChSystemGPU *) system_gpu);
	FinalizeObject(R, (ChSystemGPU *) system_gpu);
	FinalizeObject(F, (ChSystemGPU *) system_gpu);
	FinalizeObject(B, (ChSystemGPU *) system_gpu);
	FinalizeObject(SPACER_L, (ChSystemGPU *) system_gpu);
	FinalizeObject(SPACER_R, (ChSystemGPU *) system_gpu);

	ROLLER = ChSharedBodyPtr(new ChBody(new ChCollisionModelGPU));
	ChQuaternion<> roller_quat;
	roller_quat.Q_from_AngAxis(PI / 2.0, ChVector<>(0, 0, 1));

	InitObject(ROLLER, 1, ChVector<>(0, 0, 0), roller_quat, roller_friction, roller_friction, 0, true, true, -1000, -20000);
	AddCollisionGeometry(ROLLER, CYLINDER, ChVector<>(roller_radius, roller_length * 2, 0), lpos, quat);
	FinalizeObject(ROLLER, (ChSystemGPU *) system_gpu);

	//=========================================================================================================
	//////Rendering specific stuff:
	ChOpenGLManager * window_manager = new ChOpenGLManager();
	ChOpenGL openGLView(window_manager, system_gpu, 800, 600, 0, 0, "Test_Solvers");
	openGLView.render_camera->camera_pos = Vector(-50, 0, -50);
	openGLView.render_camera->look_at = Vector(0, 0, 0);
	openGLView.SetCustomCallback(RunTimeStep);
	openGLView.StartSpinning(window_manager);
	window_manager->CallGlutMainLoop();
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
		RunTimeStep(system_gpu, i);
//		if (i % save_every == 0) {
//			stringstream ss;
//			cout << "Frame: " << file << endl;
//			ss << data_folder << "/" << file << ".txt";
//			DumpObjects(system_gpu, ss.str());
//			//output.ExportData(ss.str());
//			file++;
//		}
		current_time += timestep;
	}
	stringstream ss;
	//ss << data_folder << "/" << particle_friction << "_" << plate_friction << "_" << create_particle_plate << ".txt";

	//DumpAllObjectsWithGeometry(system_gpu, ss.str());
	return 0;
}

