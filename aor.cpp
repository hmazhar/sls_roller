#include "common/common.h"
#include "common/input_output.h"
#include "common/parser.h"

ChVector<> lpos(0, 0, 0);
ChQuaternion<> quat(1, 0, 0, 0);

//all dimensions are in millimeters, milligrams
real plate_height = -10;
real plate_thickness = .25;
real plate_radius = 7;
real plate_friction = 1;

real particle_radius = .1;
real particle_mass = .05;
real particle_density = .5;
real particle_friction = .5;
Vector particle_initial_vel = Vector(0, -2.5, 0);     //initial velocity

real container_width = 7.0;
real container_thickness = .25;
real container_height = 1.0;
real wscale = 1;

real gravity = -9810;
real timestep = .0001;
real time_to_run = .6;
real current_time = 0;
int num_steps = time_to_run / timestep;

int particle_grid_x = 14;
int particle_grid_z = 14;

int particles_every = 50;     //add particles every n steps
int save_every = 100;     //save data every n steps

int max_iteration = 20;
int tolerance = 1e-3;

int particle_configuration = 0;
//0: single sphere
//1: two spheres joined together
//2: single ellipsoid

bool create_particle_plate = true;
real particle_plate_dim = 8;
real plate_particle_radius = .075;
bool all_three_kinds = true;

GPUSOLVERTYPE solver = ACCELERATED_PROJECTED_GRADIENT_DESCENT;

string data_folder = "data";
real start_height = 1;

template<class T>
void RunTimeStep(T* mSys, const int frame) {

	ChSharedPtr<ChMaterialSurface> material;
	material = ChSharedPtr<ChMaterialSurface>(new ChMaterialSurface);
	material->SetFriction(particle_friction);
	material->SetCohesion(.1);
	if (frame % particles_every == 0 && frame * timestep < .4) {
		ChSharedBodyPtr sphere;
		for (int i = 0; i < particle_grid_x; i++) {
			for (int j = 0; j < particle_grid_z; j++) {
				sphere = ChSharedBodyPtr(new ChBody(new ChCollisionModelGPU));
				Quaternion q;
				q.Q_from_NasaAngles(Vector(rand() % 1000 / 1000.0, rand() % 1000 / 1000.0, rand() % 1000 / 1000.0));
				q.Normalize();
                
				ChVector<> position(i * particle_radius * 2 - particle_grid_x * .5 * particle_radius * 2, start_height, j * particle_radius * 2 - particle_grid_z * .5 * particle_radius * 2);
                
				position.x += rand() % 1000 / 100000.0;
				position.y += rand() % 1000 / 1000.0;
				position.z += rand() % 1000 / 100000.0;
                
				if (particle_configuration == 0) {
					real percent = (rand() % 10000 / 10000.0 - .5) / 2.0;
					real radius = particle_radius + particle_radius * percent;
					InitObject(sphere, particle_mass, position, quat, material, true, false, -1, i);
					AddCollisionGeometry(sphere, SPHERE, ChVector<>(radius, radius, radius), Vector(0, 0, 0), quat);
				}
				if (particle_configuration == 1) {
					position.x *= 1.2;
					position.z *= 1.0;
					InitObject(sphere, particle_mass, position, q, material, true, false, -1, i);
					real percent = (rand() % 10000 / 10000.0 - .5) / 2.0;
                    
					AddCollisionGeometry(sphere, SPHERE, ChVector<>(particle_radius + particle_radius * percent, particle_radius * .75, particle_radius * .75), Vector(-.025, 0, 0), quat);
					percent = (rand() % 10000 / 10000.0 - .5) / 2.0;
					AddCollisionGeometry(sphere, SPHERE, ChVector<>(particle_radius + particle_radius * percent, particle_radius * .75, particle_radius * .75), Vector(.025, 0, 0), quat);
					real rho = particle_density;
                    
					//Vector inertia = Vector(3.47e-6 * rho, 2.62e-7 * rho, 3.47e-6 * rho);
					real mass = particle_mass;
					Vector r = ChVector<>(particle_radius, particle_radius, particle_radius);
					Vector inertia = Vector((1 / 5.0 * mass * (r.y * r.y + r.z * r.z), 1 / 5.0 * mass * (r.x * r.x + r.z * r.z), 1 / 5.0 * mass * (r.x * r.x + r.y * r.y)));
                    
					sphere->SetInertiaXX(inertia);
				}
				if (particle_configuration == 2) {
					real percent = (rand() % 10000 / 10000.0 - .5);
					real radius1 = particle_radius * 1.2 + particle_radius * (rand() % 10000 / 10000.0 - .5);
					real radius2 = particle_radius * .75 + particle_radius * (rand() % 10000 / 10000.0 - .5);
					real radius3 = particle_radius * 1.2 + particle_radius * (rand() % 10000 / 10000.0 - .5);
					InitObject(sphere, particle_mass, position, q, material, true, false, -1, i);
                    
					AddCollisionGeometry(sphere, ELLIPSOID, ChVector<>(radius1, radius2, radius3), Vector(0, 0, 0), quat);
                    
					real mass = particle_mass;
					Vector r = ChVector<>(radius1, radius2, radius3);
					Vector inertia = Vector((1 / 5.0 * mass * (r.y * r.y + r.z * r.z), 1 / 5.0 * mass * (r.x * r.x + r.z * r.z), 1 / 5.0 * mass * (r.x * r.x + r.y * r.y)));
					sphere->SetInertiaXX(inertia);
                    
				}
				sphere->SetPos_dt(particle_initial_vel);
				FinalizeObject(sphere, (ChSystemGPU *) mSys);
			}
		}
		if (all_three_kinds) {
			particle_configuration++;
			if (particle_configuration > 2) {
				particle_configuration = 0;
			}
		}
		//if (start_height < -5) {
		//	start_height += particle_radius * 2;
		//}
	}
    
}
int main(int argc, char* argv[]) {
	omp_set_num_threads(4);
	srand(1);
    
	cout << "Mass, Radius, Friction_Sphere, Friction_Plate, Data Folder, create_particle_plate all_three_kinds, particle configuration" << endl;
	string solver_string = "ACCELERATED_PROJECTED_GRADIENT_DESCENT";
	if (argc == 3) {
		all_three_kinds = atoi(argv[1]);
		particle_configuration = atoi(argv[2]);
	}
	if (argc == 12) {
		particle_mass = atof(argv[1]);
		particle_radius = atof(argv[2]);
		particle_friction = atof(argv[3]);
		plate_friction = atof(argv[4]);
		data_folder = argv[5];
		create_particle_plate = atoi(argv[6]);
		plate_particle_radius = atof(argv[7]);
		all_three_kinds = atoi(argv[8]);
		particle_configuration = atoi(argv[9]);
		solver_string = argv[10];
		max_iteration = atoi(argv[11]);
	}
    
	cout << particle_mass << " " << particle_radius << " " << particle_friction << " " << plate_friction << " " << data_folder << " " << create_particle_plate << " " << all_three_kinds << " "
    << particle_configuration << solver_string << endl;
	//=========================================================================================================
	//=========================================================================================================
	ChSystemGPU * system_gpu = new ChSystemGPU;
	ChCollisionSystemGPU *mcollisionengine = new ChCollisionSystemGPU();
	system_gpu->SetIntegrationType(ChSystem::INT_ANITESCU);
	system_gpu->SetAABB(R3(-2,-5,-2), R3(2,5,2));
    
	//=========================================================================================================
    
	system_gpu->SetMaxiter(max_iteration);
	system_gpu->SetIterLCPmaxItersSpeed(max_iteration);
	((ChLcpSolverGPU *) (system_gpu->GetLcpSolverSpeed()))->SetMaxIteration(max_iteration);
	system_gpu->SetTol(tolerance);
	system_gpu->SetTolSpeeds(tolerance);
	((ChLcpSolverGPU *) (system_gpu->GetLcpSolverSpeed()))->SetTolerance(tolerance);
	((ChLcpSolverGPU *) (system_gpu->GetLcpSolverSpeed()))->SetCompliance(0, 0, 0);
	((ChLcpSolverGPU *) (system_gpu->GetLcpSolverSpeed()))->SetContactRecoverySpeed(10);
	//setSolverGPU(solver_string, system_gpu);     //reads a string and sets the solver
	((ChLcpSolverGPU *) (system_gpu->GetLcpSolverSpeed()))->SetSolverType(solver);
	((ChCollisionSystemGPU *) (system_gpu->GetCollisionSystem()))->SetCollisionEnvelope(particle_radius * .1);
	mcollisionengine->setBinsPerAxis(R3(40, 15, 40));
	mcollisionengine->setBodyPerBin(100, 50);
    
	//=========================================================================================================
	system_gpu->Set_G_acc(ChVector<>(0, gravity, 0));
	system_gpu->SetStep(timestep);
	//=========================================================================================================
    
	int num_particle = particle_plate_dim / plate_particle_radius;
    
	ChSharedPtr<ChMaterialSurface> material;
	material = ChSharedPtr<ChMaterialSurface>(new ChMaterialSurface);
	material->SetFriction(plate_friction);
    
	if (create_particle_plate) {
        
		ChSharedBodyPtr sphere;
		sphere = ChSharedBodyPtr(new ChBody(new ChCollisionModelGPU));
		InitObject(sphere, particle_mass, Vector(0, 0, 0), quat, material, true, true, -20, -20);
		for (int i = 0; i < num_particle; i++) {
			for (int j = 0; j < num_particle; j++) {
                
				ChVector<> position(
                                    i * plate_particle_radius * 1.2 - num_particle * plate_particle_radius * .6,
                                    plate_height + plate_thickness + plate_particle_radius,
                                    j * plate_particle_radius * 1.2 - num_particle * plate_particle_radius * .6);
                
				position.x += rand() % 10000 / 10000.0 * plate_particle_radius * .25 - plate_particle_radius * .25 * .5;
				position.z += rand() % 10000 / 10000.0 * plate_particle_radius * .25 - plate_particle_radius * .25 * .5;
                
				AddCollisionGeometry(sphere, SPHERE, ChVector<>(plate_particle_radius, plate_particle_radius, plate_particle_radius), position, quat);
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
	real funnel_height = 10;
	real funnel_width = 2;
	real funnel_h = 4;
	real funnel_offset = funnel_width;     //.5*sqrt(2)*funnel_width-funnel_thickness*6+particle_radius*3;
	ChSharedBodyPtr Lt = ChSharedBodyPtr(new ChBody(new ChCollisionModelGPU));
	ChSharedBodyPtr Rt = ChSharedBodyPtr(new ChBody(new ChCollisionModelGPU));
	ChSharedBodyPtr Ft = ChSharedBodyPtr(new ChBody(new ChCollisionModelGPU));
	ChSharedBodyPtr Bt = ChSharedBodyPtr(new ChBody(new ChCollisionModelGPU));
    
	ChSharedBodyPtr F1 = ChSharedBodyPtr(new ChBody(new ChCollisionModelGPU));
	ChSharedBodyPtr F2 = ChSharedBodyPtr(new ChBody(new ChCollisionModelGPU));
	ChSharedBodyPtr F3 = ChSharedBodyPtr(new ChBody(new ChCollisionModelGPU));
	ChSharedBodyPtr F4 = ChSharedBodyPtr(new ChBody(new ChCollisionModelGPU));
    
	ChQuaternion<> quat_r;
	quat_r.Q_from_AngX(45);
	InitObject(F1, 100000, Vector(0, plate_height + funnel_height - funnel_width, -funnel_offset), quat_r, material, true, true, -20, -20);
	quat_r.Q_from_AngX(-45);
	InitObject(F2, 100000, Vector(0, plate_height + funnel_height - funnel_width, funnel_offset), quat_r, material, true, true, -20, -20);
	quat_r.Q_from_AngZ(45);
	InitObject(F3, 100000, Vector(funnel_offset, plate_height + funnel_height - funnel_width, 0), quat_r, material, true, true, -20, -20);
	quat_r.Q_from_AngZ(-45);
	InitObject(F4, 100000, Vector(-funnel_offset, plate_height + funnel_height - funnel_width, 0), quat_r, material, true, true, -20, -20);
    
	AddCollisionGeometry(F1, BOX, Vector(funnel_width, funnel_thickness, funnel_width), lpos, quat);
	AddCollisionGeometry(F2, BOX, Vector(funnel_width, funnel_thickness, funnel_width), lpos, quat);
	AddCollisionGeometry(F3, BOX, Vector(funnel_width, funnel_thickness, funnel_width), lpos, quat);
	AddCollisionGeometry(F4, BOX, Vector(funnel_width, funnel_thickness, funnel_width), lpos, quat);
    
	FinalizeObject(F1, (ChSystemGPU *) system_gpu);
	FinalizeObject(F2, (ChSystemGPU *) system_gpu);
	FinalizeObject(F3, (ChSystemGPU *) system_gpu);
	FinalizeObject(F4, (ChSystemGPU *) system_gpu);
    
	InitObject(Lt, 100000, Vector(-funnel_width + funnel_thickness, plate_height + funnel_height, 0), quat, material, true, true, -20, -20);
	InitObject(Rt, 100000, Vector(funnel_width - funnel_thickness, plate_height + funnel_height, 0), quat, material, true, true, -20, -20);
	InitObject(Ft, 100000, Vector(0, plate_height + funnel_height, -funnel_width + funnel_thickness), quat, material, true, true, -20, -20);
	InitObject(Bt, 100000, Vector(0, plate_height + funnel_height, funnel_width - funnel_thickness), quat, material, true, true, -20, -20);
    
	AddCollisionGeometry(Lt, BOX, Vector(funnel_thickness, funnel_h, funnel_width), lpos, quat);
	AddCollisionGeometry(Rt, BOX, Vector(funnel_thickness, funnel_h, funnel_width), lpos, quat);
	AddCollisionGeometry(Ft, BOX, Vector(funnel_width, funnel_h, funnel_thickness), lpos, quat);
	AddCollisionGeometry(Bt, BOX, Vector(funnel_width, funnel_h, funnel_thickness), lpos, quat);
    
	FinalizeObject(Lt, (ChSystemGPU *) system_gpu);
	FinalizeObject(Rt, (ChSystemGPU *) system_gpu);
	FinalizeObject(Ft, (ChSystemGPU *) system_gpu);
	FinalizeObject(Bt, (ChSystemGPU *) system_gpu);
    
	//=========================================================================================================
	//////Rendering specific stuff:
	ChOpenGLManager * window_manager = new ChOpenGLManager();
	ChOpenGL openGLView(window_manager, system_gpu, 800, 600, 0, 0, "Test_Solvers");
	openGLView.render_camera->camera_pos = Vector(0, -5, -40);
	openGLView.render_camera->look_at = Vector(0, -5, 0);
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
        //			file++;
        //		}
		current_time += timestep;
        
	}
	stringstream ss;
	ss << data_folder << "/" << particle_friction << "_" << plate_friction << "_" << create_particle_plate << ".txt";
    
	DumpAllObjectsWithGeometry(system_gpu, ss.str());
	return 0;
}

