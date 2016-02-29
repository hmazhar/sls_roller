#include <stdio.h>
#include <vector>
#include <cmath>
#include <string>

#include "common/input_output.h"

using std::cout;
using std::endl;

// all dimensions are in millimeters, milligrams
real container_width = 5;        // width of area with particles
real container_length = 25;      // length of area that roller will go over		1194mm maximum
real container_thickness = .25;  // thickness of container walls
real container_height = 2;       // height of the outer walls
real container_friction = 0;
real floor_friction = .2;
real spacer_width = 1;
real spacer_height = 1;

real roller_overlap = 1;          // amount that roller goes over the container area
real roller_length = 2.5 - .25;   // length of the roller
real roller_radius = 76.2 / 2.0;  // radius of roller
real roller_omega = 0;
real roller_velocity = -127;
real roller_mass = 1;
real roller_friction = .1;
real roller_cohesion = 0;
real particle_radius = .058 / 2.0;
real particle_std_dev = .015 / 2.0;
real particle_mass = .05;
real particle_density = 0.93;
real particle_layer_thickness = particle_radius * 16;
real particle_friction = .52;
real rolling_friction = .1;
real spinning_friction = .1;
real gravity = -9810;    // acceleration due to gravity
real timestep = .00001;  // step size
real time_end = 1;       // length of simulation
real current_time = 0;
int out_fps = 6000;
int out_steps = std::ceil((1.0 / timestep) / out_fps);

int num_steps = time_end / timestep;
int max_iteration = 20;
int tolerance = 1e-8;

std::string data_output_path = "data_sls";
std::shared_ptr<ChBody> ROLLER;
real ang = 0;

template <class T>
void RunTimeStep(T* mSys, const int frame) {
    ChVector<> roller_pos = ROLLER->GetPos();

    ROLLER->SetPos(ChVector<>(0, roller_radius + particle_layer_thickness + container_thickness,
                              roller_pos.z + roller_velocity * timestep));
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
    roller_quat.Q_from_AngAxis(CH_C_PI / 2.0, ChVector<>(0, 0, 1));

    ROLLER->SetRot(q1 % roller_quat);
    ROLLER->SetWvel_loc(Vector(0, roller_omega, 0));

    cout << "step " << frame << " " << ROLLER->GetPos().z << "\n";
}

int main(int argc, char* argv[]) {
    real roller_start = container_length + roller_radius / 3.0;

    time_end = (roller_start) / Abs(roller_velocity);

    printf("Time to run: %f %f\n", roller_start, time_end);

    num_steps = time_end / timestep;

    ChSystemParallelDVI* system = new ChSystemParallelDVI;
    system->Set_G_acc(ChVector<>(0, gravity, 0));
    system->SetIntegrationType(ChSystem::INT_ANITESCU);

    system->GetSettings()->min_threads = 8;

    system->GetSettings()->solver.tolerance = tolerance;
    system->GetSettings()->solver.solver_mode = SPINNING;
    system->GetSettings()->solver.max_iteration_normal = max_iteration;
    system->GetSettings()->solver.max_iteration_sliding = max_iteration;
    system->GetSettings()->solver.max_iteration_spinning = max_iteration;
    system->GetSettings()->solver.max_iteration_bilateral = 0;  // make 1000, should be about 220
    system->GetSettings()->solver.compute_N = false;
    system->GetSettings()->solver.alpha = 0;
    system->GetSettings()->solver.cache_step_length = true;
    system->GetSettings()->solver.use_full_inertia_tensor = false;
    system->GetSettings()->solver.contact_recovery_speed = 180;
    system->GetSettings()->solver.bilateral_clamp_speed = 1e8;
    system->ChangeSolverType(BB);
    system->SetLoggingLevel(LOG_INFO);
    system->SetLoggingLevel(LOG_TRACE);

    system->GetSettings()->collision.collision_envelope = particle_radius * .05;
    system->GetSettings()->collision.bins_per_axis = int3(40, 300, 400);
    system->GetSettings()->collision.narrowphase_algorithm = NARROWPHASE_HYBRID_MPR;
    system->GetSettings()->collision.use_two_level = false;
    system->GetSettings()->collision.fixed_bins = true;

    auto material_plate = std::make_shared<ChMaterialSurface>();
    material_plate->SetFriction(0);
    std::shared_ptr<ChBody> PLATE = std::make_shared<ChBody>(new ChCollisionModelParallel);
    utils::InitializeObject(PLATE, 100000, material_plate, ChVector<>(0, 0, 0), QUNIT, true, true, 2, 6);

    utils::AddBoxGeometry(PLATE.get(), Vector(container_thickness, container_height, container_length),
                          Vector(-container_width + container_thickness, container_height, 0));
    utils::AddBoxGeometry(PLATE.get(), Vector(container_thickness, container_height, container_length),
                          Vector(container_width - container_thickness, container_height, 0));
    utils::AddBoxGeometry(PLATE.get(), Vector(container_width, container_height, container_thickness),
                          Vector(0, container_height, -container_length + container_thickness));
    utils::AddBoxGeometry(PLATE.get(), Vector(container_width, container_height, container_thickness),
                          Vector(0, container_height, container_length - container_thickness));
    utils::AddBoxGeometry(PLATE.get(), Vector(container_width, container_thickness, container_length),
                          Vector(0, container_height * 2, 0));

    FinalizeObject(PLATE, (ChSystemParallel*)system);

    auto material_bottom = std::make_shared<ChMaterialSurface>();
    material_bottom->SetFriction(floor_friction);
    std::shared_ptr<ChBody> BOTTOM = std::make_shared<ChBody>(new ChCollisionModelParallel);
    utils::InitializeObject(BOTTOM, 100000, material_bottom, ChVector<>(0, 0, 0), QUNIT, true, true, 2, 6);
    utils::AddBoxGeometry(BOTTOM.get(), ChVector<>(container_width, container_thickness, container_length));
    FinalizeObject(BOTTOM, (ChSystemParallel*)system);

    ROLLER = std::make_shared<ChBody>(new ChCollisionModelParallel);
    ChQuaternion<> roller_quat;
    roller_quat.Q_from_AngAxis(CH_C_PI / 2.0, ChVector<>(0, 0, 1));

    auto material_roller = std::make_shared<ChMaterialSurface>();
    material_roller->SetFriction(roller_friction);

    utils::InitializeObject(ROLLER, 100000, material_roller,
                            ChVector<>(0, roller_radius + particle_layer_thickness + container_thickness, roller_start),
                            roller_quat, true, false, 6, 6);

    utils::AddCylinderGeometry(ROLLER.get(), roller_radius, roller_length * 2);
    FinalizeObject(ROLLER, (ChSystemParallel*)system);

    auto material_granular = std::make_shared<ChMaterialSurface>();
    material_granular->SetFriction(particle_friction);
    material_granular->SetRollingFriction(rolling_friction);
    material_granular->SetSpinningFriction(spinning_friction);

    utils::Generator* gen = new utils::Generator(system);

    std::shared_ptr<MixtureIngredient>& m1 = gen->AddMixtureIngredient(utils::SPHERE, 1);
    m1->setDefaultSize(particle_radius);
    m1->setDefaultDensity(particle_density);
    m1->setDistributionSize(particle_radius, particle_std_dev, particle_radius - particle_std_dev,
                            particle_radius + particle_std_dev);
    m1->setDefaultMaterialDVI(material_granular);

    gen->createObjectsBox(utils::HCP_PACK, (particle_radius + particle_std_dev) * 2, ChVector<>(0, 1.0, 0),
                          ChVector<>(container_width * .9, particle_layer_thickness, container_length * .9));

    opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
    gl_window.Initialize(1280, 720, "Bucky", system);
    gl_window.SetCamera(ChVector<>(0, 0, -10), ChVector<>(0, 0, 0), ChVector<>(0, 1, 0), 0.1);
    gl_window.Pause();
    int frame = 0;
#if 0
    while (frame < num_steps) {
        if (gl_window.Active()) {
            if (gl_window.DoStepDynamics(timestep)) {
                // TimingOutput(system);
                RunTimeStep(system, frame);
                frame++;
            }
            gl_window.Render();
        } else {
            exit(0);
        }
    }
#else

    double time = 0, exec_time = 0;
    int sim_frame = 0, out_frame = 0, next_out_frame = 0;

    while (time < time_end) {
        system->DoStepDynamics(timestep);
        if (sim_frame == next_out_frame) {
            std::cout << "write: " << out_frame << std::endl;
            DumpAllObjectsWithGeometryPovray(system, data_output_path + "data_" + std::to_string(out_frame) + ".dat",
                                             true);
            out_frame++;
            next_out_frame += out_steps;
        }
        RunTimeStep(system, frame);

        // Update counters.
        time += timestep;
        sim_frame++;
        exec_time += system->GetTimerStep();
    }
    cout << "==================================" << endl;
    cout << "Simulation time:   " << exec_time << endl;

#endif
}
