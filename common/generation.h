#include "common.h"
#include <random>
class ParticleGenerator {
	public:
		ParticleGenerator(ChSystemGPU* system) {
			mSys = system;
			mass = 1;
			use_normal_dist = false;
			material=ChSharedPtr<ChMaterialSurface>(new ChMaterialSurface);
		}

		void SetMass(real m) {
			mass = m;
		}

		void SetRadius(real3 r) {
			radius = r;
		}

		void addHCPSheet(int2 grid, real3 global, bool active, real3 vel) {
			real offset = 0;
			int grid_x = grid.x;
			int grid_z = grid.y;
			real global_x = global.x;
			real global_z = global.z;
			real x = 0, y = global.y, z = 0;

			for (int i = 0; i < grid_x; i++) {
				for (int k = 0; k < grid_z; k++) {
					body = ChSharedBodyPtr(new ChBody);

					offset = (k % 2 != 0) ? radius.x : 0;
					x = i * 2 * radius.x + offset - grid_x * 2 * radius.x / 2.0 + global_x;
					z = k * (sqrt(3.0) * radius.x) - grid_z * sqrt(3.0) * radius.x / 2.0 + global_z;

					InitObject(body, mass, Vector(x, y, z), Quaternion(1, 0, 0, 0), material, true, !active, -1, i);
					AddCollisionGeometry(body, SPHERE, ChVector<>(radius.x, radius.x, radius.x), Vector(0, 0, 0), Quaternion(1, 0, 0, 0));
					body->SetPos_dt(ChVector<>(vel.x, vel.y, vel.z));
					FinalizeObject(body, (ChSystemGPU *) mSys);
				}
			}
		}

		void addHCPCube(int3 grid, bool active, real3 global, real3 vel) {
			real offset_x = 0, offset_z = 0, height = 0;
			for (int j = 0; j < grid.y; j++) {
				height = j * (sqrt(3.0) * radius.x) * 1.15;
				offset_x = offset_z = (j % 2 != 0) ? radius.x : 0;
				real3 g = R3(offset_x + global.x, height + global.y, offset_z + global.z);
				addHCPSheet(I2(grid.x, grid.z), g, active, vel);
			}
		}

		void addPerturbedVolume(real3 origin, ShapeType type, int3 num_per_dir, real3 percent_perturbation, real3 vel, bool random = false) {

			int counter = 0;
			std::normal_distribution<double> distribution(mean, std_dev);

			for (int i = 0; i < num_per_dir.x; i++) {
				for (int j = 0; j < num_per_dir.y; j++) {
					for (int k = 0; k < num_per_dir.z; k++) {
						real3 r = radius;

						real3 a = r * percent_perturbation;
						real3 d = a + 2 * r;     //compute cell length
						real3 dp, pos;

						body = ChSharedBodyPtr(new ChBody(new ChCollisionModelGPU));

						dp.x = rand() % 10000 / 10000.0 * a.x - a.x / 2.0;
						dp.y = rand() % 10000 / 10000.0 * a.y - a.y / 2.0;
						dp.z = rand() % 10000 / 10000.0 * a.z - a.z / 2.0;

						pos.x = i * d.x - num_per_dir.x * d.x * .5;
						pos.y = j * d.y - num_per_dir.y * d.y * .5;
						pos.z = k * d.z - num_per_dir.z * d.z * .5;

						pos += dp + origin + r;

						if (use_normal_dist) {
							r.x = distribution(generator);
							r.y = distribution(generator);
							r.z = distribution(generator);
						}

						InitObject(body, mass, Vector(pos.x, pos.y, pos.z), Quaternion(1, 0, 0, 0), material, true, false, -1, counter);

						AddCollisionGeometry(body, type, ChVector<>(r.x, r.y, r.z), Vector(0, 0, 0), Quaternion(1, 0, 0, 0));

						FinalizeObject(body, (ChSystemGPU *) mSys);
						body->SetPos_dt(Vector(vel.x, vel.y, vel.z));
						counter++;
					}
				}
			}
		}

		void addVolume(real3 origin, ShapeType type, int3 num_per_dir, real3 vel) {
			addPerturbedVolume(origin, type, num_per_dir, R3(0, 0, 0), vel, false);
		}

		void SetNormalDistribution(real _mean, real _std_dev) {
			mean = _mean;
			std_dev = _std_dev;
			use_normal_dist = true;

		}

		std::default_random_engine generator;
		real mean, std_dev;
		ChSharedBodyPtr body;
		real mass;
		real3 radius;
		bool use_normal_dist;
		ChSharedPtr<ChMaterialSurface> material;
		ChSystemGPU* mSys;
};

