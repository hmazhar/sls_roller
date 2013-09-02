#include "common.h"
#include <random>
class ParticleGenerator {
	public:
		ParticleGenerator(ChSystemGPU* system) {
			mSys = system;
			mass = 1;
			use_normal_dist = false;
			use_density = false;
			use_common_material = true;
			use_normal_friction = false;
			use_normal_cohesion = false;
			material = ChSharedPtr<ChMaterialSurface>(new ChMaterialSurface);
		}

		void SetMass(real m) {
			mass = m;
		}

		void SetDensity(real d) {
			density = d;
			use_density = true;
		}

		void SetRadius(real3 r) {
			radius = r;
		}

		void addHCPSheet(int2 grid, real3 origin, bool active, real3 vel) {
			real offset = 0;
			int grid_x = grid.x;
			int grid_z = grid.y;
			real global_x = origin.x;
			real global_z = origin.z;
			real x = 0, y = origin.y, z = 0;

			for (int i = 0; i < grid_x; i++) {
				for (int k = 0; k < grid_z; k++) {
					body = ChSharedBodyPtr(new ChBody(new ChCollisionModelGPU));

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

		void addHCPCube(int3 grid, bool active, real3 origin, real3 vel) {
			real offset_x = 0, offset_z = 0, height = 0;
			for (int j = 0; j < grid.y; j++) {
				height = j * (sqrt(3.0) * radius.x) * 1.15;
				offset_x = offset_z = (j % 2 != 0) ? radius.x : 0;
				real3 new_origin = R3(offset_x + origin.x, height + origin.y, offset_z + origin.z);
				addHCPSheet(I2(grid.x, grid.z), new_origin, active, vel);
			}
		}

		void addPerturbedVolume(real3 origin, ShapeType type, int3 num_per_dir, real3 percent_perturbation, real3 vel, bool random = false) {

			int counter = 0;
			std::normal_distribution<double> distribution(mean, std_dev);
			std::normal_distribution<double> distribution_friction(mean_friction, std_dev_friction);
			std::normal_distribution<double> distribution_cohesion(mean_cohesion, std_dev_cohesion);

			for (int i = 0; i < num_per_dir.x; i++) {
				for (int j = 0; j < num_per_dir.y; j++) {
					for (int k = 0; k < num_per_dir.z; k++) {
						real3 r = radius;
						if (use_normal_dist) {
							r = r + R3(3 * std_dev);
						}

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
							r.x = fmaxf(fminf(distribution(generator), radius.x + 3 * std_dev), mean - 3 * std_dev);
							r.y = fmaxf(fminf(distribution(generator), radius.y + 3 * std_dev), mean - 3 * std_dev);
							r.z = fmaxf(fminf(distribution(generator), radius.z + 3 * std_dev), mean - 3 * std_dev);
						}
						if (use_density) {
							if (type == SPHERE) {
								mass = density * 4.0 / 3.0 * PI * r.x * r.x * r.x;
							} else if (type == BOX) {
								mass = density * r.x * r.y * r.z * 8;
							} else if (type == ELLIPSOID) {
								mass = density * 4.0 / 3.0 * PI * r.x * r.y * r.z;
							} else {
								mass = 1;

							}
						}
						if (use_common_material) {
							InitObject(body, mass, Vector(pos.x, pos.y, pos.z), Quaternion(1, 0, 0, 0), material, true, false, -1, mSys->GetNbodiesTotal());
						} else {
							InitObject(body, mass, Vector(pos.x, pos.y, pos.z), Quaternion(1, 0, 0, 0), true, false, -1, mSys->GetNbodiesTotal());
							if (use_normal_friction) {
								body->GetMaterialSurface()->SetFriction(distribution_friction(generator));
							} else {
								body->GetMaterialSurface()->SetFriction(material->GetSfriction());
							}
							if (use_normal_cohesion) {
								body->GetMaterialSurface()->SetCohesion(distribution_cohesion(generator));
							} else {
								body->GetMaterialSurface()->SetCohesion(material->GetCohesion());
							}
						}

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
		void addSnowball(real3 origin, ShapeType type, real rad, real3 vel) {
			int3 grid = I3(rad / radius.x * 2, rad / radius.x * 2, rad / radius.x * 2);
			std::normal_distribution<double> distribution(mean, std_dev);
			std::normal_distribution<double> distribution_friction(mean_friction, std_dev_friction);
			std::normal_distribution<double> distribution_cohesion(mean_cohesion, std_dev_cohesion);

			real offset_x = 0, offset_z = 0, height = 0;
			for (int j = 0; j < grid.y; j++) {
				height = j * (sqrt(3.0) * radius.x) * 1.15;
				offset_x = offset_z = (j % 2 != 0) ? radius.x : 0;
				real3 g = R3(offset_x, height, offset_z);
				{
					real offset = 0;
					int grid_x = grid.x;
					int grid_z = grid.y;
					real global_x = g.x;
					real global_z = g.z;
					real x = 0, y = g.y, z = 0;

					for (int i = 0; i < grid_x; i++) {
						for (int k = 0; k < grid_z; k++) {
							body = ChSharedBodyPtr(new ChBody(new ChCollisionModelGPU));

							offset = (k % 2 != 0) ? radius.x : 0;
							x = i * 2 * radius.x + offset - grid_x * 2 * radius.x / 2.0 + global_x;
							z = k * (sqrt(3.0) * radius.x) - grid_z * sqrt(3.0) * radius.x / 2.0 + global_z;

							real3 pos = (R3(x, y - grid.y * radius.y / 1.15, z));

							if (length(pos) < rad) {
								real3 r = radius;
								if (use_normal_dist) {
									r.x = fmaxf(distribution(generator), mean - 2 * std_dev);
									r.y = fmaxf(distribution(generator), mean - 2 * std_dev);
									r.z = fmaxf(distribution(generator), mean - 2 * std_dev);
								}

								pos += origin;
								if (use_common_material) {
									InitObject(body, mass, Vector(pos.x, pos.y, pos.z), Quaternion(1, 0, 0, 0), material, true, false, -1, mSys->GetNbodiesTotal());
								} else {
									InitObject(body, mass, Vector(pos.x, pos.y, pos.z), Quaternion(1, 0, 0, 0), true, false, -1, mSys->GetNbodiesTotal());
									if (use_normal_friction) {
										body->GetMaterialSurface()->SetFriction(distribution_friction(generator));
									} else {
										body->GetMaterialSurface()->SetFriction(material->GetSfriction());
									}
									if (use_normal_cohesion) {
										body->GetMaterialSurface()->SetCohesion(distribution_cohesion(generator));
									} else {
										body->GetMaterialSurface()->SetCohesion(material->GetCohesion());
									}
									body->GetMaterialSurface()->SetCompliance(material->GetCompliance());

								}

								AddCollisionGeometry(body, type, ChVector<>(r.x, r.y, r.z), Vector(0, 0, 0), Quaternion(1, 0, 0, 0));

								FinalizeObject(body, (ChSystemGPU *) mSys);
								body->SetPos_dt(Vector(vel.x, vel.y, vel.z));
							}

						}
					}
				}
			}

		}

		void SetNormalDistribution(real _mean, real _std_dev) {
			mean = _mean;
			std_dev = _std_dev;
			use_normal_dist = true;

		}

		void UseNormalFriction(real _mean, real _std_dev) {
			use_common_material = false;
			use_normal_friction = true;
			mean_friction = _mean;
			std_dev_friction = _std_dev;

		}
		void UseNormalCohesion(real _mean, real _std_dev) {
			use_common_material = false;
			use_normal_cohesion = true;
			mean_cohesion = _mean;
			std_dev_cohesion = _std_dev;
		}

		std::default_random_engine generator;
		real mean, std_dev;
		ChSharedBodyPtr body;
		real mass;
		real density;
		real3 radius;
		bool use_normal_dist;
		bool use_density;
		bool use_common_material;
		ChSharedPtr<ChMaterialSurface> material;
		ChSystemGPU* mSys;

		bool use_normal_friction;
		real mean_friction, std_dev_friction;

		bool use_normal_cohesion;
		real mean_cohesion, std_dev_cohesion;

		vector<real3> position;
};

