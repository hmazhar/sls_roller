#ifndef CHRONOMODELS_GENERATE_H
#define CHRONOMODELS_GENERATE_H

#include "common.h"
#include "initialization.h"
#include "input_output.h"
#include <random>

enum MixType {
	MIX_SPHERE, MIX_ELLIPSOID, MIX_DOUBLESPHERE, MIX_CUBE, MIX_CUBE_SPHERE, MIX_CYLINDER, MIX_PILL, MIX_CONE, MIX_TYPE1, MIX_TYPE2, MIX_TYPE3, MIX_TYPE4
};
class VoronoiSampler {
	public:

		VoronoiSampler(int seed_p = 200, int seed = 1) {
			srand(clock());
			seed_points = seed_p;
			boundary = .02;
			counter = 0;
		}

		void Seed() {
			for (int i = 0; i < seed_points; i++) {
				real3 value;
				value.x = GenerateRandom(min_point.x, max_point.x);
				value.y = GenerateRandom(min_point.y, max_point.y);
				value.z = GenerateRandom(min_point.z, max_point.z);
				points.push_back(value);
				values.push_back(distribution->operator()(generator));
			}
		}

		real GenerateRandom(real min_v, real max_v) {

			real random_v = rand() % 10000 / 10000.0;     //random value 0,1;
			real value = lerp(min_v, max_v, random_v);
			return value;
		}

		void SetSize(real3 min_p, real3 max_p) {
			min_point = min_p;
			max_point = max_p;
		}
		void SetNormalDist(real mean_, real std_dev_) {
			mean = mean_;
			std_dev = std_dev_;
			distribution = new std::normal_distribution<double>(mean, std_dev);
		}

		real GetProperty(real3 point) {
			vector<real> distance(points.size());
			for (int i = 0; i < seed_points; i++) {
				distance[i] = length(point - points[i]);
			}
			thrust::sort_by_key(distance.begin(), distance.end(), values.begin());

			real bound = fabs(distance[seed_points - 1] - distance[seed_points - 2]);

			if (bound < boundary) {
				counter++;
				//cout << counter << endl;
				return values[seed_points - 1] / 10.0;
			}
			//cout<<values<<values[seed_points - 1];
			return values[seed_points - 1];
		}
		void SetBoundary(real boundary_t) {
			boundary = boundary_t;
		}
	private:

		real3 min_point, max_point;
		int seed_points;
		thrust::host_vector<real3> points;
		thrust::host_vector<int> values;
		real mean, std_dev;
		std::default_random_engine generator;
		std::normal_distribution<double>* distribution;
		real boundary;
		int counter;

};

template<class T>
class ParticleGenerator {
	public:
		ParticleGenerator(T* system, bool GPU = true) {
			mSys = system;
			mass = 1;
			use_normal_dist = false;
			use_density = false;
			use_common_material = true;
			use_normal_friction = false;
			use_normal_cohesion = false;
			material = ChSharedPtr<ChMaterialSurface>(new ChMaterialSurface);
			use_mixture = false;
			volume_cylinder = false;
			useGPU = GPU;
			total_volume = total_mass = 0;
			active = false;
		}

		void SetMass(real m) {
			mass = m;
		}
		void SetActive(bool act) {
			active = act;
		}
		void SetDensity(real d) {
			density = d;
			use_density = true;
		}

		void SetRadius(real3 r) {
			radius = r;
		}

		void createBody(ChSharedBodyPtr &body, real3 pos, real mass) {

			if (useGPU) {
				body = ChSharedBodyPtr(new ChBody(new ChCollisionModelParallel));
			} else {
				body = ChSharedBodyPtr(new ChBody());
			}
			body_list.push_back(body);
			if (use_common_material) {
				if (useGPU) {
					InitObject(body, mass, Vector(pos.x, pos.y, pos.z), Quaternion(1, 0, 0, 0), material, true, active, -1, mSys->GetNbodiesTotal());
				} else {
					InitObject(body, mass, Vector(pos.x, pos.y, pos.z), Quaternion(1, 0, 0, 0), material, true, active, 2, 4);
				}
			} else {
				InitObject(body, mass, Vector(pos.x, pos.y, pos.z), Quaternion(1, 0, 0, 0), true, active, 2, mSys->GetNbodiesTotal() + 2);
				if (use_normal_friction) {
					body->GetMaterialSurface()->SetFriction(distribution_friction->operator()(generator));
				} else {
					body->GetMaterialSurface()->SetFriction(material->GetSfriction());
				}
				if (use_normal_cohesion) {
					body->GetMaterialSurface()->SetCohesion(distribution_cohesion->operator()(generator));
				} else {
					body->GetMaterialSurface()->SetCohesion(material->GetCohesion());
				}
			}
		}

		void computeMassType(ShapeType type, real3 r) {
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
		}
		real computeMassMixture(MixType type, real3 r) {

			if (use_density) {
				if (type == MIX_SPHERE) {
					mass = density * 4.0 / 3.0 * PI * r.x * r.x * r.x;
				} else if (type == MIX_DOUBLESPHERE) {
					mass = density * 4.0 / 3.0 * PI * r.x * r.x * r.x * 2;
				} else if (type == MIX_ELLIPSOID) {
					mass = density * 4.0 / 3.0 * PI * r.x * r.y * r.z;
				} else if (type == MIX_CUBE) {
					mass = density * r.x * 2 * r.y * 2 * r.z * 2;
				} else if (type == MIX_TYPE1) {
					mass = density * 4.0 / 3.0 * PI * r.x * r.x * r.x * 2;
				} else if (type == MIX_TYPE2) {
					mass = density * 4.0 / 3.0 * PI * r.x * r.y * r.z;
				} else if (type == MIX_TYPE3) {
					mass = density * 4.0 / 3.0 * PI * r.x * r.x * r.x;
				} else if (type == MIX_TYPE4) {
					mass = density * 4.0 / 3.0 * PI * r.x * 2 * r.y * r.z;
				} else {
					mass = 1;
				}
			}
			return mass;
		}
		void computeRadius(real3 & r) {

			if (use_normal_dist && std_dev > 0) {
				real3 min_r = R3(radius.x - num_std_dev * std_dev, radius.y - num_std_dev * std_dev, radius.z - num_std_dev * std_dev);
				real3 max_r = R3(radius.x + num_std_dev * std_dev, radius.y + num_std_dev * std_dev, radius.z + num_std_dev * std_dev);
				r.x = distribution->operator()(generator);
				while (r.x < min_r.x || r.x > max_r.x) {
					r.x = distribution->operator()(generator);
				}
				r.y = distribution->operator()(generator);
				while (r.y < min_r.y || r.y > max_r.y) {
					r.y = distribution->operator()(generator);
				}

				r.z = distribution->operator()(generator);
				while (r.z < min_r.z || r.z > max_r.z) {
					r.z = distribution->operator()(generator);
				}

			} else {
				r = radius;
			}
		}

		bool computePerturbedPos(real3 percent_perturbation, int3 num_per_dir, int3 index, real3 origin, real3 & pos) {

			real3 r = R3(max(max(radius.x, radius.y), radius.z)) + R3(std_dev) * num_std_dev*2.4 * use_normal_dist;

			real3 a = r * percent_perturbation;
			real3 d = a + 2 * r;     //compute cell length
			real3 dp;

			dp.x = rand() % 10000 / 10000.0 * a.x - a.x / 2.0;
			dp.y = rand() % 10000 / 10000.0 * a.y - a.y / 2.0;
			dp.z = rand() % 10000 / 10000.0 * a.z - a.z / 2.0;

			pos.x = index.x * d.x - num_per_dir.x * d.x * .5;
			pos.y = index.y * d.y - num_per_dir.y * d.y * .5;
			pos.z = index.z * d.z - num_per_dir.z * d.z * .5;

			pos += dp + r;

			if (volume_cylinder && sqrt(pos.x * pos.x + pos.z * pos.z) < cylinder_rad) {
				pos += origin;
				return true;

			} else if (volume_cylinder && sqrt(pos.x * pos.x + pos.z * pos.z) > cylinder_rad) {
				return false;
			}
			pos += origin;
			return true;
		}

		void addHCPSheet(int2 grid, real3 origin, real3 vel) {
			real offset = 0;
			int grid_x = grid.x;
			int grid_z = grid.y;
			real global_x = origin.x;
			real global_z = origin.z;
			real x = 0, y = origin.y, z = 0;

			for (int i = 0; i < grid_x; i++) {
				for (int k = 0; k < grid_z; k++) {
					offset = (k % 2 != 0) ? radius.x : 0;
					x = i * 2 * radius.x + offset - grid_x * 2 * radius.x / 2.0 + global_x;
					z = k * (sqrt(3.0) * radius.x) - grid_z * sqrt(3.0) * radius.x / 2.0 + global_z;

					createBody(body, R3(x, y, z), mass);
					AddCollisionGeometry(body, SPHERE, ChVector<>(radius.x, radius.x, radius.x), Vector(0, 0, 0), Quaternion(1, 0, 0, 0));
					body->SetPos_dt(ChVector<>(vel.x, vel.y, vel.z));
					FinalizeObject(body, (T *) mSys);
				}
			}
		}

		void addHCPCube(int3 grid, real3 origin, real3 vel) {
			real offset_x = 0, offset_z = 0, height = 0;
			for (int j = 0; j < grid.y; j++) {
				height = j * (sqrt(3.0) * radius.x) * 1.15;
				offset_x = offset_z = (j % 2 != 0) ? radius.x : 0;
				real3 new_origin = R3(offset_x + origin.x, height + origin.y, offset_z + origin.z);
				addHCPSheet(I2(grid.x, grid.z), new_origin, vel);
			}
		}

		void addPerturbedVolumeMixture(real3 origin, int3 num_per_dir, real3 percent_perturbation, real3 vel, bool random = false) {
			int counter = 0;
			int mix_type = 0;
			for (int i = 0; i < num_per_dir.x; i++) {
				for (int j = 0; j < num_per_dir.y; j++) {
					for (int k = 0; k < num_per_dir.z; k++) {
						real3 r = R3(0), pos = R3(0);
						mix_type = rand() % mixture.size();

						bool success = computePerturbedPos(percent_perturbation, num_per_dir, I3(i, j, k), origin, pos);
						if (success == false) {
							continue;
						}

						computeRadius(r);
						real mass = computeMassMixture(mixture[mix_type], r);
						total_mass += mass;
						createBody(body, pos, mass);

						if (mixture[mix_type] == MIX_SPHERE) {
							AddCollisionGeometry(body, SPHERE, ChVector<>(r.x, r.y, r.z), Vector(0, 0, 0), Quaternion(1, 0, 0, 0));

							if (useGPU == false) {
								body->SetInertiaXX(ChVector<>(2 / 5.0 * mass * r.x * r.x, 2 / 5.0 * mass * r.x * r.x, 2 / 5.0 * mass * r.x * r.x));
							}
						} else if (mixture[mix_type] == MIX_DOUBLESPHERE) {
							AddCollisionGeometry(body, SPHERE, ChVector<>(r.x, r.y, r.z), Vector(-r.x / 3.0, 0, 0), Quaternion(1, 0, 0, 0));
							AddCollisionGeometry(body, SPHERE, ChVector<>(r.x, r.y, r.z), Vector(r.x / 3.0, 0, 0), Quaternion(1, 0, 0, 0));
						} else if (mixture[mix_type] == MIX_ELLIPSOID) {
							AddCollisionGeometry(body, ELLIPSOID, ChVector<>(r.x, r.y, r.z), Vector(0, 0, 0), Quaternion(1, 0, 0, 0));
							if (useGPU == false) {
								body->SetInertiaXX(ChVector<>(1 / 5.0 * mass * (r.y * r.y + r.z * r.z), 1 / 5.0 * mass * (r.x * r.x + r.z * r.z), 1 / 5.0 * mass * (r.x * r.x + r.y * r.y)));
							}
						} else if (mixture[mix_type] == MIX_CUBE) {
							AddCollisionGeometry(body, BOX, ChVector<>(r.x, r.y, r.z), Vector(0, 0, 0), Quaternion(1, 0, 0, 0));
							if (useGPU == false) {
								body->SetInertiaXX(ChVector<>(1 / 12.0 * mass * (r.y * r.y + r.z * r.z), 1 / 12.0 * mass * (r.x * r.x + r.z * r.z), 1 / 12.0 * mass * (r.x * r.x + r.y * r.y)));
							}
						} else if (mixture[mix_type] == MIX_CUBE_SPHERE) {
							AddCollisionGeometry(body, BOX, ChVector<>(r.x, r.y, r.z), Vector(0, 0, 0), Quaternion(1, 0, 0, 0));
							AddCollisionGeometry(body, SPHERE, ChVector<>(r.x / 10.0, r.y, r.z), Vector(-r.x, -r.y, -r.y), Quaternion(1, 0, 0, 0));
							AddCollisionGeometry(body, SPHERE, ChVector<>(r.x / 10.0, r.y, r.z), Vector(-r.x, -r.y, r.y), Quaternion(1, 0, 0, 0));
							AddCollisionGeometry(body, SPHERE, ChVector<>(r.x / 10.0, r.y, r.z), Vector(r.x, -r.y, -r.y), Quaternion(1, 0, 0, 0));
							AddCollisionGeometry(body, SPHERE, ChVector<>(r.x / 10.0, r.y, r.z), Vector(r.x, -r.y, r.y), Quaternion(1, 0, 0, 0));
							AddCollisionGeometry(body, SPHERE, ChVector<>(r.x / 10.0, r.y, r.z), Vector(-r.x, r.y, -r.y), Quaternion(1, 0, 0, 0));
							AddCollisionGeometry(body, SPHERE, ChVector<>(r.x / 10.0, r.y, r.z), Vector(r.x, r.y, -r.y), Quaternion(1, 0, 0, 0));
							AddCollisionGeometry(body, SPHERE, ChVector<>(r.x / 10.0, r.y, r.z), Vector(-r.x, r.y, r.y), Quaternion(1, 0, 0, 0));
							AddCollisionGeometry(body, SPHERE, ChVector<>(r.x / 10.0, r.y, r.z), Vector(r.x, r.y, r.y), Quaternion(1, 0, 0, 0));
						} else if (mixture[mix_type] == MIX_CYLINDER) {
							AddCollisionGeometry(body, CYLINDER, ChVector<>(r.x, r.y, r.z), Vector(0, 0, 0), Quaternion(1, 0, 0, 0));
							if (useGPU == false) {
								body->SetInertiaXX(ChVector<>(1 / 12.0 * mass * (3 * r.x * r.x + r.y * r.y), 1 / 2.0 * mass * (r.x * r.x), 1 / 12.0 * mass * (3 * r.x * r.x + r.y * r.y)));
							}

						} else if (mixture[mix_type] == MIX_PILL) {
							AddCollisionGeometry(body, CYLINDER, ChVector<>(r.x, r.y, r.z), Vector(0, 0, 0), Quaternion(1, 0, 0, 0));
							AddCollisionGeometry(body, SPHERE, ChVector<>(r.x, r.y, r.z), Vector(0, r.y, 0), Quaternion(1, 0, 0, 0));
							AddCollisionGeometry(body, SPHERE, ChVector<>(r.x, r.y, r.z), Vector(0, -r.y, 0), Quaternion(1, 0, 0, 0));

						} else if (mixture[mix_type] == MIX_CONE) {
							AddCollisionGeometry(body, CONE, ChVector<>(r.x, r.y, r.z), Vector(0, 0, 0), Quaternion(1, 0, 0, 0));
						} else if (mixture[mix_type] == MIX_TYPE1) {
							AddCollisionGeometry(body, SPHERE, ChVector<>(r.x, r.y, r.z), Vector(-r.x / 4.0, 0, 0), Quaternion(1, 0, 0, 0));
							AddCollisionGeometry(body, SPHERE, ChVector<>(r.x, r.y, r.z) * .75, Vector(r.x / 4.0, 0, 0), Quaternion(1, 0, 0, 0));
						} else if (mixture[mix_type] == MIX_TYPE2) {
							AddCollisionGeometry(body, SPHERE, ChVector<>(r.x * .75, r.y, r.z), Vector(-r.x / 3.0, 0, 0), Quaternion(1, 0, 0, 0));
							AddCollisionGeometry(body, ELLIPSOID, ChVector<>(r.x*.75, r.y * .5, r.z * .5), Vector(r.x / 3.0, 0, 0), Quaternion(1, 0, 0, 0));
						} else if (mixture[mix_type] == MIX_TYPE3) {
							AddCollisionGeometry(body, SPHERE, ChVector<>(r.x*.75, r.y, r.z), Vector(0, 0, 0), Quaternion(1, 0, 0, 0));
							AddCollisionGeometry(body, ELLIPSOID, ChVector<>(r.x , r.y*.5, r.z*.5), Vector(0, 0, 0), Quaternion(1, 0, 0, 0));
//							AddCollisionGeometry(body, SPHERE, ChVector<>(r.x, r.y, r.z), Vector(0, 0, 0), Quaternion(1, 0, 0, 0));
//							AddCollisionGeometry(body, ELLIPSOID, ChVector<>(r.x * .75, r.y * .75, r.z * .75), Vector(0, r.x, 0), Quaternion(1, 0, 0, 0));
						} else if (mixture[mix_type] == MIX_TYPE4) {
							AddCollisionGeometry(body, SPHERE, ChVector<>(r.x, r.y, r.z) * .75, Vector(-r.x / 3.0, 0, 0), Quaternion(1, 0, 0, 0));
							AddCollisionGeometry(body, SPHERE, ChVector<>(r.x, r.y, r.z) * .75, Vector(r.x / 3.0, 0, 0), Quaternion(1, 0, 0, 0));
							AddCollisionGeometry(body, ELLIPSOID, ChVector<>(r.x * .5, r.y, r.z), Vector(0, 0, 0), Quaternion(1, 0, 0, 0));
						}
						//mix_type++;
//						if (mix_type > mixture.size()) {
//							mix_type = 0;
//						}

						FinalizeObject(body, (T *) mSys);
						body->SetPos_dt(Vector(vel.x, vel.y, vel.z));

						if (useGPU == true) {
							total_volume += ((ChCollisionModelParallel*) body->GetCollisionModel())->getVolume();
						}
						counter++;
					}
				}
			}
		}

		void addVolume(real3 origin, int3 num_per_dir, real3 vel) {
			addPerturbedVolumeMixture(origin, num_per_dir, R3(0, 0, 0), vel, false);
		}

		void addSnowball(real3 origin, ShapeType type, real rad, real3 vel) {

			snow_sampler.SetSize(R3(-rad * 2, -rad * 2, -rad * 2), R3(rad * 2, rad * 2, rad * 2));
			snow_sampler.SetNormalDist(mean_cohesion, std_dev_cohesion);
			snow_sampler.Seed();

			int3 grid = I3(rad / radius.x * 2, rad / radius.x * 2, rad / radius.x * 2);

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

							offset = (k % 2 != 0) ? radius.x : 0;
							x = i * 2 * radius.x + offset - grid_x * 2 * radius.x / 2.0 + global_x;
							z = k * (sqrt(3.0) * radius.x) - grid_z * sqrt(3.0) * radius.x / 2.0 + global_z;

							real3 pos = (R3(x, y - grid.y * radius.y / 1.15, z));

							if (length(pos) < rad) {
								real3 r = radius;
								computeRadius(r);
								pos += origin;

								createBody(body, pos, mass);
								AddCollisionGeometry(body, type, ChVector<>(r.x, r.y, r.z), Vector(0, 0, 0), Quaternion(1, 0, 0, 0));
								FinalizeObject(body, (T *) mSys);
								body->SetPos_dt(Vector(vel.x, vel.y, vel.z));
							}

						}
					}
				}
			}

		}

		void SetNormalDistribution(real _mean, real _std_dev, int _num_std_dev = 1) {
			mean = _mean;
			std_dev = _std_dev;
			use_normal_dist = true;
			num_std_dev = _num_std_dev;
			distribution = new std::normal_distribution<double>(mean, std_dev);
		}

		void UseNormalFriction(real _mean, real _std_dev) {
			use_common_material = false;
			use_normal_friction = true;
			mean_friction = _mean;
			std_dev_friction = _std_dev;
			distribution_friction = new std::normal_distribution<double>(mean_friction, std_dev_friction);

		}
		void UseNormalCohesion(real _mean, real _std_dev) {
			use_common_material = false;
			use_normal_cohesion = true;
			mean_cohesion = _mean;
			std_dev_cohesion = _std_dev;
			distribution_cohesion = new std::normal_distribution<double>(mean_cohesion, std_dev_cohesion);
		}

		void AddMixtureType(MixType type) {
			use_mixture = true;
			mixture.push_back(type);

		}

		void SetCylinderRadius(real rad) {
			volume_cylinder = true;
			cylinder_rad = rad;

		}

		void DumpAscii(string filename, bool only_active = false) {

			CSVGen csv_output;
			csv_output.delim = " ";
			csv_output.OpenFile(filename.c_str());
			for (int i = 0; i < body_list.size(); i++) {
				if (only_active) {
					if (body_list[i]->IsActive() == false) {
						continue;
					}
				}

				csv_output << R3(body_list[i]->GetPos().x, body_list[i]->GetPos().y, body_list[i]->GetPos().z);
				csv_output.Endline();
			}
			csv_output.CloseFile();
		}

		void loadAscii(string filename, real3 origin, ShapeType type, real3 rad, real3 vel, real3 scale) {
			ifstream ifile(filename.c_str());
			string temp;

			while (ifile.fail() == false) {
				getline(ifile, temp);
				if (ifile.fail() == true) {
					return;
				}
				stringstream ss;
				ss << temp;
				real3 pos;
				ss >> pos.x >> pos.y >> pos.z;

				createBody(body, pos * scale + origin, mass);
				AddCollisionGeometry(body, type, ChVector<>(rad.x, rad.y, rad.z), Vector(0, 0, 0), Quaternion(1, 0, 0, 0));
				FinalizeObject(body, (T *) mSys);
				body->SetPos_dt(Vector(vel.x, vel.y, vel.z));
			}
			ifile.close();
		}

		std::default_random_engine generator;
		real mean, std_dev;
		ChSharedBodyPtr body;
		real mass;
		real density;
		real3 radius;
		real total_volume, total_mass;
		bool use_normal_dist;
		bool use_density;
		bool use_common_material;
		bool useGPU;
		bool active;
		real cylinder_rad;
		bool volume_cylinder;
		ChSharedPtr<ChMaterialSurface> material;
		T* mSys;

		bool use_normal_friction;
		real mean_friction, std_dev_friction;

		bool use_normal_cohesion;
		real mean_cohesion, std_dev_cohesion;

		vector<real3> position;
		int num_std_dev;
		bool use_mixture;
		vector<MixType> mixture;
		vector<ChSharedBodyPtr> body_list;

		std::normal_distribution<double>* distribution;
		std::normal_distribution<double> *distribution_friction;
		std::normal_distribution<double> *distribution_cohesion;

		VoronoiSampler snow_sampler;
};
#endif
