#include "common.h"

class CSVGen {
	public:

		CSVGen() {
			delim = ",";
		}
		~CSVGen() {
		}

		void OpenFile(string filename) {
			ofile.open(filename.c_str());
		}

		void CloseFile() {

			ofile << ss.str();

			ofile.close();
		}
		template<class T>
		void operator<<(T token) {
			WriteToken(token);
		}
		void WriteToken(real token) {
			ss << token << delim;
		}
		void WriteToken(real2 token) {
			ss << token.x << delim << token.y << delim;
		}
		void WriteToken(real3 token) {
			ss << token.x << delim << token.y << delim << token.z << delim;
		}
		void WriteToken(real4 token) {
			ss << token.w << delim << token.x << delim << token.y << delim << token.z << delim;
		}
		void WriteToken(string token) {
			ss << token << delim;
		}
		void Endline() {
			ss << endl;
		}

		string delim;
		ofstream ofile;
		stringstream ss;

};

template<class T>
void DumpObjects(T* mSys, string filename, string delim = ",", bool dump_vel_rot = false) {
	CSVGen csv_output;
	csv_output.OpenFile(filename.c_str());

	for (int i = 0; i < mSys->Get_bodylist()->size(); i++) {
		ChBody* abody = mSys->Get_bodylist()->at(i);
		if (abody->IsActive() == true) {

			csv_output << R3(abody->GetPos().x, abody->GetPos().y, abody->GetPos().z);
			csv_output << R4(abody->GetRot().e0, abody->GetRot().e1, abody->GetRot().e2, abody->GetRot().e3);

			if (!dump_vel_rot) {
				csv_output.Endline();
			}
			if (dump_vel_rot) {
				csv_output << R3(abody->GetPos_dt().x, abody->GetPos_dt().y, abody->GetPos_dt().z);
				csv_output << R3(abody->GetWvel_loc().x, abody->GetWvel_loc().y, abody->GetWvel_loc().z);

				csv_output.Endline();

			}
		}
	}
	csv_output.CloseFile();
}
template<class T>
void DumpAllObjects(T* mSys, string filename, string delim = ",", bool dump_vel_rot = false) {
	CSVGen csv_output;
	csv_output.OpenFile(filename.c_str());
	for (int i = 0; i < mSys->Get_bodylist()->size(); i++) {
		ChBody* abody = mSys->Get_bodylist()->at(i);
		csv_output << R3(abody->GetPos().x, abody->GetPos().y, abody->GetPos().z);
		csv_output << R4(abody->GetRot().e0, abody->GetRot().e1, abody->GetRot().e2, abody->GetRot().e3);

		if (!dump_vel_rot) {
			csv_output.Endline();
		}
		if (dump_vel_rot) {
			csv_output << R3(abody->GetPos_dt().x, abody->GetPos_dt().y, abody->GetPos_dt().z);
			csv_output << R3(abody->GetWvel_loc().x, abody->GetWvel_loc().y, abody->GetWvel_loc().z);

			csv_output.Endline();

		}
	}
	csv_output.CloseFile();
}
void DumpAllObjectsWithGeometryPovray(ChSystemParallel* mSys, string filename) {

	CSVGen csv_output;
	csv_output.OpenFile(filename.c_str());
	for (int i = 0; i < mSys->Get_bodylist()->size(); i++) {
		ChBody* abody = mSys->Get_bodylist()->at(i);
		const Vector pos = abody->GetPos();
		const Vector vel = abody->GetPos_dt();
		Quaternion rot = abody->GetRot();
		Vector pos_final, rad_final;
		ShapeType type = SPHERE;

		for (int j = 0; j < abody->GetAssets().size(); j++) {
			ChSharedPtr<ChAsset> asset = abody->GetAssets().at(j);
			if (asset.IsType<ChSphereShape>()) {
				ChSphereShape * sphere_shape = ((ChSphereShape *) (asset.get_ptr()));
				float radius = sphere_shape->GetSphereGeometry().rad;
				Vector center = sphere_shape->GetSphereGeometry().center;
				center = rot.Rotate(center);
				pos_final = pos + center;
				rad_final.x = radius;
				rad_final.y = radius;
				rad_final.z = radius;
				type = SPHERE;
			}

			else if (asset.IsType<ChEllipsoidShape>()) {

				ChEllipsoidShape * ellipsoid_shape = ((ChEllipsoidShape *) (asset.get_ptr()));
				rad_final = ellipsoid_shape->GetEllipsoidGeometry().rad;
				Vector center = ellipsoid_shape->GetEllipsoidGeometry().center;
				center = rot.Rotate(center);

				pos_final = pos + center;

				type = ELLIPSOID;
			} else if (asset.IsType<ChBoxShape>()) {
				ChBoxShape * box_shape = ((ChBoxShape *) (asset.get_ptr()));
				rad_final = box_shape->GetBoxGeometry().Size;
				pos_final = pos;
				type = BOX;
			} else if (asset.IsType<ChCylinderShape>()) {
				ChCylinderShape * cylinder_shape = ((ChCylinderShape *) (asset.get_ptr()));
				double rad = cylinder_shape->GetCylinderGeometry().rad;
				rad_final.x = rad;
				rad_final.y = cylinder_shape->GetCylinderGeometry().p2.y - cylinder_shape->GetCylinderGeometry().p1.y;
				rad_final.z = rad;
				pos_final = pos;
				type = CYLINDER;
			}else if (asset.IsType<ChConeShape>()) {
				ChConeShape * cylinder_shape = ((ChConeShape *) (asset.get_ptr()));
				Vector center = cylinder_shape->GetConeGeometry().center;
				center = rot.Rotate(center);
				rad_final.x = cylinder_shape->GetConeGeometry().rad.x;
				rad_final.y = cylinder_shape->GetConeGeometry().rad.y;
				rad_final.z = cylinder_shape->GetConeGeometry().rad.z;
				pos_final = pos+center;
				type = CONE;
			}

			csv_output << R3(pos_final.x, pos_final.y, pos_final.z);
			csv_output << R4(rot.e0, rot.e1, rot.e2, rot.e3);
			csv_output << R3(vel.x,vel.y,vel.z);


			if (asset.IsType<ChSphereShape>()) {
				csv_output << type;
				csv_output << rad_final.x;
				csv_output.Endline();
			} else if (asset.IsType<ChEllipsoidShape>()) {
				csv_output << type;
				csv_output << R3(rad_final.x, rad_final.y, rad_final.z);
				csv_output.Endline();
			} else if (asset.IsType<ChBoxShape>()) {
				csv_output << type;
				csv_output << R3(rad_final.x, rad_final.y, rad_final.z);
				csv_output.Endline();
			} else if (asset.IsType<ChCylinderShape>()) {
				csv_output << type;
				csv_output << R2(rad_final.x, rad_final.y);
				csv_output.Endline();
			} else if (asset.IsType<ChConeShape>()) {
				csv_output << type;
				csv_output << R2(rad_final.x, rad_final.y);
				csv_output.Endline();
			} else {
				csv_output << type;
				csv_output << 1;
				csv_output.Endline();
			}
		}
	}
	csv_output.CloseFile();
}

void DumpAllObjectsWithGeometry(ChSystemParallel* mSys, string filename, string delim = ",") {
	ofstream ofile(filename.c_str());

	for (int i = 0; i < mSys->Get_bodylist()->size(); i++) {
		ChBody* abody = mSys->Get_bodylist()->at(i);
		const Vector pos = abody->GetPos();
		Quaternion rot = abody->GetRot();
		Vector pos_final, rad_final;
		string type = "sphere";
		string group = "g0";

		for (int j = 0; j < abody->GetAssets().size(); j++) {
			ChSharedPtr<ChAsset> asset = abody->GetAssets().at(j);
			if (asset.IsType<ChSphereShape>()) {
				ChSphereShape * sphere_shape = ((ChSphereShape *) (asset.get_ptr()));
				float radius = sphere_shape->GetSphereGeometry().rad;
				Vector center = sphere_shape->GetSphereGeometry().center;
				center = rot.Rotate(center);
				pos_final = pos + center;
				rad_final.x = radius;
				rad_final.y = radius;
				rad_final.z = radius;
				type = "sphere";
				group = "g1";
			}

			else if (asset.IsType<ChEllipsoidShape>()) {

				ChEllipsoidShape * ellipsoid_shape = ((ChEllipsoidShape *) (asset.get_ptr()));
				rad_final = ellipsoid_shape->GetEllipsoidGeometry().rad;
				Vector center = ellipsoid_shape->GetEllipsoidGeometry().center;
				center = rot.Rotate(center);

				pos_final = pos + center;

				type = "ellipsoid";
				group = "g2";
			} else if (asset.IsType<ChBoxShape>()) {
				ChBoxShape * box_shape = ((ChBoxShape *) (asset.get_ptr()));
				rad_final = box_shape->GetBoxGeometry().Size;
				pos_final = pos;
				type = "box";
				group = "g3";
			} else if (asset.IsType<ChCylinderShape>()) {
				ChCylinderShape * cylinder_shape = ((ChCylinderShape *) (asset.get_ptr()));
				double rad = cylinder_shape->GetCylinderGeometry().rad;
				rad_final.x = rad;
				rad_final.y = cylinder_shape->GetCylinderGeometry().p2.y - cylinder_shape->GetCylinderGeometry().p1.y;

				rad_final.z = rad;
				pos_final = pos;
				type = "cylinder";
				group = "g4";
			}

			ofile << group << delim << i << delim << pos_final.x << delim << pos_final.y << delim << pos_final.z << delim;
			//ofile << type << delim << pos_final.x << delim << pos_final.y << delim << pos_final.z << delim;
			ofile << rot.e0 << delim << rot.e1 << delim << rot.e2 << delim << rot.e3 << delim;
			//ofile <<type<< delim<< rad_final.x << delim << rad_final.y << delim << rad_final.z<<delim<<endl;

			if (asset.IsType<ChSphereShape>()) {
				ofile << type << delim << rad_final.x << delim << endl;
			} else if (asset.IsType<ChEllipsoidShape>()) {
				ofile << type << delim << rad_final.x << delim << rad_final.y << delim << rad_final.z << delim << endl;
			} else if (asset.IsType<ChBoxShape>()) {
				ofile << type << delim << rad_final.x << delim << rad_final.y << delim << rad_final.z << delim << endl;
			} else if (asset.IsType<ChCylinderShape>()) {
				ofile << type << delim << rad_final.x << delim << rad_final.y << delim << endl;
			} else if (asset.IsType<ChConeShape>()) {
				ofile << type << delim << rad_final.x << delim << rad_final.y << delim << endl;
			}
		}
	}
}

template<class T>
void TimingFile(T* mSys, string filename, real current_time) {
	ofstream ofile(filename.c_str(), std::ofstream::out | std::ofstream::app);

	ofile << " Residual: " << ((ChLcpSolverParallel *) (mSys->GetLcpSolverSpeed()))->GetResidual();
	ofile << " ITER: " << ((ChLcpSolverParallel *) (mSys->GetLcpSolverSpeed()))->GetTotalIterations();
	ofile << " OUTPUT STEP: Time= " << current_time << " bodies= " << mSys->GetNbodies() << " contacts= " << mSys->GetNcontacts() << " step time=" << mSys->GetTimerStep() << " lcp time="
			<< mSys->GetTimerLcp() << " CDbroad time=" << mSys->GetTimerCollisionBroad() << " CDnarrow time=" << mSys->GetTimerCollisionNarrow() << " Iterations="
			<< ((ChLcpSolverParallel*) (mSys->GetLcpSolverSpeed()))->GetTotalIterations() << "\n";
	ofile.close();
}
