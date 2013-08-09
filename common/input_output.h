#include "common.h"

template<class T>
void DumpObjects(T* mSys, string filename, string delim = ",", bool dump_vel_rot = false) {
	ofstream ofile(filename.c_str());

	for (int i = 0; i < mSys->Get_bodylist()->size(); i++) {
		ChBody* abody =  mSys->Get_bodylist()->at(i);
		if (abody->IsActive() == true) {
			ofile << abody->GetPos().x << delim << abody->GetPos().y << delim << abody->GetPos().z << delim;
			ofile << abody->GetRot().e0 << delim << abody->GetRot().e1 << delim << abody->GetRot().e2 << delim << abody->GetRot().e3;
			if (!dump_vel_rot) {
				ofile << delim << endl;
			}
			if (dump_vel_rot) {
				ofile << delim << abody->GetPos_dt().x << delim << abody->GetPos_dt().y << delim << abody->GetPos_dt().z << delim;
				ofile << abody->GetWvel_loc().x << delim << abody->GetWvel_loc().y << delim << abody->GetWvel_loc().z << delim << endl;
			}
		}
	}
}
template<class T>
void DumpAllObjects(T* mSys, string filename, string delim = ",", bool dump_vel_rot = false) {
	ofstream ofile(filename.c_str());

	for (int i = 0; i < mSys->Get_bodylist()->size(); i++) {
		ChBody* abody =  mSys->Get_bodylist()->at(i);
		ofile << abody->GetPos().x << delim << abody->GetPos().y << delim << abody->GetPos().z << delim;
		ofile << abody->GetRot().e0 << delim << abody->GetRot().e1 << delim << abody->GetRot().e2 << delim << abody->GetRot().e3;
		if (!dump_vel_rot) {
			ofile << delim << endl;
		}
		if (dump_vel_rot) {
			ofile << delim << abody->GetPos_dt().x << delim << abody->GetPos_dt().y << delim << abody->GetPos_dt().z << delim;
			ofile << abody->GetWvel_loc().x << delim << abody->GetWvel_loc().y << delim << abody->GetWvel_loc().z << delim << endl;
		}
	}
}

void DumpAllObjectsWithGeometry(ChSystemGPU* mSys, string filename, string delim = ",") {
	ofstream ofile(filename.c_str());

	for (int i = 0; i < mSys->Get_bodylist()->size(); i++) {
		ChBody* abody =  mSys->Get_bodylist()->at(i);
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
				ofile <<type<< delim<< rad_final.x << delim<<endl;
			}else if (asset.IsType<ChEllipsoidShape>()) {
				ofile <<type<< delim<< rad_final.x << delim << rad_final.y << delim << rad_final.z<<delim<<endl;
			} else if (asset.IsType<ChBoxShape>()) {
				ofile <<type<< delim<< rad_final.x << delim << rad_final.y << delim << rad_final.z<<delim<<endl;
			} else if (asset.IsType<ChCylinderShape>()) {
				ofile <<type<< delim<< rad_final.x << delim << rad_final.y << delim <<endl;
			}
		}
	}
}

template<class T>
void TimingFile(T* mSys, string filename, real current_time) {
	ofstream ofile(filename.c_str(), std::ofstream::out | std::ofstream::app);

	ofile << " Residual: " << ((ChLcpSolverGPU *) (mSys->GetLcpSolverSpeed()))->GetResidual();
	ofile << " ITER: " << ((ChLcpSolverGPU *) (mSys->GetLcpSolverSpeed()))->GetTotalIterations();
	ofile << " OUTPUT STEP: Time= " << current_time << " bodies= " << mSys->GetNbodies() << " contacts= " << mSys->GetNcontacts() << " step time=" << mSys->GetTimerStep() << " lcp time="
			<< mSys->GetTimerLcp() << " CDbroad time=" << mSys->GetTimerCollisionBroad() << " CDnarrow time=" << mSys->GetTimerCollisionNarrow() << " Iterations="
			<< ((ChLcpSolverGPU*) (mSys->GetLcpSolverSpeed()))->GetTotalIterations() << "\n";
	ofile.close();
}
