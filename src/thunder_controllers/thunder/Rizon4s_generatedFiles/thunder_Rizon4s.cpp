#include "thunder_Rizon4s.h"
#include "Rizon4s_gen.h"

#ifdef ENABLE_PYBIND11
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

// --------------------------- //
// ----- Python bindings ----- //
// --------------------------- //
namespace py = pybind11;
PYBIND11_MODULE(thunder_Rizon4s, m) {
	py::class_<thunder_Rizon4s>(m, "thunder_Rizon4s")
		.def(py::init<>())

		.def_readonly("Dl_order", &thunder_Rizon4s::Dl_order, "Order of the link friction model")
		.def_readonly("STD_PAR_LINK", &thunder_Rizon4s::STD_PAR_LINK, "Standard number of dynamic parameters per link")
		.def_readonly("jointsAvailable", &thunder_Rizon4s::jointsAvailable, "Joints that are available in the generated library")
		.def_readonly("jointsAxis", &thunder_Rizon4s::jointsAxis, "Axes of joints")
		.def_readonly("jointsDerivatives", &thunder_Rizon4s::jointsDerivatives, "Create jacobian derivatives for these joints")
		.def_readonly("jointsDimension", &thunder_Rizon4s::jointsDimension, "Degrees of freedom of each joint")
		.def_readonly("jointsName", &thunder_Rizon4s::jointsName, "Name of joints")
		.def_readonly("jointsParent", &thunder_Rizon4s::jointsParent, "Parent Id of joints")
		.def_readonly("jointsType", &thunder_Rizon4s::jointsType, "Type of joints")
		.def_readonly("ndof", &thunder_Rizon4s::ndof, "Number of degrees of freedom")
		.def_readonly("numJoints", &thunder_Rizon4s::numJoints, "Number of joints")

		.def("get_Ln2EE_xyz", &thunder_Rizon4s::get_Ln2EE_xyz, "Get parameter: Ln2EE_xyz")
		.def("get_Ln2EE_ypr", &thunder_Rizon4s::get_Ln2EE_ypr, "Get parameter: Ln2EE_ypr")
		.def("get_d3q", &thunder_Rizon4s::get_d3q, "Get parameter: d3q")
		.def("get_d4q", &thunder_Rizon4s::get_d4q, "Get parameter: d4q")
		.def("get_ddq", &thunder_Rizon4s::get_ddq, "Get parameter: ddq")
		.def("get_ddqr", &thunder_Rizon4s::get_ddqr, "Get parameter: ddqr")
		.def("get_dq", &thunder_Rizon4s::get_dq, "Get parameter: dq")
		.def("get_dqr", &thunder_Rizon4s::get_dqr, "Get parameter: dqr")
		.def("get_par_DYN", &thunder_Rizon4s::get_par_DYN, "Get parameter: par_DYN")
		.def("get_par_REG", &thunder_Rizon4s::get_par_REG, "Get parameter: par_REG")
		.def("get_q", &thunder_Rizon4s::get_q, "Get parameter: q")
		.def("get_w", &thunder_Rizon4s::get_w, "Get parameter: w")
		.def("get_world2L0_xyz", &thunder_Rizon4s::get_world2L0_xyz, "Get parameter: world2L0_xyz")
		.def("get_world2L0_ypr", &thunder_Rizon4s::get_world2L0_ypr, "Get parameter: world2L0_ypr")
		.def("set_Ln2EE_xyz", &thunder_Rizon4s::set_Ln2EE_xyz, "Set parameter: Ln2EE_xyz", py::arg("value"))
		.def("set_Ln2EE_ypr", &thunder_Rizon4s::set_Ln2EE_ypr, "Set parameter: Ln2EE_ypr", py::arg("value"))
		.def("set_d3q", &thunder_Rizon4s::set_d3q, "Set parameter: d3q", py::arg("value"))
		.def("set_d4q", &thunder_Rizon4s::set_d4q, "Set parameter: d4q", py::arg("value"))
		.def("set_ddq", &thunder_Rizon4s::set_ddq, "Set parameter: ddq", py::arg("value"))
		.def("set_ddqr", &thunder_Rizon4s::set_ddqr, "Set parameter: ddqr", py::arg("value"))
		.def("set_dq", &thunder_Rizon4s::set_dq, "Set parameter: dq", py::arg("value"))
		.def("set_dqr", &thunder_Rizon4s::set_dqr, "Set parameter: dqr", py::arg("value"))
		.def("set_par_DYN", &thunder_Rizon4s::set_par_DYN, "Set parameter: par_DYN", py::arg("value"))
		.def("set_par_REG", &thunder_Rizon4s::set_par_REG, "Set parameter: par_REG", py::arg("value"))
		.def("set_q", &thunder_Rizon4s::set_q, "Set parameter: q", py::arg("value"))
		.def("set_w", &thunder_Rizon4s::set_w, "Set parameter: w", py::arg("value"))
		.def("set_world2L0_xyz", &thunder_Rizon4s::set_world2L0_xyz, "Set parameter: world2L0_xyz", py::arg("value"))
		.def("set_world2L0_ypr", &thunder_Rizon4s::set_world2L0_ypr, "Set parameter: world2L0_ypr", py::arg("value"))

		.def("get_C", &thunder_Rizon4s::get_C, "Manipulator Coriolis matrix")
		.def("get_C_ddot", &thunder_Rizon4s::get_C_ddot, "Second time derivative of the Coriolis matrix")
		.def("get_C_dot", &thunder_Rizon4s::get_C_dot, "Time derivative of the Coriolis matrix")
		.def("get_C_std", &thunder_Rizon4s::get_C_std, "Classic formulation of the manipulator Coriolis matrix")
		.def("get_G", &thunder_Rizon4s::get_G, "Manipulator gravity terms")
		.def("get_G_ddot", &thunder_Rizon4s::get_G_ddot, "Second time derivative of the gravity vector")
		.def("get_G_dot", &thunder_Rizon4s::get_G_dot, "Time derivative of the gravity vector")
		.def("get_J_0", &thunder_Rizon4s::get_J_0, "Jacobian of frame 0")
		.def("get_J_1", &thunder_Rizon4s::get_J_1, "Jacobian of frame 1")
		.def("get_J_2", &thunder_Rizon4s::get_J_2, "Jacobian of frame 2")
		.def("get_J_3", &thunder_Rizon4s::get_J_3, "Jacobian of frame 3")
		.def("get_J_4", &thunder_Rizon4s::get_J_4, "Jacobian of frame 4")
		.def("get_J_5", &thunder_Rizon4s::get_J_5, "Jacobian of frame 5")
		.def("get_J_6", &thunder_Rizon4s::get_J_6, "Jacobian of frame 6")
		.def("get_J_7", &thunder_Rizon4s::get_J_7, "Jacobian of frame 7")
		.def("get_J_8", &thunder_Rizon4s::get_J_8, "Jacobian of frame 8")
		.def("get_J_cm_0", &thunder_Rizon4s::get_J_cm_0, "Jacobian of center of mass of link 0")
		.def("get_J_cm_1", &thunder_Rizon4s::get_J_cm_1, "Jacobian of center of mass of link 1")
		.def("get_J_cm_2", &thunder_Rizon4s::get_J_cm_2, "Jacobian of center of mass of link 2")
		.def("get_J_cm_3", &thunder_Rizon4s::get_J_cm_3, "Jacobian of center of mass of link 3")
		.def("get_J_cm_4", &thunder_Rizon4s::get_J_cm_4, "Jacobian of center of mass of link 4")
		.def("get_J_cm_5", &thunder_Rizon4s::get_J_cm_5, "Jacobian of center of mass of link 5")
		.def("get_J_cm_6", &thunder_Rizon4s::get_J_cm_6, "Jacobian of center of mass of link 6")
		.def("get_J_cm_7", &thunder_Rizon4s::get_J_cm_7, "Jacobian of center of mass of link 7")
		.def("get_J_cm_8", &thunder_Rizon4s::get_J_cm_8, "Jacobian of center of mass of link 8")
		.def("get_J_flange", &thunder_Rizon4s::get_J_flange, "Jacobian of frame flange")
		.def("get_J_flange_ddot", &thunder_Rizon4s::get_J_flange_ddot, "Time second derivative of jacobian matrix of frame flange")
		.def("get_J_flange_dot", &thunder_Rizon4s::get_J_flange_dot, "Time derivative of jacobian matrix of frame flange")
		.def("get_J_flange_pinv", &thunder_Rizon4s::get_J_flange_pinv, "Pseudo-Inverse of jacobian matrix of frame flange")
		.def("get_M", &thunder_Rizon4s::get_M, "Manipulator mass matrix")
		.def("get_M_ddot", &thunder_Rizon4s::get_M_ddot, "Second time derivative of the mass matrix")
		.def("get_M_dot", &thunder_Rizon4s::get_M_dot, "Time derivative of the mass matrix")
		.def("get_T_0", &thunder_Rizon4s::get_T_0, "relative transformation from frame-1to frame 0")
		.def("get_T_1", &thunder_Rizon4s::get_T_1, "relative transformation from frame0to frame 1")
		.def("get_T_2", &thunder_Rizon4s::get_T_2, "relative transformation from frame1to frame 2")
		.def("get_T_3", &thunder_Rizon4s::get_T_3, "relative transformation from frame2to frame 3")
		.def("get_T_4", &thunder_Rizon4s::get_T_4, "relative transformation from frame3to frame 4")
		.def("get_T_5", &thunder_Rizon4s::get_T_5, "relative transformation from frame4to frame 5")
		.def("get_T_6", &thunder_Rizon4s::get_T_6, "relative transformation from frame5to frame 6")
		.def("get_T_7", &thunder_Rizon4s::get_T_7, "relative transformation from frame6to frame 7")
		.def("get_T_8", &thunder_Rizon4s::get_T_8, "relative transformation from frame7to frame 8")
		.def("get_T_JOINT_FIXED", &thunder_Rizon4s::get_T_JOINT_FIXED, "Template transformation of a fixed joint")
		.def("get_T_JOINT_P", &thunder_Rizon4s::get_T_JOINT_P, "Template transformation of general prismatic joint R")
		.def("get_T_JOINT_R", &thunder_Rizon4s::get_T_JOINT_R, "Template transformation of general rotoidal joint R")
		.def("get_T_w_0", &thunder_Rizon4s::get_T_w_0, "absolute transformation from frame world to frame 0")
		.def("get_T_w_1", &thunder_Rizon4s::get_T_w_1, "absolute transformation from frame world to frame 1")
		.def("get_T_w_2", &thunder_Rizon4s::get_T_w_2, "absolute transformation from frame world to frame 2")
		.def("get_T_w_3", &thunder_Rizon4s::get_T_w_3, "absolute transformation from frame world to frame 3")
		.def("get_T_w_4", &thunder_Rizon4s::get_T_w_4, "absolute transformation from frame world to frame 4")
		.def("get_T_w_5", &thunder_Rizon4s::get_T_w_5, "absolute transformation from frame world to frame 5")
		.def("get_T_w_6", &thunder_Rizon4s::get_T_w_6, "absolute transformation from frame world to frame 6")
		.def("get_T_w_7", &thunder_Rizon4s::get_T_w_7, "absolute transformation from frame world to frame 7")
		.def("get_T_w_8", &thunder_Rizon4s::get_T_w_8, "absolute transformation from frame world to frame 8")
		.def("get_T_w_flange", &thunder_Rizon4s::get_T_w_flange, "absolute transformation from frame world to frame flange")
		.def("get_Y", &thunder_Rizon4s::get_Y, "Standard manipulator regressor matrix")
		.def("get_Yr", &thunder_Rizon4s::get_Yr, "Slotine-Li manipulator regressor matrix")
		.def("get_dyn2reg", &thunder_Rizon4s::get_dyn2reg, "Conversion from dynamic to regressor parameters")
		.def("get_par_KIN", &thunder_Rizon4s::get_par_KIN, "Kinematic parameters")
		.def("get_par_Ln2EE", &thunder_Rizon4s::get_par_Ln2EE, "Last link to end-effector frame.")
		.def("get_par_world2L0", &thunder_Rizon4s::get_par_world2L0, "World to base frame.")
		.def("get_reg2dyn", &thunder_Rizon4s::get_reg2dyn, "Conversion from regressor to dynamic parameters")
		.def("get_reg_C", &thunder_Rizon4s::get_reg_C, "Regressor matrix of term C*dqr")
		.def("get_reg_G", &thunder_Rizon4s::get_reg_G, "Regressor matrix of term G")
		.def("get_reg_JTw_flange", &thunder_Rizon4s::get_reg_JTw_flange, "Regressor matrix of the quantity J^T*w of link flange")
		.def("get_reg_Jdq_flange", &thunder_Rizon4s::get_reg_Jdq_flange, "Regressor matrix of the quantity J*dq of link flange")
		.def("get_reg_M", &thunder_Rizon4s::get_reg_M, "Regressor matrix of term M*ddqr");
}
#endif

thunder_Rizon4s::thunder_Rizon4s(){
	Ln2EE_xyz << 0, 0, 0;
	Ln2EE_ypr << 0, 0, 0;
	d3q << 0, 0, 0, 0, 0, 0, 0;
	d4q << 0, 0, 0, 0, 0, 0, 0;
	ddq << 0, 0, 0, 0, 0, 0, 0;
	ddqr << 0, 0, 0, 0, 0, 0, 0;
	dq << 0, 0, 0, 0, 0, 0, 0;
	dqr << 0, 0, 0, 0, 0, 0, 0;
	par_DYN << 3.7879999999999998, 0.00020000000000000001, -0.00069999999999999999, 0.087099999999999997, 0.018200000000000001, 0, 0, 0.0184, 0.0001, 0.01, 3.5960000000000001, 0.0001, 0.0035000000000000001, 0.14999999999999999, 0.027400000000000001, 0, 0, 0.027300000000000001, 0.0011000000000000001, 0.0071000000000000004, 2.6899999999999999, -0.00029999999999999997, 0.0361, 0.0998, 0.023, 0, 0, 0.023900000000000001, -0.0001, 0.0035999999999999999, 2.3540000000000001, -0.0123, -0.0011999999999999999, 0.1366, 0.0143, 0.0001, -0.0015, 0.0137, -0.00050000000000000001, 0.0040000000000000001, 2.3540000000000001, -0.0129, 0.031300000000000001, 0.10340000000000001, 0.017999999999999999, 0.0001, -0.0016000000000000001, 0.0184, -0.0011999999999999999, 0.0030000000000000001, 2.3100000000000001, 0.00029999999999999997, 0.0014, 0.1363, 0.014200000000000001, 0, 0, 0.0135, 0.00050000000000000001, 0.0037000000000000002, 2.2229999999999999, 0.020199999999999999, 0.0596, 0.072400000000000006, 0.0085000000000000006, 0.00040000000000000002, 0.0011999999999999999, 0.0089999999999999993, 0.0018, 0.0041999999999999997, 1.403, 0, 0.0001, 0.054399999999999997, 0.0032000000000000002, 0, 0, 0.0032000000000000002, 0, 0.0018, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
	par_REG << 3.7879999999999998, 0.00075759999999999998, -0.0026516000000000001, 0.32993479999999997, 0.046939177199999994, 5.3032000000000005e-07, -6.5986959999999987e-05, 0.047137472599999994, 0.00033095435999999997, 0.010002007639999999, 3.5960000000000001, 0.00035960000000000001, 0.012586, 0.53939999999999999, 0.10835405100000001, -1.2586000000000001e-06, -5.3940000000000002e-05, 0.10821003596000001, -0.0007878999999999998, 0.0071440869600000005, 2.6899999999999999, -0.00080699999999999988, 0.097109000000000001, 0.26846199999999998, 0.053298142499999993, 2.91327e-05, 8.0538599999999995e-05, 0.050692749699999998, -0.0097914781999999985, 0.007105877, 2.3540000000000001, -0.028954200000000003, -0.0028247999999999997, 0.32155640000000002, 0.058227993999999998, 6.5254960000000009e-05, 0.0024551437200000001, 0.057980740899999994, -0.00011413232000000005, 0.0043595264199999999, 2.3540000000000001, -0.030366600000000001, 0.073680200000000001, 0.24340360000000003, 0.045474122500000005, 0.0010504745800000001, 0.0015399064400000004, 0.043959661380000001, -0.0088185326800000012, 0.0056979194000000011, 2.3100000000000001, 0.00069299999999999993, 0.0032339999999999999, 0.31485299999999999, 0.057118991500000008, -9.7019999999999983e-07, -9.4455899999999998e-05, 0.056414671800000003, 5.9205800000000002e-05, 0.0037047355, 2.2229999999999999, 0.044904599999999996, 0.13249079999999999, 0.16094520000000001, 0.028048884159999999, -0.0022763141599999996, -0.0020510930400000002, 0.021559505399999998, -0.00779233392, 0.0130035246, 1.403, 0, 0.00014030000000000002, 0.076323199999999994, 0.00735199611, 0, 0, 0.0073519820799999996, -7.6323199999999995e-06, 0.0018000140300000001, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
	q << 0, 0, 0, 0, 0, 0, 0;
	w << 0, 0, 0, 0, 0, 0;
	world2L0_xyz << 0, 0, 0;
	world2L0_ypr << 0, 0, 0;
}

// get the parameter: Ln2EE_xyz
Matrix<double,3,1> thunder_Rizon4s::get_Ln2EE_xyz() {return Ln2EE_xyz;}
// get the parameter: Ln2EE_ypr
Matrix<double,3,1> thunder_Rizon4s::get_Ln2EE_ypr() {return Ln2EE_ypr;}
// get the parameter: d3q
Matrix<double,7,1> thunder_Rizon4s::get_d3q() {return d3q;}
// get the parameter: d4q
Matrix<double,7,1> thunder_Rizon4s::get_d4q() {return d4q;}
// get the parameter: ddq
Matrix<double,7,1> thunder_Rizon4s::get_ddq() {return ddq;}
// get the parameter: ddqr
Matrix<double,7,1> thunder_Rizon4s::get_ddqr() {return ddqr;}
// get the parameter: dq
Matrix<double,7,1> thunder_Rizon4s::get_dq() {return dq;}
// get the parameter: dqr
Matrix<double,7,1> thunder_Rizon4s::get_dqr() {return dqr;}
// get the parameter: par_DYN
Matrix<double,90,1> thunder_Rizon4s::get_par_DYN() {return par_DYN;}
// get the parameter: par_REG
Matrix<double,90,1> thunder_Rizon4s::get_par_REG() {return par_REG;}
// get the parameter: q
Matrix<double,7,1> thunder_Rizon4s::get_q() {return q;}
// get the parameter: w
Matrix<double,6,1> thunder_Rizon4s::get_w() {return w;}
// get the parameter: world2L0_xyz
Matrix<double,3,1> thunder_Rizon4s::get_world2L0_xyz() {return world2L0_xyz;}
// get the parameter: world2L0_ypr
Matrix<double,3,1> thunder_Rizon4s::get_world2L0_ypr() {return world2L0_ypr;}
// set the parameter: Ln2EE_xyz
void thunder_Rizon4s::set_Ln2EE_xyz(Matrix<double,3,1> value) {Ln2EE_xyz = value;}
// set the parameter: Ln2EE_ypr
void thunder_Rizon4s::set_Ln2EE_ypr(Matrix<double,3,1> value) {Ln2EE_ypr = value;}
// set the parameter: d3q
void thunder_Rizon4s::set_d3q(Matrix<double,7,1> value) {d3q = value;}
// set the parameter: d4q
void thunder_Rizon4s::set_d4q(Matrix<double,7,1> value) {d4q = value;}
// set the parameter: ddq
void thunder_Rizon4s::set_ddq(Matrix<double,7,1> value) {ddq = value;}
// set the parameter: ddqr
void thunder_Rizon4s::set_ddqr(Matrix<double,7,1> value) {ddqr = value;}
// set the parameter: dq
void thunder_Rizon4s::set_dq(Matrix<double,7,1> value) {dq = value;}
// set the parameter: dqr
void thunder_Rizon4s::set_dqr(Matrix<double,7,1> value) {dqr = value;}
// set the parameter: par_DYN
void thunder_Rizon4s::set_par_DYN(Matrix<double,90,1> value) {par_DYN = value;}
// set the parameter: par_REG
void thunder_Rizon4s::set_par_REG(Matrix<double,90,1> value) {par_REG = value;}
// set the parameter: q
void thunder_Rizon4s::set_q(Matrix<double,7,1> value) {q = value;}
// set the parameter: w
void thunder_Rizon4s::set_w(Matrix<double,6,1> value) {w = value;}
// set the parameter: world2L0_xyz
void thunder_Rizon4s::set_world2L0_xyz(Matrix<double,3,1> value) {world2L0_xyz = value;}
// set the parameter: world2L0_ypr
void thunder_Rizon4s::set_world2L0_ypr(Matrix<double,3,1> value) {world2L0_ypr = value;}

// Save parameters to file
int thunder_Rizon4s::save_par(string par_file, vector<string> par_list){
	YAML::Node yamlFile;

	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "Ln2EE_xyz"))){
		vector<double> Ln2EE_xyz_vect(Ln2EE_xyz.data(), Ln2EE_xyz.data() + 3);
		yamlFile["Ln2EE_xyz"] = Ln2EE_xyz_vect;
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "Ln2EE_ypr"))){
		vector<double> Ln2EE_ypr_vect(Ln2EE_ypr.data(), Ln2EE_ypr.data() + 3);
		yamlFile["Ln2EE_ypr"] = Ln2EE_ypr_vect;
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "d3q"))){
		vector<double> d3q_vect(d3q.data(), d3q.data() + 7);
		yamlFile["d3q"] = d3q_vect;
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "d4q"))){
		vector<double> d4q_vect(d4q.data(), d4q.data() + 7);
		yamlFile["d4q"] = d4q_vect;
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "ddq"))){
		vector<double> ddq_vect(ddq.data(), ddq.data() + 7);
		yamlFile["ddq"] = ddq_vect;
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "ddqr"))){
		vector<double> ddqr_vect(ddqr.data(), ddqr.data() + 7);
		yamlFile["ddqr"] = ddqr_vect;
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "dq"))){
		vector<double> dq_vect(dq.data(), dq.data() + 7);
		yamlFile["dq"] = dq_vect;
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "dqr"))){
		vector<double> dqr_vect(dqr.data(), dqr.data() + 7);
		yamlFile["dqr"] = dqr_vect;
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "par_DYN"))){
		vector<double> par_DYN_vect(par_DYN.data(), par_DYN.data() + 90);
		yamlFile["par_DYN"] = par_DYN_vect;
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "par_REG"))){
		vector<double> par_REG_vect(par_REG.data(), par_REG.data() + 90);
		yamlFile["par_REG"] = par_REG_vect;
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "q"))){
		vector<double> q_vect(q.data(), q.data() + 7);
		yamlFile["q"] = q_vect;
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "w"))){
		vector<double> w_vect(w.data(), w.data() + 6);
		yamlFile["w"] = w_vect;
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "world2L0_xyz"))){
		vector<double> world2L0_xyz_vect(world2L0_xyz.data(), world2L0_xyz.data() + 3);
		yamlFile["world2L0_xyz"] = world2L0_xyz_vect;
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "world2L0_ypr"))){
		vector<double> world2L0_ypr_vect(world2L0_ypr.data(), world2L0_ypr.data() + 3);
		yamlFile["world2L0_ypr"] = world2L0_ypr_vect;
	}

	try {
		YAML::Emitter emitter;
		emitter.SetIndent(2);
		emitter.SetSeqFormat(YAML::Flow);
		emitter << yamlFile << YAML::Newline;
		std::ofstream fout(par_file);
		fout << emitter.c_str();
		fout.close();
	} catch (const YAML::Exception& e) {
		std::cerr << "Error while generating YAML: " << e.what() << std::endl;
		return 0;
	}
	return 1;
}


// Load parameters from file
int thunder_Rizon4s::load_par(string par_file, vector<string> par_list){
	YAML::Node yamlFile;
	try {
		yamlFile = YAML::LoadFile(par_file);

	} catch (const YAML::Exception& e) {
		std::cerr << "Error while loading parameters: " << e.what() << std::endl;
		return 0;
	}

	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "Ln2EE_xyz"))){
		if (yamlFile["Ln2EE_xyz"]){
			vector<double> vec(yamlFile["Ln2EE_xyz"].as<vector<double>>());
			Ln2EE_xyz = Eigen::Map<Matrix<double,3,1>>(vec.data(), vec.size());
		} else {
			std::cerr << "Error while loading parameters: Ln2EE_xyz not found!" << std::endl;
			return 0;
		}
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "Ln2EE_ypr"))){
		if (yamlFile["Ln2EE_ypr"]){
			vector<double> vec(yamlFile["Ln2EE_ypr"].as<vector<double>>());
			Ln2EE_ypr = Eigen::Map<Matrix<double,3,1>>(vec.data(), vec.size());
		} else {
			std::cerr << "Error while loading parameters: Ln2EE_ypr not found!" << std::endl;
			return 0;
		}
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "d3q"))){
		if (yamlFile["d3q"]){
			vector<double> vec(yamlFile["d3q"].as<vector<double>>());
			d3q = Eigen::Map<Matrix<double,7,1>>(vec.data(), vec.size());
		} else {
			std::cerr << "Error while loading parameters: d3q not found!" << std::endl;
			return 0;
		}
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "d4q"))){
		if (yamlFile["d4q"]){
			vector<double> vec(yamlFile["d4q"].as<vector<double>>());
			d4q = Eigen::Map<Matrix<double,7,1>>(vec.data(), vec.size());
		} else {
			std::cerr << "Error while loading parameters: d4q not found!" << std::endl;
			return 0;
		}
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "ddq"))){
		if (yamlFile["ddq"]){
			vector<double> vec(yamlFile["ddq"].as<vector<double>>());
			ddq = Eigen::Map<Matrix<double,7,1>>(vec.data(), vec.size());
		} else {
			std::cerr << "Error while loading parameters: ddq not found!" << std::endl;
			return 0;
		}
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "ddqr"))){
		if (yamlFile["ddqr"]){
			vector<double> vec(yamlFile["ddqr"].as<vector<double>>());
			ddqr = Eigen::Map<Matrix<double,7,1>>(vec.data(), vec.size());
		} else {
			std::cerr << "Error while loading parameters: ddqr not found!" << std::endl;
			return 0;
		}
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "dq"))){
		if (yamlFile["dq"]){
			vector<double> vec(yamlFile["dq"].as<vector<double>>());
			dq = Eigen::Map<Matrix<double,7,1>>(vec.data(), vec.size());
		} else {
			std::cerr << "Error while loading parameters: dq not found!" << std::endl;
			return 0;
		}
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "dqr"))){
		if (yamlFile["dqr"]){
			vector<double> vec(yamlFile["dqr"].as<vector<double>>());
			dqr = Eigen::Map<Matrix<double,7,1>>(vec.data(), vec.size());
		} else {
			std::cerr << "Error while loading parameters: dqr not found!" << std::endl;
			return 0;
		}
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "par_DYN"))){
		if (yamlFile["par_DYN"]){
			vector<double> vec(yamlFile["par_DYN"].as<vector<double>>());
			par_DYN = Eigen::Map<Matrix<double,90,1>>(vec.data(), vec.size());
		} else {
			std::cerr << "Error while loading parameters: par_DYN not found!" << std::endl;
			return 0;
		}
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "par_REG"))){
		if (yamlFile["par_REG"]){
			vector<double> vec(yamlFile["par_REG"].as<vector<double>>());
			par_REG = Eigen::Map<Matrix<double,90,1>>(vec.data(), vec.size());
		} else {
			std::cerr << "Error while loading parameters: par_REG not found!" << std::endl;
			return 0;
		}
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "q"))){
		if (yamlFile["q"]){
			vector<double> vec(yamlFile["q"].as<vector<double>>());
			q = Eigen::Map<Matrix<double,7,1>>(vec.data(), vec.size());
		} else {
			std::cerr << "Error while loading parameters: q not found!" << std::endl;
			return 0;
		}
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "w"))){
		if (yamlFile["w"]){
			vector<double> vec(yamlFile["w"].as<vector<double>>());
			w = Eigen::Map<Matrix<double,6,1>>(vec.data(), vec.size());
		} else {
			std::cerr << "Error while loading parameters: w not found!" << std::endl;
			return 0;
		}
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "world2L0_xyz"))){
		if (yamlFile["world2L0_xyz"]){
			vector<double> vec(yamlFile["world2L0_xyz"].as<vector<double>>());
			world2L0_xyz = Eigen::Map<Matrix<double,3,1>>(vec.data(), vec.size());
		} else {
			std::cerr << "Error while loading parameters: world2L0_xyz not found!" << std::endl;
			return 0;
		}
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "world2L0_ypr"))){
		if (yamlFile["world2L0_ypr"]){
			vector<double> vec(yamlFile["world2L0_ypr"].as<vector<double>>());
			world2L0_ypr = Eigen::Map<Matrix<double,3,1>>(vec.data(), vec.size());
		} else {
			std::cerr << "Error while loading parameters: world2L0_ypr not found!" << std::endl;
			return 0;
		}
	}

	return 1;
}


// Manipulator Coriolis matrix
Eigen::Matrix<double,7,7> thunder_Rizon4s::get_C() {
	thread_local double buffer[49];
	thread_local long long p3[Rizon4s_C_fun_SZ_IW];
	thread_local double p4[Rizon4s_C_fun_SZ_W];
	const double* input_[] = {q.data(), dq.data(), world2L0_xyz.data(), world2L0_ypr.data(), Ln2EE_xyz.data(), Ln2EE_ypr.data(), par_DYN.data()};
	double* output_[] = {buffer};
	int check = Rizon4s_C_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,7,7>>(buffer);
}

// Second time derivative of the Coriolis matrix
Eigen::Matrix<double,7,7> thunder_Rizon4s::get_C_ddot() {
	thread_local double buffer[49];
	thread_local long long p3[Rizon4s_C_ddot_fun_SZ_IW];
	thread_local double p4[Rizon4s_C_ddot_fun_SZ_W];
	const double* input_[] = {q.data(), dq.data(), ddq.data(), d3q.data(), world2L0_xyz.data(), world2L0_ypr.data(), Ln2EE_xyz.data(), Ln2EE_ypr.data(), par_DYN.data()};
	double* output_[] = {buffer};
	int check = Rizon4s_C_ddot_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,7,7>>(buffer);
}

// Time derivative of the Coriolis matrix
Eigen::Matrix<double,7,7> thunder_Rizon4s::get_C_dot() {
	thread_local double buffer[49];
	thread_local long long p3[Rizon4s_C_dot_fun_SZ_IW];
	thread_local double p4[Rizon4s_C_dot_fun_SZ_W];
	const double* input_[] = {q.data(), dq.data(), ddq.data(), world2L0_xyz.data(), world2L0_ypr.data(), Ln2EE_xyz.data(), Ln2EE_ypr.data(), par_DYN.data()};
	double* output_[] = {buffer};
	int check = Rizon4s_C_dot_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,7,7>>(buffer);
}

// Classic formulation of the manipulator Coriolis matrix
Eigen::Matrix<double,7,7> thunder_Rizon4s::get_C_std() {
	thread_local double buffer[49];
	thread_local long long p3[Rizon4s_C_std_fun_SZ_IW];
	thread_local double p4[Rizon4s_C_std_fun_SZ_W];
	const double* input_[] = {q.data(), dq.data(), world2L0_xyz.data(), world2L0_ypr.data(), Ln2EE_xyz.data(), Ln2EE_ypr.data(), par_DYN.data()};
	double* output_[] = {buffer};
	int check = Rizon4s_C_std_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,7,7>>(buffer);
}

// Manipulator gravity terms
Eigen::Matrix<double,7,1> thunder_Rizon4s::get_G() {
	thread_local double buffer[7];
	thread_local long long p3[Rizon4s_G_fun_SZ_IW];
	thread_local double p4[Rizon4s_G_fun_SZ_W];
	const double* input_[] = {q.data(), world2L0_xyz.data(), world2L0_ypr.data(), Ln2EE_xyz.data(), Ln2EE_ypr.data(), par_DYN.data()};
	double* output_[] = {buffer};
	int check = Rizon4s_G_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,7,1>>(buffer);
}

// Second time derivative of the gravity vector
Eigen::Matrix<double,7,1> thunder_Rizon4s::get_G_ddot() {
	thread_local double buffer[7];
	thread_local long long p3[Rizon4s_G_ddot_fun_SZ_IW];
	thread_local double p4[Rizon4s_G_ddot_fun_SZ_W];
	const double* input_[] = {q.data(), dq.data(), ddq.data(), world2L0_xyz.data(), world2L0_ypr.data(), Ln2EE_xyz.data(), Ln2EE_ypr.data(), par_DYN.data()};
	double* output_[] = {buffer};
	int check = Rizon4s_G_ddot_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,7,1>>(buffer);
}

// Time derivative of the gravity vector
Eigen::Matrix<double,7,1> thunder_Rizon4s::get_G_dot() {
	thread_local double buffer[7];
	thread_local long long p3[Rizon4s_G_dot_fun_SZ_IW];
	thread_local double p4[Rizon4s_G_dot_fun_SZ_W];
	const double* input_[] = {q.data(), dq.data(), world2L0_xyz.data(), world2L0_ypr.data(), Ln2EE_xyz.data(), Ln2EE_ypr.data(), par_DYN.data()};
	double* output_[] = {buffer};
	int check = Rizon4s_G_dot_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,7,1>>(buffer);
}

// Jacobian of frame 0
Eigen::Matrix<double,6,7> thunder_Rizon4s::get_J_0() {
	thread_local double buffer[42];
	thread_local long long p3[Rizon4s_J_0_fun_SZ_IW];
	thread_local double p4[Rizon4s_J_0_fun_SZ_W];
	const double* input_[] = {q.data(), world2L0_xyz.data(), world2L0_ypr.data(), Ln2EE_xyz.data(), Ln2EE_ypr.data()};
	double* output_[] = {buffer};
	int check = Rizon4s_J_0_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,7>>(buffer);
}

// Jacobian of frame 1
Eigen::Matrix<double,6,7> thunder_Rizon4s::get_J_1() {
	thread_local double buffer[42];
	thread_local long long p3[Rizon4s_J_1_fun_SZ_IW];
	thread_local double p4[Rizon4s_J_1_fun_SZ_W];
	const double* input_[] = {q.data(), world2L0_xyz.data(), world2L0_ypr.data(), Ln2EE_xyz.data(), Ln2EE_ypr.data()};
	double* output_[] = {buffer};
	int check = Rizon4s_J_1_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,7>>(buffer);
}

// Jacobian of frame 2
Eigen::Matrix<double,6,7> thunder_Rizon4s::get_J_2() {
	thread_local double buffer[42];
	thread_local long long p3[Rizon4s_J_2_fun_SZ_IW];
	thread_local double p4[Rizon4s_J_2_fun_SZ_W];
	const double* input_[] = {q.data(), world2L0_xyz.data(), world2L0_ypr.data(), Ln2EE_xyz.data(), Ln2EE_ypr.data()};
	double* output_[] = {buffer};
	int check = Rizon4s_J_2_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,7>>(buffer);
}

// Jacobian of frame 3
Eigen::Matrix<double,6,7> thunder_Rizon4s::get_J_3() {
	thread_local double buffer[42];
	thread_local long long p3[Rizon4s_J_3_fun_SZ_IW];
	thread_local double p4[Rizon4s_J_3_fun_SZ_W];
	const double* input_[] = {q.data(), world2L0_xyz.data(), world2L0_ypr.data(), Ln2EE_xyz.data(), Ln2EE_ypr.data()};
	double* output_[] = {buffer};
	int check = Rizon4s_J_3_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,7>>(buffer);
}

// Jacobian of frame 4
Eigen::Matrix<double,6,7> thunder_Rizon4s::get_J_4() {
	thread_local double buffer[42];
	thread_local long long p3[Rizon4s_J_4_fun_SZ_IW];
	thread_local double p4[Rizon4s_J_4_fun_SZ_W];
	const double* input_[] = {q.data(), world2L0_xyz.data(), world2L0_ypr.data(), Ln2EE_xyz.data(), Ln2EE_ypr.data()};
	double* output_[] = {buffer};
	int check = Rizon4s_J_4_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,7>>(buffer);
}

// Jacobian of frame 5
Eigen::Matrix<double,6,7> thunder_Rizon4s::get_J_5() {
	thread_local double buffer[42];
	thread_local long long p3[Rizon4s_J_5_fun_SZ_IW];
	thread_local double p4[Rizon4s_J_5_fun_SZ_W];
	const double* input_[] = {q.data(), world2L0_xyz.data(), world2L0_ypr.data(), Ln2EE_xyz.data(), Ln2EE_ypr.data()};
	double* output_[] = {buffer};
	int check = Rizon4s_J_5_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,7>>(buffer);
}

// Jacobian of frame 6
Eigen::Matrix<double,6,7> thunder_Rizon4s::get_J_6() {
	thread_local double buffer[42];
	thread_local long long p3[Rizon4s_J_6_fun_SZ_IW];
	thread_local double p4[Rizon4s_J_6_fun_SZ_W];
	const double* input_[] = {q.data(), world2L0_xyz.data(), world2L0_ypr.data(), Ln2EE_xyz.data(), Ln2EE_ypr.data()};
	double* output_[] = {buffer};
	int check = Rizon4s_J_6_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,7>>(buffer);
}

// Jacobian of frame 7
Eigen::Matrix<double,6,7> thunder_Rizon4s::get_J_7() {
	thread_local double buffer[42];
	thread_local long long p3[Rizon4s_J_7_fun_SZ_IW];
	thread_local double p4[Rizon4s_J_7_fun_SZ_W];
	const double* input_[] = {q.data(), world2L0_xyz.data(), world2L0_ypr.data(), Ln2EE_xyz.data(), Ln2EE_ypr.data()};
	double* output_[] = {buffer};
	int check = Rizon4s_J_7_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,7>>(buffer);
}

// Jacobian of frame 8
Eigen::Matrix<double,6,7> thunder_Rizon4s::get_J_8() {
	thread_local double buffer[42];
	thread_local long long p3[Rizon4s_J_8_fun_SZ_IW];
	thread_local double p4[Rizon4s_J_8_fun_SZ_W];
	const double* input_[] = {q.data(), world2L0_xyz.data(), world2L0_ypr.data(), Ln2EE_xyz.data(), Ln2EE_ypr.data()};
	double* output_[] = {buffer};
	int check = Rizon4s_J_8_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,7>>(buffer);
}

// Jacobian of center of mass of link 0
Eigen::Matrix<double,6,7> thunder_Rizon4s::get_J_cm_0() {
	thread_local double buffer[42];
	thread_local long long p3[Rizon4s_J_cm_0_fun_SZ_IW];
	thread_local double p4[Rizon4s_J_cm_0_fun_SZ_W];
	const double* input_[] = {q.data(), world2L0_xyz.data(), world2L0_ypr.data(), Ln2EE_xyz.data(), Ln2EE_ypr.data(), par_DYN.data()};
	double* output_[] = {buffer};
	int check = Rizon4s_J_cm_0_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,7>>(buffer);
}

// Jacobian of center of mass of link 1
Eigen::Matrix<double,6,7> thunder_Rizon4s::get_J_cm_1() {
	thread_local double buffer[42];
	thread_local long long p3[Rizon4s_J_cm_1_fun_SZ_IW];
	thread_local double p4[Rizon4s_J_cm_1_fun_SZ_W];
	const double* input_[] = {q.data(), world2L0_xyz.data(), world2L0_ypr.data(), Ln2EE_xyz.data(), Ln2EE_ypr.data(), par_DYN.data()};
	double* output_[] = {buffer};
	int check = Rizon4s_J_cm_1_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,7>>(buffer);
}

// Jacobian of center of mass of link 2
Eigen::Matrix<double,6,7> thunder_Rizon4s::get_J_cm_2() {
	thread_local double buffer[42];
	thread_local long long p3[Rizon4s_J_cm_2_fun_SZ_IW];
	thread_local double p4[Rizon4s_J_cm_2_fun_SZ_W];
	const double* input_[] = {q.data(), world2L0_xyz.data(), world2L0_ypr.data(), Ln2EE_xyz.data(), Ln2EE_ypr.data(), par_DYN.data()};
	double* output_[] = {buffer};
	int check = Rizon4s_J_cm_2_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,7>>(buffer);
}

// Jacobian of center of mass of link 3
Eigen::Matrix<double,6,7> thunder_Rizon4s::get_J_cm_3() {
	thread_local double buffer[42];
	thread_local long long p3[Rizon4s_J_cm_3_fun_SZ_IW];
	thread_local double p4[Rizon4s_J_cm_3_fun_SZ_W];
	const double* input_[] = {q.data(), world2L0_xyz.data(), world2L0_ypr.data(), Ln2EE_xyz.data(), Ln2EE_ypr.data(), par_DYN.data()};
	double* output_[] = {buffer};
	int check = Rizon4s_J_cm_3_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,7>>(buffer);
}

// Jacobian of center of mass of link 4
Eigen::Matrix<double,6,7> thunder_Rizon4s::get_J_cm_4() {
	thread_local double buffer[42];
	thread_local long long p3[Rizon4s_J_cm_4_fun_SZ_IW];
	thread_local double p4[Rizon4s_J_cm_4_fun_SZ_W];
	const double* input_[] = {q.data(), world2L0_xyz.data(), world2L0_ypr.data(), Ln2EE_xyz.data(), Ln2EE_ypr.data(), par_DYN.data()};
	double* output_[] = {buffer};
	int check = Rizon4s_J_cm_4_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,7>>(buffer);
}

// Jacobian of center of mass of link 5
Eigen::Matrix<double,6,7> thunder_Rizon4s::get_J_cm_5() {
	thread_local double buffer[42];
	thread_local long long p3[Rizon4s_J_cm_5_fun_SZ_IW];
	thread_local double p4[Rizon4s_J_cm_5_fun_SZ_W];
	const double* input_[] = {q.data(), world2L0_xyz.data(), world2L0_ypr.data(), Ln2EE_xyz.data(), Ln2EE_ypr.data(), par_DYN.data()};
	double* output_[] = {buffer};
	int check = Rizon4s_J_cm_5_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,7>>(buffer);
}

// Jacobian of center of mass of link 6
Eigen::Matrix<double,6,7> thunder_Rizon4s::get_J_cm_6() {
	thread_local double buffer[42];
	thread_local long long p3[Rizon4s_J_cm_6_fun_SZ_IW];
	thread_local double p4[Rizon4s_J_cm_6_fun_SZ_W];
	const double* input_[] = {q.data(), world2L0_xyz.data(), world2L0_ypr.data(), Ln2EE_xyz.data(), Ln2EE_ypr.data(), par_DYN.data()};
	double* output_[] = {buffer};
	int check = Rizon4s_J_cm_6_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,7>>(buffer);
}

// Jacobian of center of mass of link 7
Eigen::Matrix<double,6,7> thunder_Rizon4s::get_J_cm_7() {
	thread_local double buffer[42];
	thread_local long long p3[Rizon4s_J_cm_7_fun_SZ_IW];
	thread_local double p4[Rizon4s_J_cm_7_fun_SZ_W];
	const double* input_[] = {q.data(), world2L0_xyz.data(), world2L0_ypr.data(), Ln2EE_xyz.data(), Ln2EE_ypr.data(), par_DYN.data()};
	double* output_[] = {buffer};
	int check = Rizon4s_J_cm_7_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,7>>(buffer);
}

// Jacobian of center of mass of link 8
Eigen::Matrix<double,6,7> thunder_Rizon4s::get_J_cm_8() {
	thread_local double buffer[42];
	thread_local long long p3[Rizon4s_J_cm_8_fun_SZ_IW];
	thread_local double p4[Rizon4s_J_cm_8_fun_SZ_W];
	const double* input_[] = {q.data(), world2L0_xyz.data(), world2L0_ypr.data(), Ln2EE_xyz.data(), Ln2EE_ypr.data(), par_DYN.data()};
	double* output_[] = {buffer};
	int check = Rizon4s_J_cm_8_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,7>>(buffer);
}

// Jacobian of frame flange
Eigen::Matrix<double,6,7> thunder_Rizon4s::get_J_flange() {
	thread_local double buffer[42];
	thread_local long long p3[Rizon4s_J_flange_fun_SZ_IW];
	thread_local double p4[Rizon4s_J_flange_fun_SZ_W];
	const double* input_[] = {q.data(), world2L0_xyz.data(), world2L0_ypr.data(), Ln2EE_xyz.data(), Ln2EE_ypr.data()};
	double* output_[] = {buffer};
	int check = Rizon4s_J_flange_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,7>>(buffer);
}

// Time second derivative of jacobian matrix of frame flange
Eigen::Matrix<double,6,7> thunder_Rizon4s::get_J_flange_ddot() {
	thread_local double buffer[42];
	thread_local long long p3[Rizon4s_J_flange_ddot_fun_SZ_IW];
	thread_local double p4[Rizon4s_J_flange_ddot_fun_SZ_W];
	const double* input_[] = {q.data(), dq.data(), ddq.data(), world2L0_xyz.data(), world2L0_ypr.data(), Ln2EE_xyz.data(), Ln2EE_ypr.data()};
	double* output_[] = {buffer};
	int check = Rizon4s_J_flange_ddot_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,7>>(buffer);
}

// Time derivative of jacobian matrix of frame flange
Eigen::Matrix<double,6,7> thunder_Rizon4s::get_J_flange_dot() {
	thread_local double buffer[42];
	thread_local long long p3[Rizon4s_J_flange_dot_fun_SZ_IW];
	thread_local double p4[Rizon4s_J_flange_dot_fun_SZ_W];
	const double* input_[] = {q.data(), dq.data(), world2L0_xyz.data(), world2L0_ypr.data(), Ln2EE_xyz.data(), Ln2EE_ypr.data()};
	double* output_[] = {buffer};
	int check = Rizon4s_J_flange_dot_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,7>>(buffer);
}

// Pseudo-Inverse of jacobian matrix of frame flange
Eigen::Matrix<double,7,6> thunder_Rizon4s::get_J_flange_pinv() {
	thread_local double buffer[42];
	thread_local long long p3[Rizon4s_J_flange_pinv_fun_SZ_IW];
	thread_local double p4[Rizon4s_J_flange_pinv_fun_SZ_W];
	const double* input_[] = {q.data(), world2L0_xyz.data(), world2L0_ypr.data(), Ln2EE_xyz.data(), Ln2EE_ypr.data()};
	double* output_[] = {buffer};
	int check = Rizon4s_J_flange_pinv_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,7,6>>(buffer);
}

// Manipulator mass matrix
Eigen::Matrix<double,7,7> thunder_Rizon4s::get_M() {
	thread_local double buffer[49];
	thread_local long long p3[Rizon4s_M_fun_SZ_IW];
	thread_local double p4[Rizon4s_M_fun_SZ_W];
	const double* input_[] = {q.data(), world2L0_xyz.data(), world2L0_ypr.data(), Ln2EE_xyz.data(), Ln2EE_ypr.data(), par_DYN.data()};
	double* output_[] = {buffer};
	int check = Rizon4s_M_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,7,7>>(buffer);
}

// Second time derivative of the mass matrix
Eigen::Matrix<double,7,7> thunder_Rizon4s::get_M_ddot() {
	thread_local double buffer[49];
	thread_local long long p3[Rizon4s_M_ddot_fun_SZ_IW];
	thread_local double p4[Rizon4s_M_ddot_fun_SZ_W];
	const double* input_[] = {q.data(), dq.data(), ddq.data(), world2L0_xyz.data(), world2L0_ypr.data(), Ln2EE_xyz.data(), Ln2EE_ypr.data(), par_DYN.data()};
	double* output_[] = {buffer};
	int check = Rizon4s_M_ddot_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,7,7>>(buffer);
}

// Time derivative of the mass matrix
Eigen::Matrix<double,7,7> thunder_Rizon4s::get_M_dot() {
	thread_local double buffer[49];
	thread_local long long p3[Rizon4s_M_dot_fun_SZ_IW];
	thread_local double p4[Rizon4s_M_dot_fun_SZ_W];
	const double* input_[] = {q.data(), dq.data(), world2L0_xyz.data(), world2L0_ypr.data(), Ln2EE_xyz.data(), Ln2EE_ypr.data(), par_DYN.data()};
	double* output_[] = {buffer};
	int check = Rizon4s_M_dot_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,7,7>>(buffer);
}

// relative transformation from frame-1to frame 0
Eigen::Matrix<double,4,4> thunder_Rizon4s::get_T_0() {
	thread_local double buffer[16];
	thread_local long long p3[Rizon4s_T_0_fun_SZ_IW];
	thread_local double p4[Rizon4s_T_0_fun_SZ_W];
	const double* input_[] = {q.data(), world2L0_xyz.data(), world2L0_ypr.data(), Ln2EE_xyz.data(), Ln2EE_ypr.data()};
	double* output_[] = {buffer};
	int check = Rizon4s_T_0_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// relative transformation from frame0to frame 1
Eigen::Matrix<double,4,4> thunder_Rizon4s::get_T_1() {
	thread_local double buffer[16];
	thread_local long long p3[Rizon4s_T_1_fun_SZ_IW];
	thread_local double p4[Rizon4s_T_1_fun_SZ_W];
	const double* input_[] = {q.data(), world2L0_xyz.data(), world2L0_ypr.data(), Ln2EE_xyz.data(), Ln2EE_ypr.data()};
	double* output_[] = {buffer};
	int check = Rizon4s_T_1_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// relative transformation from frame1to frame 2
Eigen::Matrix<double,4,4> thunder_Rizon4s::get_T_2() {
	thread_local double buffer[16];
	thread_local long long p3[Rizon4s_T_2_fun_SZ_IW];
	thread_local double p4[Rizon4s_T_2_fun_SZ_W];
	const double* input_[] = {q.data(), world2L0_xyz.data(), world2L0_ypr.data(), Ln2EE_xyz.data(), Ln2EE_ypr.data()};
	double* output_[] = {buffer};
	int check = Rizon4s_T_2_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// relative transformation from frame2to frame 3
Eigen::Matrix<double,4,4> thunder_Rizon4s::get_T_3() {
	thread_local double buffer[16];
	thread_local long long p3[Rizon4s_T_3_fun_SZ_IW];
	thread_local double p4[Rizon4s_T_3_fun_SZ_W];
	const double* input_[] = {q.data(), world2L0_xyz.data(), world2L0_ypr.data(), Ln2EE_xyz.data(), Ln2EE_ypr.data()};
	double* output_[] = {buffer};
	int check = Rizon4s_T_3_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// relative transformation from frame3to frame 4
Eigen::Matrix<double,4,4> thunder_Rizon4s::get_T_4() {
	thread_local double buffer[16];
	thread_local long long p3[Rizon4s_T_4_fun_SZ_IW];
	thread_local double p4[Rizon4s_T_4_fun_SZ_W];
	const double* input_[] = {q.data(), world2L0_xyz.data(), world2L0_ypr.data(), Ln2EE_xyz.data(), Ln2EE_ypr.data()};
	double* output_[] = {buffer};
	int check = Rizon4s_T_4_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// relative transformation from frame4to frame 5
Eigen::Matrix<double,4,4> thunder_Rizon4s::get_T_5() {
	thread_local double buffer[16];
	thread_local long long p3[Rizon4s_T_5_fun_SZ_IW];
	thread_local double p4[Rizon4s_T_5_fun_SZ_W];
	const double* input_[] = {q.data(), world2L0_xyz.data(), world2L0_ypr.data(), Ln2EE_xyz.data(), Ln2EE_ypr.data()};
	double* output_[] = {buffer};
	int check = Rizon4s_T_5_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// relative transformation from frame5to frame 6
Eigen::Matrix<double,4,4> thunder_Rizon4s::get_T_6() {
	thread_local double buffer[16];
	thread_local long long p3[Rizon4s_T_6_fun_SZ_IW];
	thread_local double p4[Rizon4s_T_6_fun_SZ_W];
	const double* input_[] = {q.data(), world2L0_xyz.data(), world2L0_ypr.data(), Ln2EE_xyz.data(), Ln2EE_ypr.data()};
	double* output_[] = {buffer};
	int check = Rizon4s_T_6_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// relative transformation from frame6to frame 7
Eigen::Matrix<double,4,4> thunder_Rizon4s::get_T_7() {
	thread_local double buffer[16];
	thread_local long long p3[Rizon4s_T_7_fun_SZ_IW];
	thread_local double p4[Rizon4s_T_7_fun_SZ_W];
	const double* input_[] = {q.data(), world2L0_xyz.data(), world2L0_ypr.data(), Ln2EE_xyz.data(), Ln2EE_ypr.data()};
	double* output_[] = {buffer};
	int check = Rizon4s_T_7_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// relative transformation from frame7to frame 8
Eigen::Matrix<double,4,4> thunder_Rizon4s::get_T_8() {
	thread_local double buffer[16];
	thread_local long long p3[Rizon4s_T_8_fun_SZ_IW];
	thread_local double p4[Rizon4s_T_8_fun_SZ_W];
	const double* input_[] = {q.data(), world2L0_xyz.data(), world2L0_ypr.data(), Ln2EE_xyz.data(), Ln2EE_ypr.data()};
	double* output_[] = {buffer};
	int check = Rizon4s_T_8_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// Template transformation of a fixed joint
Eigen::Matrix<double,4,4> thunder_Rizon4s::get_T_JOINT_FIXED(Matrix<double,1,1> q_joint, Matrix<double,3,1> axis) {
	thread_local double buffer[16];
	thread_local long long p3[Rizon4s_T_JOINT_FIXED_fun_SZ_IW];
	thread_local double p4[Rizon4s_T_JOINT_FIXED_fun_SZ_W];
	const double* input_[] = {q_joint.data(), axis.data()};
	double* output_[] = {buffer};
	int check = Rizon4s_T_JOINT_FIXED_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// Template transformation of general prismatic joint R
Eigen::Matrix<double,4,4> thunder_Rizon4s::get_T_JOINT_P(Matrix<double,1,1> q_joint, Matrix<double,3,1> axis) {
	thread_local double buffer[16];
	thread_local long long p3[Rizon4s_T_JOINT_P_fun_SZ_IW];
	thread_local double p4[Rizon4s_T_JOINT_P_fun_SZ_W];
	const double* input_[] = {q_joint.data(), axis.data()};
	double* output_[] = {buffer};
	int check = Rizon4s_T_JOINT_P_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// Template transformation of general rotoidal joint R
Eigen::Matrix<double,4,4> thunder_Rizon4s::get_T_JOINT_R(Matrix<double,1,1> q_joint, Matrix<double,3,1> axis) {
	thread_local double buffer[16];
	thread_local long long p3[Rizon4s_T_JOINT_R_fun_SZ_IW];
	thread_local double p4[Rizon4s_T_JOINT_R_fun_SZ_W];
	const double* input_[] = {q_joint.data(), axis.data()};
	double* output_[] = {buffer};
	int check = Rizon4s_T_JOINT_R_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// absolute transformation from frame world to frame 0
Eigen::Matrix<double,4,4> thunder_Rizon4s::get_T_w_0() {
	thread_local double buffer[16];
	thread_local long long p3[Rizon4s_T_w_0_fun_SZ_IW];
	thread_local double p4[Rizon4s_T_w_0_fun_SZ_W];
	const double* input_[] = {q.data(), world2L0_xyz.data(), world2L0_ypr.data(), Ln2EE_xyz.data(), Ln2EE_ypr.data()};
	double* output_[] = {buffer};
	int check = Rizon4s_T_w_0_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// absolute transformation from frame world to frame 1
Eigen::Matrix<double,4,4> thunder_Rizon4s::get_T_w_1() {
	thread_local double buffer[16];
	thread_local long long p3[Rizon4s_T_w_1_fun_SZ_IW];
	thread_local double p4[Rizon4s_T_w_1_fun_SZ_W];
	const double* input_[] = {q.data(), world2L0_xyz.data(), world2L0_ypr.data(), Ln2EE_xyz.data(), Ln2EE_ypr.data()};
	double* output_[] = {buffer};
	int check = Rizon4s_T_w_1_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// absolute transformation from frame world to frame 2
Eigen::Matrix<double,4,4> thunder_Rizon4s::get_T_w_2() {
	thread_local double buffer[16];
	thread_local long long p3[Rizon4s_T_w_2_fun_SZ_IW];
	thread_local double p4[Rizon4s_T_w_2_fun_SZ_W];
	const double* input_[] = {q.data(), world2L0_xyz.data(), world2L0_ypr.data(), Ln2EE_xyz.data(), Ln2EE_ypr.data()};
	double* output_[] = {buffer};
	int check = Rizon4s_T_w_2_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// absolute transformation from frame world to frame 3
Eigen::Matrix<double,4,4> thunder_Rizon4s::get_T_w_3() {
	thread_local double buffer[16];
	thread_local long long p3[Rizon4s_T_w_3_fun_SZ_IW];
	thread_local double p4[Rizon4s_T_w_3_fun_SZ_W];
	const double* input_[] = {q.data(), world2L0_xyz.data(), world2L0_ypr.data(), Ln2EE_xyz.data(), Ln2EE_ypr.data()};
	double* output_[] = {buffer};
	int check = Rizon4s_T_w_3_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// absolute transformation from frame world to frame 4
Eigen::Matrix<double,4,4> thunder_Rizon4s::get_T_w_4() {
	thread_local double buffer[16];
	thread_local long long p3[Rizon4s_T_w_4_fun_SZ_IW];
	thread_local double p4[Rizon4s_T_w_4_fun_SZ_W];
	const double* input_[] = {q.data(), world2L0_xyz.data(), world2L0_ypr.data(), Ln2EE_xyz.data(), Ln2EE_ypr.data()};
	double* output_[] = {buffer};
	int check = Rizon4s_T_w_4_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// absolute transformation from frame world to frame 5
Eigen::Matrix<double,4,4> thunder_Rizon4s::get_T_w_5() {
	thread_local double buffer[16];
	thread_local long long p3[Rizon4s_T_w_5_fun_SZ_IW];
	thread_local double p4[Rizon4s_T_w_5_fun_SZ_W];
	const double* input_[] = {q.data(), world2L0_xyz.data(), world2L0_ypr.data(), Ln2EE_xyz.data(), Ln2EE_ypr.data()};
	double* output_[] = {buffer};
	int check = Rizon4s_T_w_5_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// absolute transformation from frame world to frame 6
Eigen::Matrix<double,4,4> thunder_Rizon4s::get_T_w_6() {
	thread_local double buffer[16];
	thread_local long long p3[Rizon4s_T_w_6_fun_SZ_IW];
	thread_local double p4[Rizon4s_T_w_6_fun_SZ_W];
	const double* input_[] = {q.data(), world2L0_xyz.data(), world2L0_ypr.data(), Ln2EE_xyz.data(), Ln2EE_ypr.data()};
	double* output_[] = {buffer};
	int check = Rizon4s_T_w_6_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// absolute transformation from frame world to frame 7
Eigen::Matrix<double,4,4> thunder_Rizon4s::get_T_w_7() {
	thread_local double buffer[16];
	thread_local long long p3[Rizon4s_T_w_7_fun_SZ_IW];
	thread_local double p4[Rizon4s_T_w_7_fun_SZ_W];
	const double* input_[] = {q.data(), world2L0_xyz.data(), world2L0_ypr.data(), Ln2EE_xyz.data(), Ln2EE_ypr.data()};
	double* output_[] = {buffer};
	int check = Rizon4s_T_w_7_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// absolute transformation from frame world to frame 8
Eigen::Matrix<double,4,4> thunder_Rizon4s::get_T_w_8() {
	thread_local double buffer[16];
	thread_local long long p3[Rizon4s_T_w_8_fun_SZ_IW];
	thread_local double p4[Rizon4s_T_w_8_fun_SZ_W];
	const double* input_[] = {q.data(), world2L0_xyz.data(), world2L0_ypr.data(), Ln2EE_xyz.data(), Ln2EE_ypr.data()};
	double* output_[] = {buffer};
	int check = Rizon4s_T_w_8_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// absolute transformation from frame world to frame flange
Eigen::Matrix<double,4,4> thunder_Rizon4s::get_T_w_flange() {
	thread_local double buffer[16];
	thread_local long long p3[Rizon4s_T_w_flange_fun_SZ_IW];
	thread_local double p4[Rizon4s_T_w_flange_fun_SZ_W];
	const double* input_[] = {q.data(), world2L0_xyz.data(), world2L0_ypr.data(), Ln2EE_xyz.data(), Ln2EE_ypr.data()};
	double* output_[] = {buffer};
	int check = Rizon4s_T_w_flange_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// Standard manipulator regressor matrix
Eigen::Matrix<double,7,90> thunder_Rizon4s::get_Y() {
	thread_local double buffer[630];
	thread_local long long p3[Rizon4s_Y_fun_SZ_IW];
	thread_local double p4[Rizon4s_Y_fun_SZ_W];
	const double* input_[] = {q.data(), dq.data(), ddq.data(), world2L0_xyz.data(), world2L0_ypr.data(), Ln2EE_xyz.data(), Ln2EE_ypr.data()};
	double* output_[] = {buffer};
	int check = Rizon4s_Y_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,7,90>>(buffer);
}

// Slotine-Li manipulator regressor matrix
Eigen::Matrix<double,7,90> thunder_Rizon4s::get_Yr() {
	thread_local double buffer[630];
	thread_local long long p3[Rizon4s_Yr_fun_SZ_IW];
	thread_local double p4[Rizon4s_Yr_fun_SZ_W];
	const double* input_[] = {q.data(), dq.data(), dqr.data(), ddqr.data(), world2L0_xyz.data(), world2L0_ypr.data(), Ln2EE_xyz.data(), Ln2EE_ypr.data()};
	double* output_[] = {buffer};
	int check = Rizon4s_Yr_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,7,90>>(buffer);
}

// Conversion from dynamic to regressor parameters
Eigen::Matrix<double,90,1> thunder_Rizon4s::get_dyn2reg() {
	thread_local double buffer[90];
	thread_local long long p3[Rizon4s_dyn2reg_fun_SZ_IW];
	thread_local double p4[Rizon4s_dyn2reg_fun_SZ_W];
	const double* input_[] = {par_DYN.data()};
	double* output_[] = {buffer};
	int check = Rizon4s_dyn2reg_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,90,1>>(buffer);
}

// Kinematic parameters
Eigen::Matrix<double,54,1> thunder_Rizon4s::get_par_KIN() {
	thread_local double buffer[54];
	thread_local long long p3[Rizon4s_par_KIN_fun_SZ_IW];
	thread_local double p4[Rizon4s_par_KIN_fun_SZ_W];
	const double* input_[] = {world2L0_xyz.data(), world2L0_ypr.data(), Ln2EE_xyz.data(), Ln2EE_ypr.data()};
	double* output_[] = {buffer};
	int check = Rizon4s_par_KIN_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,54,1>>(buffer);
}

// Last link to end-effector frame.
Eigen::Matrix<double,6,1> thunder_Rizon4s::get_par_Ln2EE() {
	thread_local double buffer[6];
	thread_local long long p3[Rizon4s_par_Ln2EE_fun_SZ_IW];
	thread_local double p4[Rizon4s_par_Ln2EE_fun_SZ_W];
	const double* input_[] = {Ln2EE_xyz.data(), Ln2EE_ypr.data()};
	double* output_[] = {buffer};
	int check = Rizon4s_par_Ln2EE_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,1>>(buffer);
}

// World to base frame.
Eigen::Matrix<double,6,1> thunder_Rizon4s::get_par_world2L0() {
	thread_local double buffer[6];
	thread_local long long p3[Rizon4s_par_world2L0_fun_SZ_IW];
	thread_local double p4[Rizon4s_par_world2L0_fun_SZ_W];
	const double* input_[] = {world2L0_xyz.data(), world2L0_ypr.data()};
	double* output_[] = {buffer};
	int check = Rizon4s_par_world2L0_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,1>>(buffer);
}

// Conversion from regressor to dynamic parameters
Eigen::Matrix<double,90,1> thunder_Rizon4s::get_reg2dyn() {
	thread_local double buffer[90];
	thread_local long long p3[Rizon4s_reg2dyn_fun_SZ_IW];
	thread_local double p4[Rizon4s_reg2dyn_fun_SZ_W];
	const double* input_[] = {par_REG.data()};
	double* output_[] = {buffer};
	int check = Rizon4s_reg2dyn_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,90,1>>(buffer);
}

// Regressor matrix of term C*dqr
Eigen::Matrix<double,7,90> thunder_Rizon4s::get_reg_C() {
	thread_local double buffer[630];
	thread_local long long p3[Rizon4s_reg_C_fun_SZ_IW];
	thread_local double p4[Rizon4s_reg_C_fun_SZ_W];
	const double* input_[] = {q.data(), dq.data(), dqr.data(), world2L0_xyz.data(), world2L0_ypr.data(), Ln2EE_xyz.data(), Ln2EE_ypr.data()};
	double* output_[] = {buffer};
	int check = Rizon4s_reg_C_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,7,90>>(buffer);
}

// Regressor matrix of term G
Eigen::Matrix<double,7,90> thunder_Rizon4s::get_reg_G() {
	thread_local double buffer[630];
	thread_local long long p3[Rizon4s_reg_G_fun_SZ_IW];
	thread_local double p4[Rizon4s_reg_G_fun_SZ_W];
	const double* input_[] = {q.data(), world2L0_xyz.data(), world2L0_ypr.data(), Ln2EE_xyz.data(), Ln2EE_ypr.data()};
	double* output_[] = {buffer};
	int check = Rizon4s_reg_G_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,7,90>>(buffer);
}

// Regressor matrix of the quantity J^T*w of link flange
Eigen::Matrix<double,7,12> thunder_Rizon4s::get_reg_JTw_flange() {
	thread_local double buffer[84];
	thread_local long long p3[Rizon4s_reg_JTw_flange_fun_SZ_IW];
	thread_local double p4[Rizon4s_reg_JTw_flange_fun_SZ_W];
	const double* input_[] = {q.data(), w.data(), world2L0_xyz.data(), world2L0_ypr.data(), Ln2EE_xyz.data(), Ln2EE_ypr.data()};
	double* output_[] = {buffer};
	int check = Rizon4s_reg_JTw_flange_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,7,12>>(buffer);
}

// Regressor matrix of the quantity J*dq of link flange
Eigen::Matrix<double,6,12> thunder_Rizon4s::get_reg_Jdq_flange() {
	thread_local double buffer[72];
	thread_local long long p3[Rizon4s_reg_Jdq_flange_fun_SZ_IW];
	thread_local double p4[Rizon4s_reg_Jdq_flange_fun_SZ_W];
	const double* input_[] = {q.data(), dq.data(), world2L0_xyz.data(), world2L0_ypr.data(), Ln2EE_xyz.data(), Ln2EE_ypr.data()};
	double* output_[] = {buffer};
	int check = Rizon4s_reg_Jdq_flange_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,12>>(buffer);
}

// Regressor matrix of term M*ddqr
Eigen::Matrix<double,7,90> thunder_Rizon4s::get_reg_M() {
	thread_local double buffer[630];
	thread_local long long p3[Rizon4s_reg_M_fun_SZ_IW];
	thread_local double p4[Rizon4s_reg_M_fun_SZ_W];
	const double* input_[] = {q.data(), ddqr.data(), world2L0_xyz.data(), world2L0_ypr.data(), Ln2EE_xyz.data(), Ln2EE_ypr.data()};
	double* output_[] = {buffer};
	int check = Rizon4s_reg_M_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,7,90>>(buffer);
}

