#ifndef USE_IKFOM_H
#define USE_IKFOM_H
//  Annotated by zlwang  2022.9
/*
这个hpp主要包含：
1.状态变量、输入、噪声量的定义  
2.噪声协方差的初始化   
3.前向传播中fx和fw矩阵的生成   
4.SO3转欧拉角
*/
#include <IKFoM_toolkit/esekfom/esekfom.hpp>

typedef MTK::vect<3, double> vect3;
typedef MTK::SO3<double> SO3;
typedef MTK::S2<double, 98090, 10000, 1> S2; 
typedef MTK::vect<1, double> vect1;
typedef MTK::vect<2, double> vect2;

//状态变量(23维) 注意重力分量是二维的流形(在S2.hpp中有该流形的定义)
MTK_BUILD_MANIFOLD(state_ikfom,
((vect3, pos))
((SO3, rot))
((SO3, offset_R_L_I))
((vect3, offset_T_L_I))
((vect3, vel))
((vect3, bg))
((vect3, ba))
((S2, grav))
);
//输入u(6维)，也就是IMU的实际测量值
MTK_BUILD_MANIFOLD(input_ikfom,
((vect3, acc))
((vect3, gyro))
);
//噪声w(12维)
MTK_BUILD_MANIFOLD(process_noise_ikfom,
((vect3, ng))
((vect3, na))
((vect3, nbg))
((vect3, nba))
);

//噪声协方差Q的初始化(对应公式(8)的Q, 在IMU_Processing.hpp中使用)
MTK::get_cov<process_noise_ikfom>::type process_noise_cov()
{
	MTK::get_cov<process_noise_ikfom>::type cov = MTK::get_cov<process_noise_ikfom>::type::Zero();
	MTK::setDiagonal<process_noise_ikfom, vect3, 0>(cov, &process_noise_ikfom::ng, 0.0001);// 0.03
	MTK::setDiagonal<process_noise_ikfom, vect3, 3>(cov, &process_noise_ikfom::na, 0.0001); // *dt 0.01 0.01 * dt * dt 0.05
	MTK::setDiagonal<process_noise_ikfom, vect3, 6>(cov, &process_noise_ikfom::nbg, 0.00001); // *dt 0.00001 0.00001 * dt *dt 0.3 //0.001 0.0001 0.01
	MTK::setDiagonal<process_noise_ikfom, vect3, 9>(cov, &process_noise_ikfom::nba, 0.00001);   //0.001 0.05 0.0001/out 0.01
	return cov;
}

//double L_offset_to_I[3] = {0.04165, 0.02326, -0.0284}; // Avia 
//vect3 Lidar_offset_to_IMU(L_offset_to_I, 3);
Eigen::Matrix<double, 24, 1> get_f(state_ikfom &s, const input_ikfom &in)  //对应公式(2) 中的f
{
	// 对应顺序为速度(3)，角速度(3),外参偏置T(3),外参偏置R(3)，加速度(3),角速度偏置(3),加速度偏置(3),位置(3)，与论文公式不一致
	Eigen::Matrix<double, 24, 1> res = Eigen::Matrix<double, 24, 1>::Zero();
	vect3 omega;
	in.gyro.boxminus(omega, s.bg);    				 // 输入的imu的角速度(也就是实际测量值) - 估计的bias值(对应公式的第1行)
	vect3 a_inertial = s.rot * (in.acc-s.ba); 		//  输入的imu的加速度，先转到世界坐标系（对应公式的第3行）
	for(int i = 0; i < 3; i++ ){
		res(i) = s.vel[i];							//速度（对应公式第2行）
		res(i + 3) =  omega[i]; 					//角速度（对应公式第1行）
		res(i + 12) = a_inertial[i] + s.grav[i]; 	//更新的加速度
	}
	return res;
	//TODO 这里公式中的4-5行直接令成0了  是不是该加上process_noise_ikfom(噪声量)试试？
}

//对应公式(7)的Fx  注意该矩阵没乘dt，没加单位阵
Eigen::Matrix<double, 24, 23> df_dx(state_ikfom &s, const input_ikfom &in)
{
	//当中的23个对应了status的维度计算，为pos(3), rot(3),offset_R_L_I(3),offset_T_L_I(3), vel(3), bg(3), ba(3), grav(2);和fast-lio论文顺序不同
	Eigen::Matrix<double, 24, 23> cov = Eigen::Matrix<double, 24, 23>::Zero();
	// 
	cov.template block<3, 3>(0, 12) = Eigen::Matrix3d::Identity();  //对应公式(7)第2行第3列   I
	//TODO 这个点template啥意思？ 效果上和.block是一样的
	vect3 acc_;
	in.acc.boxminus(acc_, s.ba);  		//测量加速度a_m - bias
	vect3 omega;
	in.gyro.boxminus(omega, s.bg);		//测量角速度w_m - bias
	cov.template block<3, 3>(12, 3) = -s.rot.toRotationMatrix()*MTK::hat(acc_);   	//对应公式(7)第3行第1列      -Ra^ 
	cov.template block<3, 3>(12, 18) = -s.rot.toRotationMatrix();					//对应公式(7)第3行第5列 	 -R
	Eigen::Matrix<state_ikfom::scalar, 2, 1> vec = Eigen::Matrix<state_ikfom::scalar, 2, 1>::Zero();
	Eigen::Matrix<state_ikfom::scalar, 3, 2> grav_matrix;
	s.S2_Mx(grav_matrix, vec, 21);
	cov.template block<3, 2>(12, 21) =  grav_matrix;  //对应公式(7)第3行第6列   I
	cov.template block<3, 3>(3, 15) = -Eigen::Matrix3d::Identity(); 	//对应公式(7)第1行第4列   -I
	return cov;
}

//对应公式(7)的Fw
Eigen::Matrix<double, 24, 12> df_dw(state_ikfom &s, const input_ikfom &in)
{
	Eigen::Matrix<double, 24, 12> cov = Eigen::Matrix<double, 24, 12>::Zero();    
	cov.template block<3, 3>(12, 3) = -s.rot.toRotationMatrix();        	//对应公式(7)第3行第2列  -R 
	cov.template block<3, 3>(3, 0) = -Eigen::Matrix3d::Identity();			//对应公式(7)第1行第1列  -A(w dt)  直接简化乘I
	cov.template block<3, 3>(15, 6) = Eigen::Matrix3d::Identity();			//对应公式(7)第4行第3列  I
	cov.template block<3, 3>(18, 9) = Eigen::Matrix3d::Identity();			//对应公式(7)第5行第4列  I
	return cov;
}

//从SO3转换到欧拉角(输出为deg)
vect3 SO3ToEuler(const SO3 &orient) 
{
	Eigen::Matrix<double, 3, 1> _ang;     //欧拉角
	Eigen::Vector4d q_data = orient.coeffs().transpose();    //四元数  对应xyzw
	//scalar w=orient.coeffs[3], x=orient.coeffs[0], y=orient.coeffs[1], z=orient.coeffs[2];
	double sqw = q_data[3]*q_data[3];
	double sqx = q_data[0]*q_data[0];
	double sqy = q_data[1]*q_data[1];
	double sqz = q_data[2]*q_data[2];
	double unit = sqx + sqy + sqz + sqw; // if normalized is one, otherwise is correction factor
	double test = q_data[3]*q_data[1] - q_data[2]*q_data[0];

	if (test > 0.49999*unit) { // singularity at north pole
	
		_ang << 2 * std::atan2(q_data[0], q_data[3]), M_PI/2, 0;
		double temp[3] = {_ang[0] * 57.3, _ang[1] * 57.3, _ang[2] * 57.3};
		vect3 euler_ang(temp, 3);
		return euler_ang;
	}
	if (test < -0.49999*unit) { // singularity at south pole
		_ang << -2 * std::atan2(q_data[0], q_data[3]), -M_PI/2, 0;
		double temp[3] = {_ang[0] * 57.3, _ang[1] * 57.3, _ang[2] * 57.3};
		vect3 euler_ang(temp, 3);
		return euler_ang;
	}
		
	_ang <<
			std::atan2(2*q_data[0]*q_data[3]+2*q_data[1]*q_data[2] , -sqx - sqy + sqz + sqw),
			std::asin (2*test/unit),
			std::atan2(2*q_data[2]*q_data[3]+2*q_data[1]*q_data[0] , sqx - sqy - sqz + sqw);
	double temp[3] = {_ang[0] * 57.3, _ang[1] * 57.3, _ang[2] * 57.3};    //rad转到deg
	vect3 euler_ang(temp, 3);
		// euler_ang[0] = roll, euler_ang[1] = pitch, euler_ang[2] = yaw
	return euler_ang;
}

#endif