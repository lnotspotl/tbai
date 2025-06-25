#ifndef LIB_HPP
#define LIB_HPP

#include <Eigen/Dense>
#include <iostream>
#include <iomanip>

namespace state_estimator {

	//TODO: MOVE TO LIBRARY

	inline Eigen::Matrix<double,3,3> calc_SSM(const Eigen::Matrix<double,3,1> &x) {
		Eigen::Matrix<double,3,3> X;
		X <<	0.0, -x(2), x(1),
			x(2), 0.0, -x(0),
			-x(1), x(0), 0.0;
		return X;
	}

	inline Eigen::Matrix<double,3,3> calc_Rquat(const Eigen::Matrix<double,4,1> &q) {
		double eta = q(0);
		Eigen::Matrix<double,3,1> eps = q.tail(3);
		Eigen::Matrix<double,3,3> S = calc_SSM(eps);
		Eigen::Matrix<double,3,3> R = Eigen::Matrix<double,3,3>::Identity() + 2*eta*S + 2*S*S;
		return R;
		
	}

	inline Eigen::Matrix<double,3,3> calc_Rzyx(const Eigen::Matrix<double,3,1>&eta) {
		Eigen::Matrix<double,3,3> R;
		double cphi = cos(eta(0));
		double sphi = sin(eta(0));
		double cth = cos(eta(1));
		double sth = sin(eta(1));
		double cpsi = cos(eta(2));
		double spsi = sin(eta(2));

		R <<	cpsi*cth,	-spsi*cphi+cpsi*sth*sphi,	spsi*sphi+cpsi*cphi*sth,
			spsi*cth,	cpsi*cphi+sphi*sth*spsi,	-cpsi*sphi+sth*spsi*cphi,
			-sth,		cth*sphi,			cth*cphi;
		
		Eigen::Matrix<double,3,1> u1 = R.block(0,0,3,1);
		Eigen::Matrix<double,3,1> u2 = R.block(0,1,3,1);
		Eigen::Matrix<double,3,1> u3 = R.block(0,2,3,1);
	
		/*std::cout << "****" << std::endl;
		std::cout << u1.norm() << "==1" << std::endl;
		std::cout << u2.norm() << "==1" << std::endl;
		std::cout << u3.norm() << "==1" << std::endl;
		std::cout << u1.transpose() << " x " << u2.transpose() << "=" <<std::endl << u1.cross(u2).transpose() << "\n==\n" << u3.transpose() << std::endl;*/
		return R;
	}

	inline Eigen::Matrix<double,4,1> calc_euler2q(const Eigen::Matrix<double,3,1> &eta) {
		Eigen::Matrix<double,3,3> R = calc_Rzyx(eta);
		double tr = R.trace();
		double p1,p2,p3,p4;

		if (R(0,0)>R(1,1)&&R(0,0)>R(2,2)&&R(0,0)>tr) {
			p1 = sqrt(1+2*R(0,0)-tr);
			p2 = (R(1,0)+R(0,1))/p1;
			p3 = (R(0,2)+R(2,0))/p1;
			p4 = (R(2,1)-R(1,2))/p1;
		} else if (R(1,1)>R(2,2)&&R(1,1)>tr) {
			p2 = sqrt(1+2*R(1,1)-tr);
			p1 = (R(1,0)+R(0,1))/p2;
			p3 = (R(2,1)+R(1,2))/p2;
			p4 = (R(0,2)-R(2,0))/p2;
		} else if (R(2,2)>tr) {
			p3 = sqrt(1+2*R(2,2)-tr);
			p1 = (R(0,2)+R(2,0))/p3;
			p2 = (R(2,1)+R(1,2))/p3;   
			p4 = (R(1,0)-R(0,1))/p3; 
		} else {
			p4 = sqrt(1+tr);
			p1 = (R(2,1)-R(1,2))/p4;
			p2 = (R(0,2)-R(2,0))/p4;
			p3 = (R(1,0)-R(0,1))/p4;   
		}
		Eigen::Matrix<double,4,1> q;
		q << p4,p1,p2,p3;
		q = 0.5*q;

		q = q/(q.transpose()*q);
		
		/*std::cout << "****" << std::endl;
		std::cout << "norm(q)=" << q.norm() << std::endl;*/
		return q;
	}

	inline Eigen::Matrix<double,4,1> calc_qmult(const Eigen::Matrix<double,4,1> &a, const Eigen::Matrix<double,4,1> &b) {
		double a0 = a(0);
		Eigen::Matrix<double,3,1> ai = a.tail(3);
		Eigen::Matrix<double,4,4> Q; Q << a0, -ai.transpose(), ai, (a0*Eigen::Matrix<double,3,3>::Identity()-calc_SSM(ai));
		Eigen::Matrix<double,4,1> q = Q*b;
		return q;
	}

		inline Eigen::Matrix<double,4,1> calc_quatinv(const Eigen::Matrix<double,4,1> &q) {
		Eigen::Matrix<double,4,1> qinv;
		qinv << q(0), -q(1), -q(2), -q(3);
		return qinv;
	}

	inline Eigen::Matrix<double,3,3> calc_W(const Eigen::Matrix<double,3,1> &eta) {
		double cphi = cos(eta(0));
		double sphi = sin(eta(0));
		double cth = cos(eta(1));
		double sth = sin(eta(1));
	
		Eigen::Matrix<double,3,3> W;
		W <<	1, sphi*sth/cth,	cphi*sth/cth,
			0, cphi,		-sphi,
			0, sphi/cth,		cphi/cth;

		return W;
	}


	/*inline Eigen::Matrix<double,4,1> normalize(const Eigen::Matrix<double,4,1> q) {
		Eigen::Matrix<double,4,1> qn = q/q.norm();
		return qn;
	}*/

	inline Eigen::Matrix<double,3,1> calc_R2euler(const Eigen::Matrix<double,3,3> R) {
		double phi = atan2(R(2,1),R(2,2));
		double theta = -asin(R(2,0));
		double psi = atan2(R(1,0),R(0,0));
		Eigen::Matrix<double,3,1> eta;
		eta << phi, theta, psi;
		return eta;
	}

	inline std::string ns2str(uint64_t t)  {

		uint64_t  y = (t)/31536000000000000;
		uint64_t  d = (t-y*31536000000000000)/86400000000000;
		uint64_t  h = (t-y*31536000000000000-d*86400000000000)/3600000000000;
		uint64_t  m = (t-y*31536000000000000-d*86400000000000-h*3600000000000)/60000000000;
		uint64_t  s = (t-y*31536000000000000-d*86400000000000-h*3600000000000-m*60000000000)/1000000000;
		uint64_t ms = (t-y*31536000000000000-d*86400000000000-h*3600000000000-m*60000000000-s*1000000000)/1000000;
		uint64_t us = (t-y*31536000000000000-d*86400000000000-h*3600000000000-m*60000000000-s*1000000000-ms*1000000)/1000;
		uint64_t ns =  t-y*31536000000000000-d*86400000000000-h*3600000000000-m*60000000000-s*1000000000-ms*1000000-us*1000;

		//ROS_WARN("%lu:%lu:%lu:%lu:%lu:%lu:%lu:%lu",y,d,h,m,s,ms,us,ns);


		uint64_t tmp = t;
		uint64_t x = 1;
		int dec=9;
		for (dec=9;dec>-1;dec--) {
			if (tmp-(tmp/10)*10>0) break;
			tmp=tmp/10;
		}
		if (dec<0) dec=0;
		std::ostringstream o;


		if (y>0) o << y << ":";
		if (y>0||d>0) o << d << ":";
		if (y>0||d>0||h>0) o << h << ":";
		if (y>0||d>0||h>0||m>0) o << m << ":";

		if (y==0&&d==0&&h==0&&m==0) {
			if (s==0) {
				if (ms>0) {
					dec=dec-3;
					if (dec<0) o << ((double)(t-(t/1000000000)*1000000000))/1000000.0f << " ms";
					else o << std::fixed << std::setprecision(dec) << ((double)(t-(t/1000000000)*1000000000))/1000000.0f << " ms";
				} else if (us>0) {
					dec=dec-6;
					if (dec<0) o << ((double)(t-(t/1000000)*1000000))/1000.0f << " us";
					else o << std::fixed << std::setprecision(dec) << ((double)(t-(t/1000000)*1000000))/1000.0f << " us";
				}
				else o << t << " ns";
			} else o << std::fixed << std::setprecision(dec) << ((double)t)/1000000000.0f << " s";
		} else {
			t = t-y*31536000000000000-d*86400000000000-h*3600000000000-m*60000000000;
			o << std::fixed << std::setprecision(dec) << ((double)t)/1000000000.0f;
			if (y>0) o << " y:d:h:m:s";
			else if (d>0) o << " d:h:m:s";
			else if (h>0) o << " h:m:s";
			else o << " m:s";
		}


		return o.str();
	}

} //namespace state_estimator


#endif