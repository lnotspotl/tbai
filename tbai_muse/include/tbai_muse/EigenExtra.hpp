#ifndef EIGENEXTRA_HPP
#define EIGENEXTRA_HPP

#include <fstream>
#include <Eigen/Dense>

namespace Eigen {
	typedef Matrix<double,4,4> Matrix4d;
	typedef Matrix<double,5,5> Matrix5d;
	typedef Matrix<double,6,6> Matrix6d;
	typedef Matrix<double,7,7> Matrix7d;
	typedef Matrix<double,8,8> Matrix8d;
	typedef Matrix<double,9,9> Matrix9d;
	typedef Matrix<double,4,1> Vector4d;
	typedef Matrix<double,5,1> Vector5d;
	typedef Matrix<double,6,1> Vector6d;
	typedef Matrix<double,7,1> Vector7d;
	typedef Matrix<double,8,1> Vector8d;
	typedef Matrix<double,9,1> Vector9d;

	template<class Matrix>
	void write_binary(const char* filename, const Matrix& matrix){
		std::ofstream out(filename,std::ios::out | std::ios::binary | std::ios::trunc);
		typename Matrix::Index rows=matrix.rows(), cols=matrix.cols();
		out.write((char*) (&rows), sizeof(typename Matrix::Index));
		out.write((char*) (&cols), sizeof(typename Matrix::Index));
		write_binary(out,matrix);
		out.close();
	}

	template<class Matrix>
	void write_binary(std::ofstream& out, const Matrix& matrix) {
		typename Matrix::Index rows=matrix.rows(), cols=matrix.cols();
		out.write((char*) matrix.data(), rows*cols*sizeof(typename Matrix::Scalar) );
	}


	template<class Matrix>
	void read_binary(const char* filename, Matrix& matrix){
	    std::ifstream in(filename,std::ios::in | std::ios::binary);
	    typename Matrix::Index rows=0, cols=0;
	    in.read((char*) (&rows),sizeof(typename Matrix::Index));
	    in.read((char*) (&cols),sizeof(typename Matrix::Index));
	    matrix.resize(rows, cols);
	    in.read( (char *) matrix.data() , rows*cols*sizeof(typename Matrix::Scalar) );
	    in.close();
	}

} //namespace eigen

#endif
