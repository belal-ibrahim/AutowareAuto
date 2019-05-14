#ifndef NDT_MATCHING__SYMMETRIC_EIGEN_SOLVER_HPP_
#define NDT_MATCHING__SYMMETRIC_EIGEN_SOLVER_HPP_

#include <iostream>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <ndt_matching/visibility_control.hpp>

namespace autoware_bridge
{
namespace ndt_matching
{

class NDT_MATCHING_PUBLIC SymmetricEigensolver3x3 {
public:
	SymmetricEigensolver3x3();

	SymmetricEigensolver3x3(const Matrix3 input_matrix);

	void compute();

	Vector3 eigenvalues();

	Matrix3 eigenvectors();

private:

	void computeEigenvector0(real_t a00, real_t a01, real_t a02, real_t a11, real_t a12, real_t a22, uint32_t i0);

	void computeEigenvector1(real_t a00, real_t a01, real_t a02, real_t a11, real_t a12, real_t a22, uint32_t i0, uint32_t i1);

	void computeEigenvector2(uint32_t i0, uint32_t i1, uint32_t i2);

	void computeOrthogonalComplement(Vector3 &w, Vector3 &u, Vector3 &v);

	Vector3 cross(Vector3 u, Vector3 v);

	Matrix3 input_;
	Matrix3 evecs_;
	Vector3 evals_;
};


SymmetricEigensolver3x3::SymmetricEigensolver3x3()
{
	input_.setZero();
	evecs_.setZero();
	evals_.setZero();
}

SymmetricEigensolver3x3::SymmetricEigensolver3x3(const Matrix3 input_matrix)
{
	input_ = input_matrix;
	evecs_.setZero();
	evals_.setZero();
}

void SymmetricEigensolver3x3::compute()
{
	real_t a00 = input_(0, 0);
	real_t a01 = input_(0, 1);
	real_t a02 = input_(0, 2);
	real_t a11 = input_(1, 1);
	real_t a12 = input_(1, 2);
	real_t a22 = input_(2, 2);

	real_t max0 = (fabs(a00) > fabs(a01)) ? fabs(a00) : fabs(a01);
	real_t max1 = (fabs(a02) > fabs(a11)) ? fabs(a02) : fabs(a11);
	real_t max2 = (fabs(a12) > fabs(a22)) ? fabs(a12) : fabs(a22);

	real_t maxAbsElement = (max0 > max1) ? max0 : max1;

	maxAbsElement = (maxAbsElement > max2) ? maxAbsElement : max2;

	if (maxAbsElement == 0.0) {
		evecs_.setIdentity();
		evals_.setZero();

		return;
	}

	real_t invMaxAbsElement = 1.0 / maxAbsElement;

	a00 *= invMaxAbsElement;
	a01 *= invMaxAbsElement;
	a02 *= invMaxAbsElement;
	a11 *= invMaxAbsElement;
	a12 *= invMaxAbsElement;
	a22 *= invMaxAbsElement;

	real_t norm = a01 * a01 + a02 * a02 + a12 * a12;

	if (norm > 0.0) {
		real_t traceDiv3 = (a00 + a11 + a22) / 3.0;
		real_t b00 = a00 - traceDiv3;
		real_t b11 = a11 - traceDiv3;
		real_t b22 = a22 - traceDiv3;
		real_t denom = sqrt((b00 * b00 + b11 * b11 + b22 * b22 + norm * 2.0) / 6.0);
		real_t c00 = b11 * b22 - a12 * a12;
		real_t c01 = a01 * b22 - a12 * a02;
		real_t c02 = a01 * a12 - b11 * a02;
		real_t det = (b00 * c00 - a01 * c01 + a02 * c02) / (denom * denom * denom);
		real_t halfDet = det * 0.5;

		halfDet = (halfDet > -1.0) ? halfDet : -1.0;
		halfDet = (halfDet < 1.0) ? halfDet : 1.0;

		real_t angle = acos(halfDet) / 3.0;
		real_t beta2 = cos(angle) * 2.0;
		real_t beta0 = cos(angle + M_PI * 2.0 / 3.0) * 2.0;
		real_t beta1 = -(beta0 + beta2);

		evals_(0) = traceDiv3 + denom * beta0;
		evals_(1) = traceDiv3 + denom * beta1;
		evals_(2) = traceDiv3 + denom * beta2;

		uint32_t i0, i2, i1 = 1;

		if (halfDet >= 0.0) {
			i0 = 2;
			i2 = 0;
		} else {
			i0 = 0;
			i2 = 2;
		}

		computeEigenvector0(a00, a01, a02, a11, a12, a22, i0);
		computeEigenvector1(a00, a01, a02, a11, a12, a22, i0, i1);
		computeEigenvector2(i0, i1, i2);

	} else {
		evals_(0) = a00;
		evals_(1) = a11;
		evals_(2) = a22;
		evecs_.setIdentity();
	}

	evals_ *= maxAbsElement;
}

Vector3 SymmetricEigensolver3x3::eigenvalues()
{
	return evals_;
}

Matrix3 SymmetricEigensolver3x3::eigenvectors()
{
	return evecs_;
}


void SymmetricEigensolver3x3::computeEigenvector0(real_t a00, real_t a01, real_t a02, real_t a11, real_t a12, real_t a22, uint32_t i0)
{
	Matrix3 row_mat;
	real_t eval0 = evals_(i0);

	row_mat(0, 0) = a00 - eval0;
	row_mat(0, 1) = a01;
	row_mat(0, 2) = a02;
	row_mat(1, 0) = a01;
	row_mat(1, 1) = a11 - eval0;
	row_mat(1, 2) = a12;
	row_mat(2, 0) = a02;
	row_mat(2, 1) = a12;
	row_mat(2, 2) = a22 - eval0;


	//row0 is r0xr1, row1 is r0xr2, row2 is r1xr2
	Matrix3 rxr;

	rxr.row(0) = cross(row_mat.row(0), row_mat.row(1));
	rxr.row(1) = cross(row_mat.row(0), row_mat.row(2));
	rxr.row(2) = cross(row_mat.row(1), row_mat.row(2));

	real_t d0 = rxr(0, 0) * rxr(0, 0) + rxr(0, 1) * rxr(0, 1) * rxr(0, 2) * rxr(0, 2);
	real_t d1 = rxr(1, 0) * rxr(1, 0) + rxr(1, 1) * rxr(1, 1) * rxr(1, 2) * rxr(1, 2);
	real_t d2 = rxr(2, 0) * rxr(2, 0) + rxr(2, 1) * rxr(2, 1) * rxr(2, 2) * rxr(2, 2);

	real_t dmax = (d0 > d1) ? d0 : d1;
	uint32_t imax = (d0 > d1) ? 0 : 1;

	dmax = (d2 > dmax) ? d2 : dmax;
	imax = (d2 > dmax) ? 2 : imax;

	evecs_.col(i0) = rxr.row(imax) / sqrt(dmax);
}

void SymmetricEigensolver3x3::computeEigenvector1(real_t a00, real_t a01, real_t a02, real_t a11, real_t a12, real_t a22, uint32_t i0, uint32_t i1)
{
	Vector3 u, v;
	Vector3 evec0 = evecs_.col(i0);


	computeOrthogonalComplement(evec0, u, v);

	Vector3 au, av;
	real_t t0, t1, t2;
	real_t eval1 = evals_(i1);

	t0 = u(0);
	t1 = u(1);
	t2 = u(2);

	au(0) = (a00 - eval1) * t0 + a01 * t1 + a02 * t2;
	au(1) = a01 * t0 + (a11 - eval1) * t1 + a12 * t2;
	au(2) = a02 * t0 + a12 * t1 + (a22 - eval1) * t2;

	t0 = v(0);
	t1 = v(1);
	t2 = v(2);

	av(0) = (a00 - eval1) * t0 + a01 * t1 + a02 * t2;
	av(1) = a01 * t0 + (a11 - eval1) * t1 + a12 * t2;
	av(2) = a02 * t0 + a12 * t1 + (a22 - eval1) * t2;

	real_t m00 = u(0) * au(0) + u(1) * au(1) + u(2) * au(2);
	real_t m01 = u(0) * av(0) + u(1) * av(1) + u(2) * av(2);
	real_t m11 = v(0) * av(0) + v(1) * av(1) + v(2) * av(2);

	real_t abs_m00 = fabs(m00);
	real_t abs_m01 = fabs(m01);
	real_t abs_m11 = fabs(m11);

	if (abs_m00 > 0 || abs_m01 > 0 || abs_m11 > 0) {
		real_t u_mult = (abs_m00 >= abs_m11) ? m01 : m11;
		real_t v_mult = (abs_m00 >= abs_m11) ? m00 : m01;

		bool8_t res = fabs(u_mult) >= fabs(v_mult);
		real_t *large = (res) ? &u_mult : &v_mult;
		real_t *small = (res) ? &v_mult : &u_mult;

		*small /= (*large);
		*large = 1.0 / sqrt(1.0 + (*small) * (*small));
		*small *= (*large);

		u *= u_mult;
		v *= v_mult;
		evecs_.col(i1) = u - v;
	} else {
		evecs_.col(i1) = u;
	}
}


Vector3 SymmetricEigensolver3x3::cross(Vector3 u, Vector3 v)
{
	Vector3 out;

	out(0) = u(1) * v(2) - u(2) * v(1);
	out(1) = u(2) * v(0) - u(0) * v(2);
	out(2) = u(0) * v(1) - u(1) * v(0);

	return out;
}

void SymmetricEigensolver3x3::computeOrthogonalComplement(Vector3 &w, Vector3 &u, Vector3 &v)
{
	bool8_t c = (fabs(w(0)) > fabs(w(1)));

	real_t inv_length = (c) ? (1.0 / sqrt(w(0) * w(0) + w(2) * w(2))) : (1.0 / sqrt(w(1) * w(1) + w(2) * w(2)));

	u(0) = (c) ? -w(2) * inv_length : 0.0;
	u(1) = (c) ? 0.0 : w(2) * inv_length;
	u(2) = (c) ? w(0) * inv_length : -w(1) * inv_length;

	v = cross(w, u);
}

void SymmetricEigensolver3x3::computeEigenvector2(uint32_t i0, uint32_t i1, uint32_t i2)
{
	Vector3 evec0 = evecs_.col(i0);
	Vector3 evec1 = evecs_.col(i1);

	evecs_.col(i2) = cross(evec0, evec1);

}
}  // namespace ndt_matching
}  // namespace autoware_bridge

#endif  // NDT_MATCHING__SYMMETRIC_EIGEN_SOLVER_HPP_
