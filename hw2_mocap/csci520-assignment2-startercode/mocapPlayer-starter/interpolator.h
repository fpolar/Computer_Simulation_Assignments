/*
  interpolator.h
  
  Create interpolated motion.

  Revision 1 - Alla and Kiran, Jan 18, 2002
  Revision 2 - Jernej Barbic, Yili Zhao, Feb 2012
*/


#ifndef _INTERPOLATOR_H
#define _INTERPOLATOR_H

#include "motion.h"
#include "quaternion.h"

enum InterpolationType
{
  LINEAR = 0, BEZIER = 1
};

enum AngleRepresentation
{
  EULER = 0, QUATERNION = 1
};

class Interpolator
{
public: 
  //constructor, destructor
  Interpolator();
  ~Interpolator();

  //Set interpolation type
  void SetInterpolationType(InterpolationType interpolationType) {m_InterpolationType = interpolationType;};
  //Set angle representation for interpolation
  void SetAngleRepresentation(AngleRepresentation angleRepresentation) {m_AngleRepresentation = angleRepresentation;};

  //Create interpolated motion and store it into pOutputMotion (which will also be allocated)
  void Interpolate(Motion * pInputMotion, Motion ** pOutputMotion, int N);

protected:
  InterpolationType m_InterpolationType; //Interpolation type (Linear, Bezier)
  AngleRepresentation m_AngleRepresentation; //Angle representation (Euler, Quaternion)

  // conversion routines
  // angles are given in degrees; assume XYZ Euler angle order
  void Rotation2Euler(double R[9], double angles[3]);
  void Euler2Rotation(double angles[3], double R[9]);
  void Euler2Quaternion(double angles[3], Quaternion<double> & q); 
  void Quaternion2Euler(Quaternion<double> & q, double angles[3]); 

  // quaternion interpolation
  Quaternion<double> Slerp(double t, Quaternion<double> & qStart, Quaternion<double> & qEnd);
  Quaternion<double> Double(Quaternion<double> p, Quaternion<double> q);

  // interpolation routines
  void LinearInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N);
  void BezierInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N);
  void LinearInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N);
  void BezierInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N);

  // Bezier spline evaluation
  vector DeCasteljauEuler(double t, vector p0, vector p1, vector p2, vector p3); // evaluate Bezier spline at t, using DeCasteljau construction, vector version
  Quaternion<double> DeCasteljauQuaternion(double t, Quaternion<double> p0, Quaternion<double> p1, Quaternion<double> p2, Quaternion<double> p3); // evaluate Bezier spline at t, using DeCasteljau construction, Quaternion version

  //Frank's helper functions

  void MatrixMultiply(double in1[9], double in2[9], double out[9]) {
	  //for (int i = 0; i < 9; i++) {
		 // out[i] = 0;
	  //}
	  //for (int i = 0; i < 3; i++) {
		 // for (int j = 0; j < 3; j++) {
			//  for (int k = 0; k < 3; k++) {
			//	  out[i * 3 + j] += in1[i * 3 + k] * in2[k * 3 + j];
			//  }
		 // }
	  //}
	out[0] = in1[0] * in2[0] + in1[1] * in2[3] + in1[2] * in2[6];
	out[1] = in1[0] * in2[1] + in1[1] * in2[4] + in1[2] * in2[7];
	out[2] = in1[0] * in2[2] + in1[1] * in2[5] + in1[2] * in2[8];
	out[3] = in1[3] * in2[0] + in1[4] * in2[3] + in1[5] * in2[6];
	out[4] = in1[3] * in2[1] + in1[4] * in2[4] + in1[5] * in2[7];
	out[5] = in1[3] * in2[2] + in1[4] * in2[5] + in1[5] * in2[8];
	out[6] = in1[6] * in2[0] + in1[7] * in2[3] + in1[8] * in2[6];
	out[7] = in1[6] * in2[1] + in1[7] * in2[4] + in1[8] * in2[7];
	out[8] = in1[6] * in2[2] + in1[7] * in2[5] + in1[8] * in2[8];
  }
};

#endif
