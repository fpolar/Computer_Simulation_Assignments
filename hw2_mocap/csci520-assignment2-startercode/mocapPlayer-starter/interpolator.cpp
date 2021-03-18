#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include "motion.h"
#include "interpolator.h"
#include "types.h"

Interpolator::Interpolator()
{
  //Set default interpolation type
  m_InterpolationType = LINEAR;

  //set default angle representation to use for interpolation
  m_AngleRepresentation = EULER;
}

Interpolator::~Interpolator()
{
}

//Create interpolated motion
void Interpolator::Interpolate(Motion * pInputMotion, Motion ** pOutputMotion, int N) 
{
  //Allocate new motion
  *pOutputMotion = new Motion(pInputMotion->GetNumFrames(), pInputMotion->GetSkeleton()); 

  //Perform the interpolation
  if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == EULER))
    LinearInterpolationEuler(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == QUATERNION))
    LinearInterpolationQuaternion(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == EULER))
    BezierInterpolationEuler(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == QUATERNION))
    BezierInterpolationQuaternion(pInputMotion, *pOutputMotion, N);
  else
  {
    printf("Error: unknown interpolation / angle representation type.\n");
    exit(1);
  }
}

void Interpolator::LinearInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

  int startKeyframe = 0;
  while (startKeyframe + N + 1 < inputLength)
  {
    int endKeyframe = startKeyframe + N + 1;

    Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
    Posture * endPosture = pInputMotion->GetPosture(endKeyframe);

    // copy start and end keyframe
    pOutputMotion->SetPosture(startKeyframe, *startPosture);
    pOutputMotion->SetPosture(endKeyframe, *endPosture);

    // interpolate in between
    for(int frame=1; frame<=N; frame++)
    {
      Posture interpolatedPosture;
      double t = 1.0 * frame / (N+1);

      // interpolate root position
      interpolatedPosture.root_pos = startPosture->root_pos * (1-t) + endPosture->root_pos * t;

      // interpolate bone rotations
	  for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++) {
		  interpolatedPosture.bone_rotation[bone] = startPosture->bone_rotation[bone] * (1 - t) + endPosture->bone_rotation[bone] * t;
	  }

      pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
    }

    startKeyframe = endKeyframe;
  }

  for(int frame=startKeyframe+1; frame<inputLength; frame++)
    pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::Rotation2Euler(double R[9], double angles[3])
{
  double cy = sqrt(R[0]*R[0] + R[3]*R[3]);

  if (cy > 16*DBL_EPSILON) 
  {
    angles[0] = atan2(R[7], R[8]);
    angles[1] = atan2(-R[6], cy);
    angles[2] = atan2(R[3], R[0]);
  } 
  else 
  {
    angles[0] = atan2(-R[5], R[4]);
    angles[1] = atan2(-R[6], cy);
    angles[2] = 0;
  }

  for(int i=0; i<3; i++)
    angles[i] *= 180 / M_PI;
}


void Interpolator::Euler2Rotation(double angles[3], double R[9])
{
  // students should implement this

	double radians[3] = { (angles[0] / 180.00)*M_PI, (angles[0] / 180.00)*M_PI , (angles[0] / 180.00)*M_PI };

	double r3[9] = { cos(radians[2]), -sin(radians[2]), 0.0, /**/ sin(radians[2]), cos(radians[2]), 0.0, /**/ 0.0, 0.0, 1.0 };
	double r2[9] = { cos(radians[2]), 0.0, sin(radians[2]), /**/ 0.0, 1.0, 0.0, /**/ -sin(radians[2]), 0.0, cos(radians[2])};
	double r1[9] = { 1.0, 0.0, 0.0, /**/ 0.0, cos(radians[2]), -sin(radians[2]), /**/ 0.0, sin(radians[2]), cos(radians[2])};

	double r3_r2[9];

	MatrixMultiply(r3, r2, r3_r2);
	MatrixMultiply(r3_r2, r1, R);
}

void Interpolator::BezierInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  // students should implement this
	
	vector p1, p2, p3, p4, a, b;

	int frameCount = pInputMotion->GetNumFrames();
	int startKeyFrame = 0;

	while (startKeyFrame + N + 1 < frameCount) {
		int endKeyFrame = startKeyFrame + N + 1;
		int prevKeyFrame = startKeyFrame - N - 1;
		int nextKeyFrame = endKeyFrame + N + 1;

	// straight copy of the first and last (start/end) key frames 
		Posture *startPosture = pInputMotion->GetPosture(startKeyFrame);
		Posture *endPosture = pInputMotion->GetPosture(endKeyFrame);
		pOutputMotion->SetPosture(startKeyFrame, *startPosture);
		pOutputMotion->SetPosture(endKeyFrame, *endPosture);

	// interpolate the in betweens
		Posture *prevPosture;
		Posture *nextPosture;

		for (int f = 1; f <= N; f++) {
			p2 = startPosture->root_pos;
			p3 = endPosture->root_pos;

			Posture interpolatedPosture;
			double t = 1.0 * f / (N + 1);
			if (startKeyFrame == 0) { 
				//first frame
				nextPosture = pInputMotion->GetPosture(nextKeyFrame);
				p3 = nextPosture->root_pos;
				a = p2 + ((p3 + p3 - p4) - p2) / 3;
				b = p3 - (((p3 + p3 - p2) + p4) / 2 - p3) / 3;
			}else if(nextKeyFrame > frameCount) {
				//last frame
				prevPosture = pInputMotion->GetPosture(prevKeyFrame);
				p1 = prevPosture->root_pos;
				a = p2 + (((p2 + p2 - p1) + p3) / 2 - p2) / 3;
				b = p3 + ((p2 + p2 - p1) - p3) / 3;
			}
			else {
				//in betweens
				prevPosture = pInputMotion->GetPosture(prevKeyFrame);
				nextPosture = pInputMotion->GetPosture(nextKeyFrame);
				p1 = prevPosture->root_pos;
				p4 = nextPosture->root_pos;
				a = p2 + (((p2 + p2 - p1) + p3) / 2 - p2) / 3;
				b = p3 - (((p3 + p3 - p2) + p4) / 2 - p3) / 3;
			}

			interpolatedPosture.root_pos = DeCasteljauEuler(t, p2, a, b, p3);

			// interpolate bone rotations for this frame
			for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
			{
				p2 = startPosture->bone_rotation[bone];
				p3 = endPosture->bone_rotation[bone];

				Posture interpolatedPosture;
				double t = 1.0 * f / (N + 1);
				if (startKeyFrame == 0) {
					//first frame
					nextPosture = pInputMotion->GetPosture(nextKeyFrame);
					p3 = nextPosture->root_pos;
					a = p2 + ((p3 + p3 - p4) - p2) / 3;
					b = p3 - (((p3 + p3 - p2) + p4) / 2 - p3) / 3;
				}
				else if (nextKeyFrame > frameCount) {
					//last frame
					prevPosture = pInputMotion->GetPosture(prevKeyFrame);
					p1 = prevPosture->root_pos;
					a = p2 + (((p2 + p2 - p1) + p3) / 2 - p2) / 3;
					b = p3 + ((p2 + p2 - p1) - p3) / 3;
				}
				else {
					//in betweens
					prevPosture = pInputMotion->GetPosture(prevKeyFrame);
					nextPosture = pInputMotion->GetPosture(nextKeyFrame);
					p1 = prevPosture->root_pos;
					p4 = nextPosture->root_pos;
					a = p2 + (((p2 + p2 - p1) + p3) / 2 - p2) / 3;
					b = p3 - (((p3 + p3 - p2) + p4) / 2 - p3) / 3;
				}
				interpolatedPosture.bone_rotation[bone] = DeCasteljauEuler(t, p2, a, b, p3);
			}
			pOutputMotion->SetPosture(startKeyFrame + f, interpolatedPosture);
		}
		startKeyFrame = endKeyFrame;
	}

	for (int f = startKeyFrame + 1; f < frameCount; f++) {
		pOutputMotion->SetPosture(f, *(pInputMotion->GetPosture(f)));
	}
}

void Interpolator::LinearInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  // students should implement this

	int frameCount = pInputMotion->GetNumFrames();
	int startKeyFrame = 0;

	while (startKeyFrame + N + 1 < frameCount) {
		int endKeyFrame = startKeyFrame + N + 1;

		Posture * startPosture = pInputMotion->GetPosture(startKeyFrame);
		Posture * endPosture = pInputMotion->GetPosture(endKeyFrame);

		pOutputMotion->SetPosture(startKeyFrame, *startPosture);
		pOutputMotion->SetPosture(endKeyFrame, *endPosture);

		for (int f = 1; f <= N; f++) {
			Posture interpolatedPosture;
			double t = 1.0 * f / (N + 1);

			interpolatedPosture.root_pos = startPosture->root_pos * (1 - t) + endPosture->root_pos * t;

			Quaternion<double> startQuat, endQuat;
			for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
			{
				Euler2Quaternion(startPosture->bone_rotation[bone].p, startQuat);
				Euler2Quaternion(endPosture->bone_rotation[bone].p, endQuat);
				Quaternion2Euler(Slerp(t, startQuat, endQuat), interpolatedPosture.bone_rotation[bone].p);
			}
			pOutputMotion->SetPosture(startKeyFrame + f, interpolatedPosture);
		}

		startKeyFrame = endKeyFrame;
	}

	for (int f = startKeyFrame + 1; f < frameCount; f++) {
		pOutputMotion->SetPosture(f, *(pInputMotion->GetPosture(f)));
	}
}

void Interpolator::BezierInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  // students should implement this

	vector p1, p2, p3, p4, a, b;
	Quaternion<double> p1Quat, p2Quat, p3Quat, p4Quat, aQuat, bQuat;

	int frameCount = pInputMotion->GetNumFrames();
	int startKeyFrame = 0;

	while (startKeyFrame + N + 1 < frameCount) {
		int endKeyFrame = startKeyFrame + N + 1;
		int prevKeyFrame = startKeyFrame - N - 1;
		int nextKeyFrame = endKeyFrame + N + 1;

		// straight copy of the first and last (start/end) key frames 
		Posture *startPosture = pInputMotion->GetPosture(startKeyFrame);
		Posture *endPosture = pInputMotion->GetPosture(endKeyFrame);
		pOutputMotion->SetPosture(startKeyFrame, *startPosture);
		pOutputMotion->SetPosture(endKeyFrame, *endPosture);

		// interpolate the in betweens
		Posture *prevPosture;
		Posture *nextPosture;

		for (int f = 1; f <= N; f++) {
			p2 = startPosture->root_pos;
			p3 = endPosture->root_pos;

			Posture interpolatedPosture;
			double t = 1.0 * f / (N + 1);
			if (startKeyFrame == 0) {
				//first frame
				nextPosture = pInputMotion->GetPosture(nextKeyFrame);
				p3 = nextPosture->root_pos;
				a = p1 + ((p2 + p2 - p3) - p1) / 3;
				b = p2 - (((p2 + p2 - p1) + p3) / 2 - p2) / 3;
			}
			else if (nextKeyFrame > frameCount) {
				//last frame
				prevPosture = pInputMotion->GetPosture(prevKeyFrame);
				p1 = prevPosture->root_pos;
				a = p2 + (((p2 + p2 - p1) + p3) / 2 - p2) / 3;
				b = p3 + ((p2 + p2 - p1) - p3) / 3;
			}
			else {
				//in betweens
				prevPosture = pInputMotion->GetPosture(prevKeyFrame);
				nextPosture = pInputMotion->GetPosture(nextKeyFrame);
				p1 = prevPosture->root_pos;
				p4 = nextPosture->root_pos;
				a = p2 + (((p2 + p2 - p1) + p3) / 2 - p2) / 3;
				b = p3 - (((p3 + p3 - p2) + p4) / 2 - p3) / 3;
			}

			interpolatedPosture.root_pos = DeCasteljauEuler(t, p2, a, b, p3);
			int error;
			if (startKeyFrame > 1025) {
				error = 0;
			}

			// interpolate bone rotations
			for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
			{
				Euler2Quaternion(startPosture->bone_rotation[bone].p, p2Quat);
				Euler2Quaternion(endPosture->bone_rotation[bone].p, p3Quat);

				if (startKeyFrame == 0) {
					//first frame
					Euler2Quaternion(nextPosture->bone_rotation[bone].p, p4Quat);

					aQuat = Slerp((1.0 / 3.0), p2Quat, Double(p4Quat, p3Quat));
					bQuat = Slerp((-1.0 / 3.0), p3Quat, Slerp(0.5, Double(p2Quat, p3Quat), p4Quat));
				}
				else if (nextKeyFrame > frameCount) {
					//last frame
					aQuat = Slerp((1.0 / 3.0), p2Quat, Slerp(0.5, Double(p1Quat, p2Quat), p3Quat));
					bQuat = Slerp((1.0 / 3.0), p3Quat, Double(p1Quat, p2Quat));
				}
				else {
					//in betweens
					Euler2Quaternion(nextPosture->bone_rotation[bone].p, p4Quat);

					aQuat = Slerp((1.0 / 3.0), p2Quat, Slerp(0.5, Double(p1Quat, p2Quat), p3Quat));
					bQuat = Slerp((-1.0 / 3.0), p3Quat, Slerp(0.5, Double(p2Quat, p3Quat), p4Quat));
				}
				Quaternion2Euler(DeCasteljauQuaternion(t, p2Quat, aQuat, bQuat, p3Quat), interpolatedPosture.bone_rotation[bone].p);
			}

			pOutputMotion->SetPosture(startKeyFrame + f, interpolatedPosture);
		}

		startKeyFrame = endKeyFrame;
	}

	for (int f = startKeyFrame + 1; f < frameCount; f++) {
		pOutputMotion->SetPosture(f, *(pInputMotion->GetPosture(f)));
	}
}

void Interpolator::Euler2Quaternion(double angles[3], Quaternion<double> & q) 
{
  // students should implement this
	double R[9];
	Euler2Rotation(angles, R);
	q = Quaternion<double>::Matrix2Quaternion(R);
	q.Normalize();
}

void Interpolator::Quaternion2Euler(Quaternion<double> & q, double angles[3]) 
{
  // students should implement this
	double R[9];
	q.Quaternion2Matrix(R);
	Rotation2Euler(R, angles);
}

Quaternion<double> Interpolator::Slerp(double t, Quaternion<double> & qStart, Quaternion<double> & qEnd_)
{
  // students should implement this
  Quaternion<double> result;
  double cosq = qStart.Gets() * qEnd_.Gets() + qStart.Getx() * qEnd_.Getx() + qStart.Gety() * qEnd_.Gety() + qStart.Getz() * qEnd_.Getz();
  int sign = 1;
  if (cosq < 0) { //<= ?
	sign = -1;
  }
  double a = acos(sign*cosq);
  double sinq = sin(a);
  //error check ?
  //if (sinq == 0 || sinq != sinq)
  //{
	 // return qStart;
  //}

  result = (sin((1 - t) * a) * qStart + sign*sin(t * a) *  qEnd_) / sinq;
  result.Normalize();
  return result;
}

Quaternion<double> Interpolator::Double(Quaternion<double> p, Quaternion<double> q)
{
  // students should implement this
  Quaternion<double> result;
  double cosq = p.Gets() * q.Gets() + p.Getx() * q.Getx() + p.Gety() * q.Gety() + p.Getz() * q.Getz();
  int sign = 1;
  if (cosq < 0) { //<= ?
	  sign = -1;
  }
  double a = acos(sign*cosq);
  double sinq = sin(a);
  //error check ?
  //if (sinq == 0 || sinq != sinq)
  //{
	 // return qStart;
  //}

  result = (sin((1 - 2.0) * a) * p + sign*sin(2.0 * a) *  q) / sinq;
  result.Normalize();
  return result;
}

//whoops I labeled the vectors based on the slides before I saw this func TODO refactor?
vector Interpolator::DeCasteljauEuler(double t, vector p0, vector p1, vector p2, vector p3)
{
  // students should implement this
  vector result, q0, q1, q2;

  q0 = p0 * (1 - t) + p1 * t;
  q1 = p1 * (1 - t) + p2 * t;
  q2 = p2 * (1 - t) + p3 * t;
  result = (q0 * (1 - t) + q1 * t) * (1 - t) + (q1 * (1 - t) + q2 * t) * t;
  
  return result;
}

Quaternion<double> Interpolator::DeCasteljauQuaternion(double t, Quaternion<double> p0, Quaternion<double> p1, Quaternion<double> p2, Quaternion<double> p3)
{
  // students should implement this
  Quaternion<double> result, q0, q1, q2;

  q0 = Slerp(t, p0, p1);
  q1 = Slerp(t, p1, p2);
  q2 = Slerp(t, p2, p3);
  result = Slerp(t, Slerp(t, q0, q1), Slerp(t, q1, q2));

  return result;
}

