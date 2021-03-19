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



//	LinearInterpolation
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
		for (int frame = 1; frame <= N; frame++)
		{
			Posture interpolatedPosture;
			double t = 1.0 * frame / (N + 1);

			// interpolate root position
			interpolatedPosture.root_pos = startPosture->root_pos * (1 - t) + endPosture->root_pos * t;

			// interpolate bone rotations
			for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
				interpolatedPosture.bone_rotation[bone] = startPosture->bone_rotation[bone] * (1 - t) + endPosture->bone_rotation[bone] * t;

			pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
		}

		startKeyframe = endKeyframe;
	}

	for (int frame = startKeyframe + 1; frame < inputLength; frame++)
		pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::Rotation2Euler(double R[9], double angles[3])
{
	double cy = sqrt(R[0] * R[0] + R[3] * R[3]);

	if (cy > 16 * DBL_EPSILON)
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

	for (int i = 0; i < 3; i++)
		angles[i] *= 180 / M_PI;
}

void Interpolator::Euler2Rotation(double angles[3], double R[9])
{
	//converting degrees to radians then to rotation matrices
	double radians[3] = { (double)(angles[0] / 180) * M_PI, (double)(angles[1] / 180) * M_PI, (double)(angles[2] / 180) * M_PI};
	double rx[9] = {
		1.0, 0.0, 0.0,
		0.0, cos(radians[0]), -sin(radians[0]),
		0.0, sin(radians[0]), cos(radians[0])
	};

	double ry[9] = {
		cos(radians[1]), 0.0, sin(radians[1]),
		0.0, 1.0, 0.0,
		-sin(radians[1]), 0.0, cos(radians[1])
	};

	double rz[9] = {
		cos(radians[2]), -sin(radians[2]), 0.0,
		sin(radians[2]), cos(radians[2]), 0.0,
		0.0, 0.0, 1.0
	};

	double rz_ry[9];

	//multiplying rotation matrices to get 1 total rotation matrix
	MatrixMultiply(rz, ry, rz_ry);
	MatrixMultiply(rz_ry, rx, R);
}

void Interpolator::BezierInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
	int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

	int startKeyframe = 0;

	vector p0, p1, p2, p3, a, b;

	while (startKeyframe + N + 1 < inputLength)
	{
		int endKeyframe = startKeyframe + N + 1;
		int preKeyframe = startKeyframe - N - 1;
		int nextKeyframe = endKeyframe + N + 1;


		// copy start and end keyframe
		Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
		Posture * endPosture = pInputMotion->GetPosture(endKeyframe);
		pOutputMotion->SetPosture(startKeyframe, *startPosture);
		pOutputMotion->SetPosture(endKeyframe, *endPosture);

		// interpolate in between
		Posture * prePosture;
		Posture * nextPosture;
		for (int frame = 1; frame <= N; frame++)
		{
			Posture interpolatedPosture;
			double t = 1.0 * frame / (N + 1);

			p1 = startPosture->root_pos;
			p2 = endPosture->root_pos;

			// interpolate root position
			if (startKeyframe == 0) {
				//first frame, no previous posture
				nextPosture = pInputMotion->GetPosture(nextKeyframe);
				p3 = nextPosture->root_pos;
				a = p1 + ((p2 + p2 - p3) - p1) / 3;
				b = p2 - (((p2 + p2 - p1) + p3) / 2 - p2) / 3;
			}
			else {
				prePosture = pInputMotion->GetPosture(preKeyframe);
				p0 = prePosture->root_pos;
				if (nextKeyframe < inputLength)
				{
					nextPosture = pInputMotion->GetPosture(nextKeyframe);
					p3 = nextPosture->root_pos;
					a = p1 + (((p1 + p1 - p0) + p2) / 2 - p1) / 3;
					b = p2 - (((p2 + p2 - p1) + p3) / 2 - p2) / 3;
				}
				else
				{
					//last frame, no next posture
					nextPosture = prePosture;
					a = p1 + (((p1 + p1 - p0) + p2) / 2 - p1) / 3;
					b = p2 + ((p1 + p1 - p0) - p2) / 3;
				}
			}

			interpolatedPosture.root_pos = DeCasteljauEuler(t, p1, a, b, p2);

			// interpolate bone rotations
			for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
			{

				p1 = startPosture->bone_rotation[bone];
				p2 = endPosture->bone_rotation[bone];

				if (startKeyframe == 0) {
					p3 = nextPosture->bone_rotation[bone];
					a = p1 + ((p2 + p2 - p3) - p1) / 3;
					b = p2 - (((p2 + p2 - p1) + p3) / 2 - p2) / 3;
				}
				else {
					p0 = prePosture->bone_rotation[bone];
					if (nextKeyframe <= inputLength)
					{
						p3 = nextPosture->bone_rotation[bone];
						a = p1 + (((p1 + p1 - p0) + p2) / 2 - p1) / 3;
						b = p2 - (((p2 + p2 - p1) + p3) / 2 - p2) / 3;
					}
					else {
						a = p1 + (((p1 + p1 - p0) + p2) / 2 - p1) / 3;
						b = p2 + ((p1 + p1 - p0) - p2) / 3;
					}
				}

				interpolatedPosture.bone_rotation[bone] = DeCasteljauEuler(t, p1, a, b, p2);
			}

			pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
		}
		startKeyframe = endKeyframe;
	}
	for (int frame = startKeyframe + 1; frame < inputLength; frame++)
		pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::LinearInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
	int inputLength = pInputMotion->GetNumFrames();

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
		for (int frame = 1; frame <= N; frame++)
		{
			Posture interpolatedPosture;
			double t = 1.0 * frame / (N + 1);

			// interpolate root position
			interpolatedPosture.root_pos = startPosture->root_pos * (1 - t) + endPosture->root_pos * t;

			// interpolate bone rotations
			Quaternion<double> qStart, qEnd;
			for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
			{
				Euler2Quaternion(startPosture->bone_rotation[bone].p, qStart);
				Euler2Quaternion(endPosture->bone_rotation[bone].p, qEnd);
				Quaternion2Euler(Slerp(t, qStart, qEnd), interpolatedPosture.bone_rotation[bone].p);
			}
			pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
		}

		startKeyframe = endKeyframe;
	}

	for (int frame = startKeyframe + 1; frame < inputLength; frame++) {
		pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
	}
}

void Interpolator::BezierInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
	int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

	int startKeyframe = 0;

	vector p0, p1, p2, p3, a, b;
	Quaternion<double> p0Quat,p1Quat,p2Quat,p3Quat, aQuat, bQuat;

	while (startKeyframe + N + 1 < inputLength)
	{
		int endKeyframe = startKeyframe + N + 1;
		int preKeyframe = startKeyframe - N - 1;
		int nextKeyframe = endKeyframe + N + 1;

		// copy start and end keyframe
		Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
		Posture * endPosture = pInputMotion->GetPosture(endKeyframe);
		pOutputMotion->SetPosture(startKeyframe, *startPosture);
		pOutputMotion->SetPosture(endKeyframe, *endPosture);

		// interpolate in between
		Posture * prePosture;
		Posture * nextPosture;
		for (int frame = 1; frame <= N; frame++)
		{
			Posture interpolatedPosture;
			double t = 1.0 * frame / (N + 1);

			p1 = startPosture->root_pos;
			p2 = endPosture->root_pos;

			// interpolate root position
			if (startKeyframe == 0)
			{
				//First frame, no previous posture so set to start
				prePosture = pInputMotion->GetPosture(startKeyframe);
				nextPosture = pInputMotion->GetPosture(nextKeyframe);
				p3 = nextPosture->root_pos;
				a = p1 + ((p2 + p2 - p3) - p1) / 3;
				b = p2 - (((p2 + p2 - p1) + p3) / 2 - p2) / 3;
			}else{
				prePosture = pInputMotion->GetPosture(preKeyframe);
				p0 = prePosture->root_pos;

				if (nextKeyframe < inputLength)
				{
					nextPosture = pInputMotion->GetPosture(nextKeyframe);
					p3 = nextPosture->root_pos;
					a = p1 + (((p1 + p1 - p0) + p2) / 2 - p1) / 3;
					b = p2 - (((p2 + p2 - p1) + p3) / 2 - p2) / 3;
				}
				else
				{
					//Last frame, no next posture so set to previous posture
					nextPosture = prePosture;
					a = p1 + (((p1 + p1 - p0) + p2) / 2 - p1) / 3;
					b = p2 + ((p1 + p1 - p0) - p2) / 3;
				}
			}
			
			interpolatedPosture.root_pos = DeCasteljauEuler(t, p1, a, b, p2);

			// interpolate bone rotations
			for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
			{
				Euler2Quaternion(startPosture->bone_rotation[bone].p, p1Quat);
				Euler2Quaternion(endPosture->bone_rotation[bone].p, p2Quat);

				if (startKeyframe == 0)
				{
					//First frame, no previous posture so set to start
					Euler2Quaternion(prePosture->bone_rotation[bone].p, p0Quat);
					if (nextKeyframe <= inputLength)
					{
						Euler2Quaternion(nextPosture->bone_rotation[bone].p, p3Quat);

						aQuat = Slerp((1.0 / 3.0), p1Quat, Slerp(0.5, Double(p0Quat, p1Quat), p2Quat));
						bQuat = Slerp((-1.0 / 3.0), p2Quat, Slerp(0.5, Double(p1Quat, p2Quat), p3Quat));
					}
					else
					{
						aQuat = Slerp((1.0 / 3.0), p1Quat, Slerp(0.5, Double(p0Quat, p1Quat), p2Quat));
						bQuat = Slerp((1.0 / 3.0), p2Quat, Double(p0Quat, p1Quat));
					}
				}
				else
				{
					Euler2Quaternion(nextPosture->bone_rotation[bone].p, p3Quat);

					aQuat = Slerp((1.0 / 3.0), p1Quat, Double(p3Quat, p2Quat));
					bQuat = Slerp((-1.0 / 3.0), p2Quat, Slerp(0.5, Double(p1Quat, p2Quat), p3Quat));
				}
				Quaternion2Euler(DeCasteljauQuaternion(t, p1Quat, aQuat, bQuat, p2Quat), interpolatedPosture.bone_rotation[bone].p);
			}

			pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
		}

		startKeyframe = endKeyframe;
	}

	for (int frame = startKeyframe + 1; frame < inputLength; frame++)
		pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::Euler2Quaternion(double angles[3], Quaternion<double> & q)
{
	double R[9];
	Euler2Rotation(angles, R);
	q = Quaternion<double>::Matrix2Quaternion(R);
	q.Normalize();
}

void Interpolator::Quaternion2Euler(Quaternion<double> & q, double angles[3])
{
	double R[9];
	q.Quaternion2Matrix(R);
	Rotation2Euler(R, angles);
}

//	Quaternion operations
Quaternion<double> Interpolator::Slerp(double t, Quaternion<double> & qStart, Quaternion<double> & qEnd)
{
	Quaternion<double> result;

	double cosq = qStart.Gets() * qEnd.Gets() + qStart.Getx() * qEnd.Getx() + qStart.Gety() * qEnd.Gety() + qStart.Getz() * qEnd.Getz();
	double a, sinq;
	if (cosq > 0)
	{
		a = acos(cosq);
		sinq = sin(a);
		if (sinq == 0 || sinq != sinq)//error check
		{
			return qStart;
		}
		result = (sin((1 - t) * a) * qStart + sin(t * a) * qEnd) / sinq;
		result.Normalize();
		return result;
	}
	else
	{
		a = acos(-cosq);
		sinq = sin(a);
		if (sinq == 0 || sinq != sinq)//error check
		{
			return qStart;
		}
		result = (sin((1 - t) * a) * qStart + -sin(t * a) *  qEnd) / sinq;
		result.Normalize();
		return result;
	}
}

Quaternion<double> Interpolator::Double(Quaternion<double> p, Quaternion<double> q)
{
	Quaternion<double> result;

	double cosq = p.Gets() * q.Gets() + p.Getx() * q.Getx() + p.Gety() * q.Gety() + p.Getz() * q.Getz();
	double a, sinq;
	if (cosq > 0)
	{
		a = acos(cosq);
		sinq = sin(a);
		if (sinq == 0 || sinq != sinq)//error check
		{
			return p;
		}
		result = (sin((1 - 2.0) * a) * p + sin(2.0 * a) * q) / sinq;
		result.Normalize();
		return result;
	}
	else
	{
		a = acos(-cosq);
		sinq = sin(a);
		if (sinq == 0 || sinq != sinq)//error check
		{
			return p;
		}
		result = (sin((1 - 2.0) * a) * p + -sin(2.0 * a) *  q) / sinq;
		result.Normalize();
		return result;
	}
}

vector Interpolator::DeCasteljauEuler(double t, vector p0, vector p1, vector p2, vector p3)
{
	vector q0, q1, q2;

	q0 = p0 * (1 - t) + p1 * t;
	q1 = p1 * (1 - t) + p2 * t;
	q2 = p2 * (1 - t) + p3 * t;

	return (q0 * (1 - t) + q1 * t) * (1 - t) + (q1 * (1 - t) + q2 * t) * t;
}

Quaternion<double> Interpolator::DeCasteljauQuaternion(double t, Quaternion<double> p0, Quaternion<double> p1, Quaternion<double> p2, Quaternion<double> p3)
{
	Quaternion<double> q0, q1, q2;

	q0 = Slerp(t, p0, p1);
	q1 = Slerp(t, p1, p2);
	q2 = Slerp(t, p2, p3);

	return Slerp(t, Slerp(t, q0, q1), Slerp(t, q1, q2));
}
