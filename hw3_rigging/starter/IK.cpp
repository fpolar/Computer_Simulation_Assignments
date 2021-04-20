#include "IK.h"
#include "FK.h"
#include "minivectorTemplate.h"
#include <Eigen/Dense>
#include <adolc/adolc.h>
#include <cassert>
#if defined(_WIN32) || defined(WIN32)
  #ifndef _USE_MATH_DEFINES
    #define _USE_MATH_DEFINES
  #endif
#endif
#include <math.h>
using namespace std;

// CSCI 520 Computer Animation and Simulation
// Jernej Barbic and Yijing Li

namespace
{

// Converts degrees to radians.
template<typename real>
inline real deg2rad(real deg) { return deg * M_PI / 180.0; }

template<typename real>
Mat3<real> Euler2Rotation(const real angle[3], RotateOrder order)
{
  Mat3<real> RX = Mat3<real>::getElementRotationMatrix(0, deg2rad(angle[0]));
  Mat3<real> RY = Mat3<real>::getElementRotationMatrix(1, deg2rad(angle[1]));
  Mat3<real> RZ = Mat3<real>::getElementRotationMatrix(2, deg2rad(angle[2]));

  switch(order)
  {
    case RotateOrder::XYZ:
      return RZ * RY * RX;
    case RotateOrder::YZX:
      return RX * RZ * RY;
    case RotateOrder::ZXY:
      return RY * RX * RZ;
    case RotateOrder::XZY:
      return RY * RZ * RX;
    case RotateOrder::YXZ:
      return RZ * RX * RY;
    case RotateOrder::ZYX:
      return RX * RY * RZ;
  }
  assert(0);
}

// Performs forward kinematics, using the provided "fk" class.
// This is the function whose Jacobian matrix will be computed using adolc.
// numIKJoints and IKJointIDs specify which joints serve as handles for IK:
//   IKJointIDs is an array of integers of length "numIKJoints"
// Input: numIKJoints, IKJointIDs, fk, eulerAngles (of all joints)
// Output: handlePositions (world-coordinate positions of all the IK joints; length is 3 * numIKJoints)
template<typename real>
void forwardKinematicsFunction(
    int numIKJoints, const int * IKJointIDs, const FK & fk,
    const std::vector<real> & eulerAngles, std::vector<real> & handlePositions)
{
  // Students should implement this.
  // The implementation of this function is very similar to function computeLocalAndGlobalTransforms in the FK class.
  // The recommended approach is to first implement FK::computeLocalAndGlobalTransforms.
  // Then, implement the same algorithm into this function. To do so,
  // you can use fk.getJointUpdateOrder(), fk.getJointRestTranslation(), and fk.getJointRotateOrder() functions.
  // Also useful is the multiplyAffineTransform4ds function in minivectorTemplate.h .
  // It would be in principle possible to unify this "forwardKinematicsFunction" and FK::computeLocalAndGlobalTransforms(),
  // so that code is only written once. We considered this; but it is actually not easily doable.
  // If you find a good approach, feel free to document it in the README file, for extra credit.
	real euler[3], orient[3];	
	int numJoints = fk.getNumJoints();
	vector<Mat3<real>> localTransform(numJoints), globalTransform(numJoints);
	vector<Vec3<real>> localTranslate(numJoints), globalTranslate(numJoints);

	for (int i = 0; i < fk.getNumJoints(); ++i) {
		// Calculate rotations
		euler[0] = eulerAngles[i * 3];
		euler[1] = eulerAngles[i * 3 + 1];
		euler[2] = eulerAngles[i * 3 + 2];

		// Calculate localJointTransform
		Mat3<real> RMMatrix = Euler2Rotation(euler, fk.getJointRotateOrder(i));;
		orient[0] = fk.getJointOrient(i).data()[0];
		orient[1] = fk.getJointOrient(i).data()[1];
		orient[2] = fk.getJointOrient(i).data()[2];

		Mat3<real> ROMatrix = Euler2Rotation(orient, RotateOrder::XYZ);
		localTransform[i] = ROMatrix * RMMatrix;

		// Calculate localJointTranslate and globalJointTranslate
		localTranslate[i][0] = fk.getJointRestTranslation(i)[0];
		localTranslate[i][1] = fk.getJointRestTranslation(i)[1];
		localTranslate[i][2] = fk.getJointRestTranslation(i)[2];

		int jointOrder = fk.getJointUpdateOrder(i);

		globalTranslate[jointOrder][0] = fk.getJointRestTranslation(jointOrder)[0];
		globalTranslate[jointOrder][1] = fk.getJointRestTranslation(jointOrder)[1];
		globalTranslate[jointOrder][2] = fk.getJointRestTranslation(jointOrder)[2];

		// If joint is root
		if (fk.getJointParent(jointOrder) == -1) {
			globalTransform[jointOrder] = localTransform[jointOrder];
		}else {
			multiplyAffineTransform4ds(
				globalTransform[fk.getJointParent(jointOrder)],
				globalTranslate[fk.getJointParent(jointOrder)],
				localTransform[jointOrder],
				localTranslate[jointOrder],
				globalTransform[jointOrder],
				globalTranslate[jointOrder]
			);
		}
	}
	// output handle positions
	for (int i = 0; i < numIKJoints; ++i) {
		handlePositions[i * 3] = globalTranslate[IKJointIDs[i]][0];
		handlePositions[i * 3 + 1] = globalTranslate[IKJointIDs[i]][1];
		handlePositions[i * 3 + 2] = globalTranslate[IKJointIDs[i]][2];
	}
}

} // end anonymous namespaces

IK::IK(int numIKJoints, const int * IKJointIDs, FK * inputFK, int adolc_tagID)
{
  this->numIKJoints = numIKJoints;
  this->IKJointIDs = IKJointIDs;
  this->fk = inputFK;
  this->adolc_tagID = adolc_tagID;

  FKInputDim = fk->getNumJoints() * 3;
  FKOutputDim = numIKJoints * 3;

  train_adolc();
}

void IK::train_adolc()
{
  // Students should implement this.
  // Here, you should setup adol_c:
  //   Define adol_c inputs and outputs. 
  //   Use the "forwardKinematicsFunction" as the function that will be computed by adol_c.
  //   This will later make it possible for you to compute the gradient of this function in IK::doIK
  //   (in other words, compute the "Jacobian matrix" J).
  // See ADOLCExample.cpp .

	int m = FKOutputDim;
	int n = FKInputDim;

	trace_on(adolc_tagID);

	//init input
	vector<adouble> eulerAngles(n);
	for (int i = 0; i < n; ++i) {
		eulerAngles[i] <<= 0.0;
	}

	// Compute output, handleVertexPosition
	vector<adouble> handleVertexPosition(m);
	forwardKinematicsFunction(numIKJoints, IKJointIDs, *fk, eulerAngles, handleVertexPosition);

	vector<double> result(m);
	for (int i = 0; i < m; ++i) {
		handleVertexPosition[i] >>= result[i];
	}

	trace_off();
}

void IK::doIK(const Vec3d * targetHandlePositions, Vec3d * jointEulerAngles)
{
  // You may find the following helpful:
  int numJoints = fk->getNumJoints(); // Note that is NOT the same as numIKJoints!

  // Students should implement this.
  // Use adolc to evalute the forwardKinematicsFunction and its gradient (Jacobian). It was trained in train_adolc().
  // Specifically, use ::function, and ::jacobian .
  // See ADOLCExample.cpp .
  //
  // Use it implement the Tikhonov IK method (or the pseudoinverse method for extra credit).
  // Note that at entry, "jointEulerAngles" contains the input Euler angles. 
  // Upon exit, jointEulerAngles should contain the new Euler angles.
  double alpha = 0.001;
  vector<double> handleVertexPosition(FKOutputDim);
  ::function(adolc_tagID, FKOutputDim, FKInputDim, jointEulerAngles->data(), handleVertexPosition.data());

  // Calculate Jacobian Matrix
  vector<double> jacobianMatrix(FKInputDim * FKOutputDim);
  vector<double*> jacobianMatrixRows(FKOutputDim);

  for (int i = 0; i < FKOutputDim; i++) {
	  jacobianMatrixRows[i] = &jacobianMatrix[i * FKInputDim];
  }
  ::jacobian(adolc_tagID, FKOutputDim, FKInputDim, jointEulerAngles->data(), jacobianMatrixRows.data());

  Eigen::MatrixXd eigenJacobian(FKOutputDim, FKInputDim), eigenJacobianTranspose, identityMatrix;

  // Calculate Jacobian matrix
  for (int i = 0; i < FKOutputDim; i++) {
	  for (int j = 0; j < FKInputDim; j++) {
		  eigenJacobian(i, j) = jacobianMatrix[i * FKInputDim + j];
	  }
  }

  // Calculate Jacobian transpose matrix
  eigenJacobianTranspose = eigenJacobian.transpose();

  // Get the identity Matrix
  identityMatrix = identityMatrix.Identity(FKInputDim, FKInputDim);

  // Calculate delta B
  vector<double> deltaB(FKOutputDim);
  Eigen::VectorXd eigenDeltaB(FKOutputDim);

  for (int i = 0; i < FKOutputDim; i++) {
	  deltaB[i] = targetHandlePositions->data()[i] - handleVertexPosition.data()[i];
	  eigenDeltaB[i] = deltaB[i];
  }

  // Solve: (J^T * J + alpha * I) * deltaTheta = J^T * deltaB
  // Solve for LHS and RHS separately and then combine to get delta theta
  Eigen::MatrixXd lhs(FKInputDim, FKInputDim);
  lhs = eigenJacobianTranspose * eigenJacobian + alpha * identityMatrix;

  Eigen::VectorXd rhs(FKOutputDim);
  rhs = eigenJacobianTranspose * eigenDeltaB;

  // Find delta theta
  Eigen::VectorXd eigenDeltaTheta = lhs.ldlt().solve(rhs);
  vector<double> deltaTheta(FKInputDim);

  for (int i = 0; i < FKInputDim; i++) {
	  deltaTheta[i] = eigenDeltaTheta[i];
  }

  // Update angles with delta theta
  for (int i = 0; i < numJoints; i++) {
	  jointEulerAngles[i][0] = jointEulerAngles[i][0] + deltaTheta[i * 3];
	  jointEulerAngles[i][1] = jointEulerAngles[i][1] + deltaTheta[i * 3 + 1];
	  jointEulerAngles[i][2] = jointEulerAngles[i][2] + deltaTheta[i * 3 + 2];
  }
}

