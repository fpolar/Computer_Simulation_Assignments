/*

  USC/Viterbi/Computer Science
  "Jello Cube" Assignment 1 starter code
  Frank Pol - 4727489361
*/

#include "jello.h"
#include "physics.h"
#include <vector>


/* Computes force on a spring with Hook's linear model of elasticity and linear damping
   Returns result in pointer to point f. */
void computeSpringForce(point *L, point *vL, point *f, double R, double k, double kd) {
	double len, dampDot;
	point fh = { 0, 0, 0 }, fd = { 0, 0, 0 };
	pDISTANCE(fh, *L, len);
	if (len == 0) return;

	if (L->x != 0 && L->y != 0 && L->z != 0) {
		int xyz = 0;
	}

	memset(f, 0, sizeof(point));
	//Hook's
	pMULTIPLY(*L, -k * (len - R) / len, fh);
	//linear damping
	pDOT(*vL, *L, dampDot);
	pMULTIPLY(*L, -kd * dampDot / len / len, fd);
	pSUM(fh, fd, *f);
}

/* Computes acceleration to every control point of the jello cube, 
   which is in state given by 'jello'.
   Returns result in array 'a'. */
void computeAcceleration(struct world * jello, struct point a[8][8][8])
{
  /* for you to implement ... */

	point p, v; // current point and velocity from the jello world passed in
	point *currOutPoint; // pointer to current point in a

	//all vector and forces init
	point L = { 0, 0, 0 }, vL = { 0, 0, 0 }, f = { 0, 0, 0 }, structuralForce = { 0, 0, 0 }, shearForce = { 0, 0, 0 }, bendForce = { 0, 0, 0 }, collisionForce = { 0, 0, 0 };
	double R = 0;

	//force field variables init
	point pWeights = {0,0,0}, pFloor, fieldForce, forceFieldVal[8], interpolationPoints[6], interpTemp, pOffset = { -.5, -.5, -.5 };

	for (int i = 0; i < 8; i++) {
		for (int j = 0; j < 8; j++) {
			for (int k = 0; k < 8; k++) {

				a[i][j][k] = { 0,0,0 };

				p = jello->p[i][j][k];
				v = jello->v[i][j][k];
				currOutPoint = &(a[i][j][k]);

				memset(currOutPoint, 0, sizeof(point));
				memset(&structuralForce, 0, sizeof(point));
				memset(&shearForce, 0, sizeof(point));
				memset(&bendForce, 0, sizeof(point));
				memset(&collisionForce, 0, sizeof(point));
				memset(&fieldForce, 0, sizeof(point));

				//computing forces from neighbors 
				for (int l = -2; l <= 2; l++) {
					for (int m = -2; m <= 2 ; m++) {
						for (int n = -2; n <= 2; n++)
						{
							//do not compute forces with itself or anything out of bounds
							if ((l == 0 && m == 0 && n == 0) || (i + l < 0 || i + l > 7 || j + m < 0 || j + m > 7 || k + n < 0 || k + n > 7)) continue;

							if (abs(l) < 2 && abs(m) < 2 && abs(n) < 2) {
								pDIFFERENCE(jello->v[i][j][k], jello->v[i + l][j + m][k + n], vL);
								pDIFFERENCE(jello->p[i][j][k], jello->p[i + l][j + m][k + n], L);
								if ((abs(l) == 1 && m == 0 && n == 0) || (abs(n) == 1 && l == 0 && m == 0) || (abs(m) == 1 && n == 0 && l == 0)) {
									//^up to 6 cases for structural springs
									computeSpringForce(&L, &vL, &f, 1.0 / 7, jello->kElastic, jello->dElastic);
									pSUM(structuralForce, f, structuralForce);
								}
								else {
									//case for shear springs:
									//calculate R value based on relative positions of verts in the discrete cube being compared
									//min = 0, max = ~1.732 (should be) 
									R = sqrt(abs(l) + abs(m) + abs(n));
									computeSpringForce(&L, &vL, &f, 1.0 / 7*R, jello->kElastic, jello->dElastic);
									pSUM(shearForce, f, shearForce);
								}
							}
							else if ((abs(l) == 2 && m == 0 && n == 0) || (abs(n) == 2 && l == 0 && m == 0) || (abs(m) == 2 && n == 0 && l == 0)) {
								//^up to 6 cases for bend springs
								pDIFFERENCE(jello->v[i][j][k], jello->v[i + l][j + m][k + n], vL);
								pDIFFERENCE(jello->p[i][j][k], jello->p[i + l][j + m][k + n], L);
								computeSpringForce(&L, &vL, &f, 1.0 / 7 * 2, jello->kElastic, jello->dElastic);
								pSUM(bendForce, f, bendForce);
							}

						}
					}
				}

				//check for and compute collision forces
				L = { 0, 0, 0 }, vL = v, f = { 0, 0, 0 };
				if (p.x <= -2) {
					L.x = p.x + 2;
				}
				if (p.x >= 2) {
					L.x = p.x - 2;
				}
				if (p.y <= -2) {
					L.y = p.y + 2;
				}
				if (p.y >= 2) {
					L.y = p.y - 2;
				}
				if (p.z <= -2) {
					L.z = p.z + 2;
				}
				if (p.z >= 2) {
					L.z = p.z - 2;
				}
				computeSpringForce(&L, &vL, &f, 0, jello->kCollision, jello->dCollision);
				pSUM(collisionForce, f, collisionForce);

				//reading and calculating force field forces

				//Since the we are restricted to the bounding box, the positions can be from -4 to 4
				//to change this range into 0 to 1 (for the interpolation weights), we divide by 4 (x0.25) and add .5
				pMULTIPLY(p, 0.25, pWeights);
				pDIFFERENCE(pWeights, pOffset, pWeights);
				pCLAMP(pWeights);

				//scale p from percent weights to force field resolution
				pMULTIPLY(pWeights, jello->resolution - 1, pWeights);

				//floor weights to get indices for force field array
				pFloor.x = floor(pWeights.x);
				pFloor.y = floor(pWeights.y);
				pFloor.z = floor(pWeights.z);
				
				forceFieldVal[0] = jello->forceField[int(pFloor.x*jello->resolution*jello->resolution + pFloor.y * jello->resolution + pFloor.z)];
				forceFieldVal[1] = jello->forceField[int(pFloor.x*jello->resolution*jello->resolution + pFloor.y * jello->resolution + pFloor.z+1)];
				forceFieldVal[2] = jello->forceField[int(pFloor.x*jello->resolution*jello->resolution + (pFloor.y+1) * jello->resolution + pFloor.z)];
				forceFieldVal[3] = jello->forceField[int(pFloor.x*jello->resolution*jello->resolution + (pFloor.y+1) * jello->resolution + pFloor.z+1)];
				forceFieldVal[4] = jello->forceField[int((pFloor.x + 1)*jello->resolution*jello->resolution + pFloor.y * jello->resolution + pFloor.z)];
				forceFieldVal[5] = jello->forceField[int((pFloor.x + 1)*jello->resolution*jello->resolution + pFloor.y * jello->resolution + pFloor.z+1)];
				forceFieldVal[6] = jello->forceField[int((pFloor.x + 1)*jello->resolution*jello->resolution + (pFloor.y + 1) * jello->resolution + pFloor.z)];
				forceFieldVal[7] = jello->forceField[int((pFloor.x + 1)*jello->resolution*jello->resolution + (pFloor.y + 1) * jello->resolution + pFloor.z+1)];

				//return the weights to range between 0 and 1
				pDIFFERENCE(pWeights, pFloor, pWeights);
				pINTERPOLATE(forceFieldVal[0], forceFieldVal[1], pWeights.z, interpolationPoints[0], interpTemp);
				pINTERPOLATE(forceFieldVal[2], forceFieldVal[3], pWeights.z, interpolationPoints[1], interpTemp);
				pINTERPOLATE(forceFieldVal[4], forceFieldVal[5], pWeights.z, interpolationPoints[2], interpTemp);
				pINTERPOLATE(forceFieldVal[6], forceFieldVal[7], pWeights.z, interpolationPoints[3], interpTemp);
				pINTERPOLATE(interpolationPoints[0], interpolationPoints[1], pWeights.y, interpolationPoints[4], interpTemp);
				pINTERPOLATE(interpolationPoints[2], interpolationPoints[3], pWeights.y, interpolationPoints[5], interpTemp);
				pINTERPOLATE(interpolationPoints[4], interpolationPoints[5], pWeights.x, fieldForce, interpTemp);

				//summing all calculated forces for jello point
				pSUM(*currOutPoint, structuralForce, *currOutPoint);
				pSUM(*currOutPoint, shearForce, *currOutPoint);
				pSUM(*currOutPoint, bendForce, *currOutPoint);
				pSUM(*currOutPoint, collisionForce, *currOutPoint);
				pSUM(*currOutPoint, fieldForce, *currOutPoint);

				pMULTIPLY(a[i][j][k], (1 / jello->mass), a[i][j][k]);
			}
		}
	}
}

/* performs one step of Euler Integration */
/* as a result, updates the jello structure */
void Euler(struct world * jello)
{
  int i,j,k;
  point a[8][8][8];

  computeAcceleration(jello, a);
  
  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
        jello->p[i][j][k].x += jello->dt * jello->v[i][j][k].x;
        jello->p[i][j][k].y += jello->dt * jello->v[i][j][k].y;
        jello->p[i][j][k].z += jello->dt * jello->v[i][j][k].z;
        jello->v[i][j][k].x += jello->dt * a[i][j][k].x;
        jello->v[i][j][k].y += jello->dt * a[i][j][k].y;
        jello->v[i][j][k].z += jello->dt * a[i][j][k].z;

      }
}

/* performs one step of RK4 Integration */
/* as a result, updates the jello structure */
void RK4(struct world * jello)
{
  point F1p[8][8][8], F1v[8][8][8], 
        F2p[8][8][8], F2v[8][8][8],
        F3p[8][8][8], F3v[8][8][8],
        F4p[8][8][8], F4v[8][8][8];

  point a[8][8][8];


  struct world buffer;

  int i,j,k;

  buffer = *jello; // make a copy of jello

  computeAcceleration(jello, a);

  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
         pMULTIPLY(jello->v[i][j][k],jello->dt,F1p[i][j][k]);
         pMULTIPLY(a[i][j][k],jello->dt,F1v[i][j][k]);
         pMULTIPLY(F1p[i][j][k],0.5,buffer.p[i][j][k]);
         pMULTIPLY(F1v[i][j][k],0.5,buffer.v[i][j][k]);
         pSUM(jello->p[i][j][k],buffer.p[i][j][k],buffer.p[i][j][k]);
         pSUM(jello->v[i][j][k],buffer.v[i][j][k],buffer.v[i][j][k]);
      }

  computeAcceleration(&buffer, a);

  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
         // F2p = dt * buffer.v;
         pMULTIPLY(buffer.v[i][j][k],jello->dt,F2p[i][j][k]);
         // F2v = dt * a(buffer.p,buffer.v);     
         pMULTIPLY(a[i][j][k],jello->dt,F2v[i][j][k]);
         pMULTIPLY(F2p[i][j][k],0.5,buffer.p[i][j][k]);
         pMULTIPLY(F2v[i][j][k],0.5,buffer.v[i][j][k]);
         pSUM(jello->p[i][j][k],buffer.p[i][j][k],buffer.p[i][j][k]);
         pSUM(jello->v[i][j][k],buffer.v[i][j][k],buffer.v[i][j][k]);
      }

  computeAcceleration(&buffer, a);

  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
         // F3p = dt * buffer.v;
         pMULTIPLY(buffer.v[i][j][k],jello->dt,F3p[i][j][k]);
         // F3v = dt * a(buffer.p,buffer.v);     
         pMULTIPLY(a[i][j][k],jello->dt,F3v[i][j][k]);
         pMULTIPLY(F3p[i][j][k],1.0,buffer.p[i][j][k]);
         pMULTIPLY(F3v[i][j][k],1.0,buffer.v[i][j][k]);
         pSUM(jello->p[i][j][k],buffer.p[i][j][k],buffer.p[i][j][k]);
         pSUM(jello->v[i][j][k],buffer.v[i][j][k],buffer.v[i][j][k]);
      }
         
  computeAcceleration(&buffer, a);


  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
         // F3p = dt * buffer.v;
         pMULTIPLY(buffer.v[i][j][k],jello->dt,F4p[i][j][k]);
         // F3v = dt * a(buffer.p,buffer.v);     
         pMULTIPLY(a[i][j][k],jello->dt,F4v[i][j][k]);

         pMULTIPLY(F2p[i][j][k],2,buffer.p[i][j][k]);
         pMULTIPLY(F3p[i][j][k],2,buffer.v[i][j][k]);
         pSUM(buffer.p[i][j][k],buffer.v[i][j][k],buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],F1p[i][j][k],buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],F4p[i][j][k],buffer.p[i][j][k]);
         pMULTIPLY(buffer.p[i][j][k],1.0 / 6,buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],jello->p[i][j][k],jello->p[i][j][k]);

         pMULTIPLY(F2v[i][j][k],2,buffer.p[i][j][k]);
         pMULTIPLY(F3v[i][j][k],2,buffer.v[i][j][k]);
         pSUM(buffer.p[i][j][k],buffer.v[i][j][k],buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],F1v[i][j][k],buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],F4v[i][j][k],buffer.p[i][j][k]);
         pMULTIPLY(buffer.p[i][j][k],1.0 / 6,buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],jello->v[i][j][k],jello->v[i][j][k]);
      }

  return;  
}
