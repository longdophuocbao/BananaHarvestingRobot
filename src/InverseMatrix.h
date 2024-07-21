#include <Arduino.h>
#include "Constants.h"

#ifndef INVERSEMATRIX_H
#define INVERSEMATRIX_H

void inverseMatrix(double mat[6][6], double mat_inv[6][6]);
void inverseMatrix_3(float mat[3][3], float mat_inv[3][3]);
#endif