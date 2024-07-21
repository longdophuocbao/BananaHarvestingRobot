#include "Kinematic.h"

void Forward_Kinemaic(double _Theta1, double _Theta2, double *X_value, double *Y_value)
{
    double L1 = L1_length;
    double L2 = L2_length;
    *X_value = L1 * cos(_Theta1) + L2 * cos(_Theta1 + _Theta2);
    *Y_value = L1 * sin(_Theta1) + L2 * sin(_Theta1 + _Theta2);
}

void Inverse_Kinemaic(double X_value, double Y_value, double *_Theta1, double *_Theta2)
{
    double l1 = L1_length;
    double l2 = L2_length;
    *_Theta2 = acos((pow(X_value, 2) + pow(Y_value, 2) - pow(l1, 2) - pow(l2, 2)) / (2 * l1 * l2));
    *_Theta1 = atan2(Y_value, X_value) - atan2(l2 * sin(*_Theta2), l1 + l2 * cos(*_Theta2));
    // *_Theta1 = atan2((_l1 + _l2 * ((X * X + _Y * _Y - _l1 * _l1 - _l2 * _l2) / (2 * _l1 * _l2))) * _Y - _l2 * sin(*_Theta2) * X, (_l1 + _l2 * ((X * X + _Y * _Y - _l1 * _l1 - _l2 * _l2) / (2 * _l1 * _l2))) * X + _l2 * sin(*_Theta2) * _Y);
}
