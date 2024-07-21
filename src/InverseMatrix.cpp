#include "InverseMatrix.h"

template <typename T>
void swap(T &a, T &b)
{
    T temp = a;
    a = b;
    b = temp;
}
const int N = 6;

void inverseMatrix(double mat[6][6], double mat_inv[6][6])
{
    // define identity matrix.
    double identity[6][6];
    for (int i = 0; i < 6; ++i)
    {
        for (int j = 0; j < 6; ++j)
        {
            identity[i][j] = (i == j) ? 1.0 : 0.0;
        }
    }

    // Gaussian elimination
    for (int i = 0; i < 6; ++i)
    {
        // Find the non-zero element in column i within the rows from i to N.
        int pivot = i;
        while (mat[pivot][i] == 0 && pivot < N)
        {
            pivot++;
        }
        if (mat[pivot][i] == 0)
        {                 // Kiểm tra ma trận không khả nghịch
            //return false; // Ma trận không khả nghịch
        }
        // Swap row i and pivot if pivot is not equal to i
        if (pivot != i)
        {
            for (int j = 0; j < N; ++j)
            {
                swap(mat[i][j], mat[pivot][j]);
                swap(identity[i][j], identity[pivot][j]);
            }
        }
        // Transform row d so that the diagonal element becomes 1
        float divisor = mat[i][i];
        for (int j = 0; j < N; ++j)
        {
            mat[i][j] /= divisor;
            identity[i][j] /= divisor;
        }
        // Eliminate non-zero elements in column i in other rows.
        for (int k = 0; k < N; ++k)
        {
            if (k != i)
            {
                float multiplier = mat[k][i];
                for (int j = 0; j < N; ++j)
                {
                    mat[k][j] -= multiplier * mat[i][j];
                    identity[k][j] -= multiplier * identity[i][j];
                }
            }
        }
    }

    // Copy the inverse matrix into the original matrix.
    for (int i = 0; i < N; ++i)
    {
        for (int j = 0; j < N; ++j)
        {
            mat_inv[i][j] = identity[i][j];
        }
    }
}

void inverseMatrix_3(float mat[3][3], float mat_inv[3][3])
{
    // define identity matrix.
    float identity[3][3];
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            identity[i][j] = (i == j) ? 1.0 : 0.0;
        }
    }

    // Gaussian elimination
    for (int i = 0; i < 3; ++i)
    {
        // Find the non-zero element in column i within the rows from i to 3.
        int pivot = i;
        while (mat[pivot][i] == 0 && pivot < 3)
        {
            pivot++;
        }
        // if (pivot == 3) {
        //     //std::cerr << "Can't inverse Matrix" << std::endl;
        //     return;
        // }
        // Swap row i and pivot if pivot is not equal to i
        if (pivot != i)
        {
            for (int j = 0; j < 3; ++j)
            {
                swap(mat[i][j], mat[pivot][j]);
                swap(identity[i][j], identity[pivot][j]);
            }
        }
        // Transform row d so that the diagonal element becomes 1
        float divisor = mat[i][i];
        for (int j = 0; j < 3; ++j)
        {
            mat[i][j] /= divisor;
            identity[i][j] /= divisor;
        }
        // Eliminate non-zero elements in column i in other rows.
        for (int k = 0; k < 3; ++k)
        {
            if (k != i)
            {
                float multiplier = mat[k][i];
                for (int j = 0; j < 3; ++j)
                {
                    mat[k][j] -= multiplier * mat[i][j];
                    identity[k][j] -= multiplier * identity[i][j];
                }
            }
        }
    }

    // Copy the inverse matrix into the original matrix.
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            mat_inv[i][j] = identity[i][j];
        }
    }
}

