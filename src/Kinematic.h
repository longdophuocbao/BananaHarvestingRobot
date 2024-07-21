#include <Arduino.h>
#include "Constants.h"

#ifndef KINEMATIC_H
#define KINEMATIC_H

void Forward_Kinemaic(double _Theta1, double _Theta2, double *X_value, double *Y_value);
void Inverse_Kinemaic(double X_value, double Y_value, double *_Theta1, double *_Theta2);

struct StructTrajectory
{
private:
  uint16_t Total_Index;
  unsigned int ACC_Time, DEC_Time;
  double **theta;
  double **speed;

public:
  // Constructor
  StructTrajectory(unsigned int totalIndex, unsigned int accTime, unsigned int decTime)
      : Total_Index(totalIndex), ACC_Time(accTime), DEC_Time(decTime)
  {
    uint16_t n = Total_Index + 1;

    theta = new double *[2];
    speed = new double *[2];
    for (int i = 0; i < 2; ++i)
    {
      theta[i] = new double[n];
      speed[i] = new double[n];

      if (!theta[i] || !speed[i])
      { // Kiểm tra cấp phát bộ nhớ
        deallocateMemory();
        // throw std::bad_alloc(); // Ném ngoại lệ nếu cấp phát thất bại
      }

      for (uint16_t j = 0; j <= Total_Index; ++j)
      {
        theta[i][j] = 0.0;
        speed[i][j] = 0.0;
      }
    }
  }

  // Copy constructor
  StructTrajectory(const StructTrajectory &other)
      : Total_Index(other.Total_Index), ACC_Time(other.ACC_Time), DEC_Time(other.DEC_Time)
  {
    uint16_t n = Total_Index + 1;

    theta = new double *[2];
    speed = new double *[2];
    for (int i = 0; i < 2; ++i)
    {
      theta[i] = new double[n];
      speed[i] = new double[n];

      if (!theta[i] || !speed[i])
      {
        deallocateMemory();
        // throw std::bad_alloc();
      }

      for (uint16_t j = 0; j <= Total_Index; ++j)
      {
        theta[i][j] = other.theta[i][j];
        speed[i][j] = other.speed[i][j];
      }
    }
  }

  // Copy assignment operator
  StructTrajectory &operator=(const StructTrajectory &other)
  {
    if (this != &other)
    { // Tránh tự gán
      deallocateMemory();

      Total_Index = other.Total_Index;
      ACC_Time = other.ACC_Time;
      DEC_Time = other.DEC_Time;

      uint16_t n = Total_Index + 1;

      theta = new double *[2];
      speed = new double *[2];
      for (int i = 0; i < 2; ++i)
      {
        theta[i] = new double[n];
        speed[i] = new double[n];

        if (!theta[i] || !speed[i])
        {
          deallocateMemory();
          // throw std::bad_alloc();
        }

        for (uint16_t j = 0; j <= Total_Index; ++j)
        {
          theta[i][j] = other.theta[i][j];
          speed[i][j] = other.speed[i][j];
        }
      }
    }
    return *this;
  }

  // Destructor
  ~StructTrajectory()
  {
    deallocateMemory();
  }

  // Getter/setter (ví dụ)
  double getTheta(uint8_t row, uint16_t col) const
  {
    if (row >= 0 && row < 2 && col >= 0 && col <= Total_Index)
    {
      return theta[row][col];
    }
    else
    {
      Serial.println("Lỗi: Chỉ số không hợp lệ trong getTheta\n"); // In thông báo lỗi
      return 0.0;                                                  // Hoặc throw một ngoại lệ
    }
  }

  // Setter cho theta
  void setTheta(uint8_t row, uint16_t col, double value)
  {
    if (row >= 0 && row < 2 && col >= 0 && col <= Total_Index)
    {
      theta[row][col] = value;
    }
    else
    {
      Serial.println("Lỗi: Chỉ số không hợp lệ trong setTheta\n");
    }
  }

  // Getter cho speed
  double getSpeed(uint8_t row, uint16_t col) const
  {
    if (row >= 0 && row < 2 && col >= 0 && col <= Total_Index)
    {
      return speed[row][col];
    }
    else
    {
      Serial.println("Lỗi: Chỉ số không hợp lệ trong getSpeed\n");
      return 0.0; // Hoặc throw một ngoại lệ
    }
  }

  // Setter cho speed
  void setSpeed(uint8_t row, uint16_t col, uint16_t value)
  {
    if (row >= 0 && row < 2 && col >= 0 && col <= Total_Index)
    {
      speed[row][col] = value;
    }
    else
    {
      Serial.println("Lỗi: Chỉ số không hợp lệ trong setSpeed\n");
      //  Hoặc bạn có thể ném một ngoại lệ tùy theo cách xử lý lỗi của bạn
    }
  }
  // Getter cho Total_Index
  uint16_t getTotal_Index() const
  {
    return Total_Index;
  }

  // Setter cho Total_Index
  void setTotal_Index(uint16_t newTotalIndex)
  {
    // Kiểm tra xem chỉ số mới có hợp lệ không
    if (newTotalIndex >= 0)
    {
      // Giải phóng bộ nhớ cũ (nếu có)
      deallocateMemory();

      // Cập nhật Total_Index
      Total_Index = newTotalIndex;

      // Cấp phát lại bộ nhớ cho theta và speed với kích thước mới
      uint16_t n = Total_Index + 1;
      theta = new double *[2];
      speed = new double *[2];
      for (int i = 0; i < 2; ++i)
      {
        theta[i] = new double[n];
        speed[i] = new double[n];

        // Kiểm tra cấp phát bộ nhớ
        if (!theta[i] || !speed[i])
        {
          deallocateMemory();
          Serial.println("Fail");
          // throw std::bad_alloc();
        }

        // Khởi tạo các phần tử mới với giá trị 0
        for (uint16_t j = 0; j <= Total_Index; ++j)
        {
          theta[i][j] = 0.0;
          speed[i][j] = 0.0;
        }
      }
    }
    else
    {
      Serial.println("Lỗi: Chỉ số Total_Index không hợp lệ\n");
      //  Hoặc bạn có thể ném một ngoại lệ tùy theo cách xử lý lỗi của bạn
    }
  }

private:
  void deallocateMemory()
  {
    for (int i = 0; i < 2; ++i)
    {
      delete[] theta[i];
      delete[] speed[i];
    }
    delete[] theta;
    delete[] speed;
  }
};

#endif