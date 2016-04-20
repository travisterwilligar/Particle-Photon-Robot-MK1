#ifndef _TB6612FNG
#define _TB6612FNG

#include "application.h"

  class Motors{
    private:
      int STBY;
      int PWMA;
      int AIN1;
      int AIN2;
      int PWMB;
      int BIN1;
      int BIN2;
    public:
      Motors(int STBY, int PWMA, int AIN1, int AIN2, int PWMB, int BIN1, int BIN2);
      void move(int motor, int speed, int direction);
      void forward(int speed);
      void reverse(int speed);
      void right(int speed);
      void left(int speed);
      void stop();
  };

#endif
