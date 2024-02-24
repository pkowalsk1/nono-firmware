// #include "pico/sleep.h"

class NonoLifeCycle
{
public:
  NonoLifeCycle(){};
  ~NonoLifeCycle(){};

  void init();
  void gpioInterupt();

private:
};
