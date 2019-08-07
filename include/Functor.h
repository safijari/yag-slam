#ifndef FUNCTOR_H
#define FUNCTOR_H

#include <stdint.h>

class Functor {
public:
  /**
   * Functor function
   */
  virtual void operator()(int32_t){};
}; // Functor

#endif
