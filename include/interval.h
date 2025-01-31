//
// Created by treyb on 2025-01-21.
//

#ifndef INTERVAL_H
#define INTERVAL_H

#include "usefulstuff.h"

class interval {
  public:
    double min, max;

    interval() : min(+infinity), max(-infinity) {} //default interval is empty

    interval(double min, double max) : min(min), max(max) {}

    interval(const interval& a, const interval& b) {
      //create the interval tightly enclosing the two input intervals
      min = a.min <= b.min ? a.min : b.min;
      max = a.max >= b.max ? a.max : b.max;
    }

    double size() const {
      return max - min;
    }

    //true if a value x is inside the interval [min, max]
    bool contains(double x) const {
      return min <= x && x <= max;
    }

    //true is a value x is inside the interval (min, max)
    bool surrounds(double x) const {
      return min < x && x < max;
    }

  double clamp(double x) const {
      if (x < min) return min;
      if (x > max) return max;
      return x;
    }

  interval expand(double delta) const {
      auto padding = delta/2;
      return interval(min - padding, max + padding);
    }

    //static means this the value is shared between all objects of this class (interesting!)
    static const interval empty, universe;
};

const interval interval::empty    = interval(+infinity, -infinity);
const interval interval::universe = interval(-infinity, +infinity);

interval operator+(const interval& ival, double displacement) {
  return interval(ival.min + displacement, ival.max + displacement);
}

interval operator+(double displacement, const interval& ival) {
  return ival + displacement;
}

#endif //INTERVAL_H
