/*
  NAME:
  gbj_filter_exponential

  DESCRIPTION:
  The library smooths (filters) a data serie by exponential filtering.
  - A new filtered value is calculated from stored previous one and
    a currently measured value.
  - The serie of filtered values represents a serie of really measured
    (usually measured and/or calculated) values with reduced digital noise.

  LICENSE:
  This program is free software; you can redistribute it and/or modify
  it under the terms of the license GNU GPL v3 http://www.gnu.org/licenses/gpl-3.0.html
  (related to original code) and MIT License (MIT) for added code.

  CREDENTIALS:
  Author: Libor Gabaj
  GitHub: https://github.com/mrkaleArduinoLib/gbj_filter_exponential.git

  CREDITS:
  Wikipedia: Exponential smoothing (https://en.wikipedia.org/wiki/Exponential_smoothing)
*/
#ifndef GBJ_FILTER_EXPONENTIAL_H
#define GBJ_FILTER_EXPONENTIAL_H

#include <WString.h>
#include <cmath>
#include "math.h"
#include "Arduino.h"



class gbj_filter_exponential
{
  public:
    static const String VERSION;

    /*
      Constructor.

      DESCRIPTION:
      Constructor stores the smoothing factor within a class instance object
      with initial internal status flags.
      - The constructor has all arguments defaulted. If some argument after
        some defaulted arguments should have a specific value, use corresponding
        constants in place of those defaulted arguments.

      PARAMETERS:
      smoothingFactor - smoothing factor for exponential filtering.
                        - Data type: float
                        - Default value: 0.5
                        - Limited range: 0.0 ~ 1.0

      RETURN:  object
    */
    gbj_filter_exponential(float smoothingFactor = 0.5);


    /*
      Reset all status flags.

      DESCRIPTION:
      The method initiates all internal status flags of a class
      instance object to default values as they are right after power up of
      a microcontroler.

      PARAMETERS: none

      RETURN: none
    */
    inline void init() { _init = true; };

    /*
      Calculate new filtered value from measured value.

      DESCRIPTION:
      The method calculates a new filtered value from the input value,
      previous stored filtered value, and stored smoothing factor in the class
      instance object.
      - Right after microcontroler power up or initiating the instance object
        by corresponding method the very first input value is considered as
        a previous filtered value, or starting value.

      PARAMETERS:
      value - measured value to be filtered.
              - Data type: float
              - Default value: none
              - Limited range: rational numbers

      RETURN: Filtered value
    */
    float getValue(float value);

    inline void setFactor(float factor) { _factor = constrain(fabs(factor), 0.0, 1.0); };
    inline float getFactor() { return _factor; };


  private:
    float _factor;        // Smoothing factor
    float _value;         // Recent filtered value
    bool  _init = true;   // Flag about initial filtering
};

#endif
