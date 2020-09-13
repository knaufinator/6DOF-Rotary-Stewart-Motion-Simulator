#include "gbj_filter_exponential.h"
#include <WString.h>
const String gbj_filter_exponential::VERSION = "GBJ_FILTER_EXPONENTIAL 1.1.0";


// Constructor
gbj_filter_exponential::gbj_filter_exponential(float smoothingFactor)
{
  setFactor(smoothingFactor);
  init();
}


// Filtering
float gbj_filter_exponential::getValue(float value)
{
  if (_init)
  {
    _value = value;
    _init = false;
  }
  else
  {
    _value += getFactor() * (value - _value);
  }
  return _value;
}
