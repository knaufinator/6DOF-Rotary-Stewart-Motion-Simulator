#ifndef _DIGITAL_FILTERS_H_
#define _DIGITAL_FILTERS_H_

#include <stdexcept>
#include <cmath>
#include "CircularDelay/CircularDelay.hpp"

template<typename T>
constexpr T squareOf(T input){return input * input;}

namespace tps{
	template<typename T>
	constexpr T pow(T input, unsigned int power){return (power == 0 ? 1 : input * (power <= 1 ? 1 : tps::pow(input, power-1)));}
}

template<typename T>
constexpr T calcC_Cr(T k, T m){return 2 * sqrt(k * m);}

/**
 * @brief      Abstract base class for digital moving filters.
 *
 *             Moving filter are real time filter used for applications where
 *             continuous filtering is necessary as it can be part of an control
 *             system.
 *
 * @tparam     Type  Floating point type used.
 */
template<typename Type>
class DigitalFilter{
public:
	virtual Type update(Type newValue) = 0;
	virtual Type getOutput() = 0;
};

/**
 * @brief      Class for differentiator.
 */
template<typename T>
class Differentiator : public DigitalFilter<T> {
public:
	Differentiator(T sampleTime):
		sampleTime(sampleTime)
	{}
	T update(T input){
		y = (input - x1) / sampleTime;
		x1 = input;
		return y;
	}
	T getOutput(){return y;}
private:
	const T sampleTime;
	T y = 0;
	T x1 = 0;
};

/**
 * @brief      Class for a low pass filter.
 *
 *             Design to be a first order Butterworth low pass filter.
 *             Transformation done using the matched-Z-transform method
 */
class LowPassFilter : public DigitalFilter<double> {
public:
	/**
	 * @brief      Constructor to set sample time and the tau constant
	 *
	 * @param[in]  idt     Sample time for the low pass filter
	 * @param[in]  itua_c  Or
	 *             @f$ \tau_c
	 *             @f$ The time constant for the filter. Note that
	 *             @f$ \tau_c = \frac{1}{2 pi f_c}@f$  where @f$ f_c @f$ is the cutoff frequency
	 */
	LowPassFilter(double idt, double omega_c, double ioutput = 0):
		epow(exp(-idt * omega_c)),
		dt(idt),
		output(ioutput){
			if(omega_c < idt){
				throw std::domain_error("LowPassFilter constructor error: tua_c is smaller than the sample time dt.");
			}
		}
	/**
	 * @brief      Update function to push new value into the low pass filter
	 *
	 * @param[in]  newValue  The new value after dt time
	 *
	 * @return     The new output value
	 */
	double update(double newValue) final{return output = (output-newValue) * epow + newValue;}
	/**
	 * @brief      Gets the output.
	 *
	 * @return     The output.
	 */
	double getOutput() final{return output;}
	/**
	 * @brief      Force the output to a desired value
	 *
	 *             This can be useful when the output needs to be forced in case
	 *             of extreme inputs or such
	 *
	 * @param[in]  newOutput  The new output
	 */
	void configOutput(double newOutput){output = newOutput;}
	const double* outputPointer(){return &output;}
private:
	const double epow; /// one time calculation constant
	const double dt;
	double output;
};

/**
 * @brief      Class for a 2nd order low pass filter.
	*
 *             Design to be a 2nd order Butterworth low pass filter.
 *             Transformation done using the bilinear transform method
 */
class LowPassFilter2 : public DigitalFilter<double> {
public:
	/**
	 * @brief      Constructor to set sample time and the tau constant
	 *
	 * @param[in]  idt     Sample time for the low pass filter
	 * @param[in]  itua_c  Or
	 *             @f$ \tau_c
	 *             @f$ The time constant for the filter. Note that
	 *             @f$ \tau_c = \frac{1}{2 pi f_c}@f$  where @f$ f_c @f$ is the cutoff frequency
	 */
	LowPassFilter2(double dt, double tau_c, double ioutput = 0):
		yc{
			-2 * (pow(dt, 2) - 4 * pow(tau_c, 2)) / (pow(dt, 2) + 2 * sqrt(2) * tau_c * dt + 4 * pow(tau_c, 2)),
			(-pow(dt, 2) + 2 * sqrt(2) * tau_c * dt - 4 * pow(tau_c, 2)) / (pow(dt, 2) + 2 * sqrt(2) * tau_c * dt + 4 * pow(tau_c, 2))
		},
		xc{
			pow(dt, 2) / (pow(dt, 2) + 2 * sqrt(2) * tau_c * dt + 4 * pow(tau_c, 2)),
			2 * pow(dt, 2) / (pow(dt, 2) + 2 * sqrt(2) * tau_c * dt + 4 * pow(tau_c, 2)),
			pow(dt, 2) / (pow(dt, 2) + 2 * sqrt(2) * tau_c * dt + 4 * pow(tau_c, 2))
		}
		{
			if(tau_c < M_PI * dt){
				throw std::domain_error("LowPassFilter constructor error: tua_c is smaller than the sample time dt.");
			}
		}
	/**
	 * @brief      Update function to push new value into the low pass filter
	 *
	 * @param[in]  newValue  The new value after dt time
	 *
	 * @return     The new output value
	 */
	double update(double newValue) final{
		x.push(newValue);
		double output = 0;
		for (int i = 0; i < 2; ++i)
			output += y.get(i) * yc[i];
		for (int i = 0; i < 3; ++i)
			output += x.get(i) * xc[i];
		return y.push(output);
	}
	/**
	 * @brief      Gets the output.
	 *
	 * @return     The output.
	 */
	double getOutput() final{return y.get(0);}
	/**
	 * @brief      Force the output to a desired value
	 *
	 *             This can be useful when the output needs to be forced in case
	 *             of extreme inputs or such
	 *
	 * @param[in]  newOutput  The new output
	 */
	void configOutput(double newOutput){
		for(auto& it : x){
			it = newOutput;
		}
		for(auto& it : y){
			it = newOutput;
		}
	}
private:
	const double yc[2];
	const double xc[3];
	CircularDelay<double, 2> y;
	CircularDelay<double, 3> x;
};

/**
 * @brief      Class for high pass filter using bilinear transform.
 */
class HighPassFilter : public DigitalFilter<double> {
public:
	/**
	 * @brief      Constructor to set sample time and the tau constant
	 *
	 * @param[in]  idt     Sample time for the low pass filter
	 * @param[in]  itua_c  Or
	 *             @f$ \tau_c
	 *             @f$ The time constant for the filter. Note that
	 *             @f$ \tau_c = \frac{1}{2 pi f_c}@f$  where @f$ f_c @f$ is the cutoff frequency
	 */
	HighPassFilter(double idt, double omega_c):
		amplFac(1/((idt * omega_c / 2) + 1)),
		y1c((idt * omega_c / 2) - 1),
		dt(idt){
			if(omega_c < idt){
				throw std::domain_error("LowPassFilter constructor error: tua_c is smaller than the sample time dt.");
			}
		}
	/**
	 * @brief      Update function to push new value into the low pass filter
	 *
	 * @param[in]  newValue  The new value after dt time
	 *
	 * @return     The new output value
	 */
	double update(double newValue) final{
		// Note that output before assignment equals y1 being y[n-1]
		output = amplFac * (newValue - x1 - output * y1c);
		x1 = newValue;
		return output;
	}
	/**
	 * @brief      Gets the output.
	 *
	 * @return     The output.
	 */
	double getOutput() final{return output;}
	/**X
	 * @brief      Force the output to a desired value
	 *
	 *             This can be useful when the output needs to be forced in case
	 *             of extreme inputs or such
	 *
	 * @param[in]  newOutput  The new output
	 */
	void configOutput(double newOutput){output = newOutput;}
	const double* outputPointer(){return &output;}
private:
	const double amplFac; // one time calculation constant
	const double y1c; // one time calculation constant
	const double dt;
	double x1 = 0;
	double output = 0;
};

/**
 * @brief      Class for third order high pass filter. This is designed using
 *             the bilinear transform.
 */
class HighPassFilter3 : public DigitalFilter<double> {
public:
	HighPassFilter3(double sampleTime, double omega_c, double ioutput = 0):
		xc{
			8
			,
			-24
			,
			24
			,
			-8
		},
		yc{
			1 / (1 * tps::pow(sampleTime * omega_c, 3) + 4 * tps::pow(sampleTime * omega_c, 2) + 8 * sampleTime * omega_c + 8),
			     3 * tps::pow(sampleTime * omega_c, 3) + 4 * tps::pow(sampleTime * omega_c, 2) - 8 * sampleTime * omega_c - 24,
			     3 * tps::pow(sampleTime * omega_c, 3) - 4 * tps::pow(sampleTime * omega_c, 2) - 8 * sampleTime * omega_c + 24,
			     1 * tps::pow(sampleTime * omega_c, 3) - 4 * tps::pow(sampleTime * omega_c, 2) + 8 * sampleTime * omega_c - 8
		}
		{
			if(omega_c < sampleTime){
				throw std::domain_error("LowPassFilter constructor error: tua_c is smaller than the sample time dt.");
			}
		}
	double update(double newValue) final{
		x.push(newValue);
		double y0 = 0;
		const double* doubleP = xc;
		for (auto it = x.rbegin(); it != x.rend(); it++)
		{
			y0 += *it * *doubleP++;
		}
		doubleP = yc + 1;
		for (auto it = y.rbegin(); it != y.rend(); it++)
		{
			y0 -= *it * *doubleP++;
		}
		return y.push(yc[0] * y0);
		// return y.push(yc[0] * (
		// 		x.get(0) * xc[0] + x.get(1) * xc[1] + x.get(2) * xc[2] + x.get(3) * xc[3] -
		// 		y.get(0) * yc[1] - y.get(1) * yc[2] - y.get(2) * yc[3]
		// 	)
		// );
	}
	double getOutput() final{return y.get(0);}
private:
	const double xc[4];
	const double yc[4];
	CircularDelay<double, 3> y;
	CircularDelay<double, 4> x;
};

/**
 * @brief      Class for third order high pass filter. This is designed using
 *             the bilinear transform.
 */
class LowPassFilter3 : public DigitalFilter<double> {
public:
	LowPassFilter3(long double sampleTime, long double omega_c, long double ioutput = 0):
		yc{
			1
			,
			(double)((3 * tps::pow(sampleTime * omega_c, 3) + 4 * tps::pow(sampleTime * omega_c, 2) - 8 * sampleTime * omega_c - 24)
			/
			(1 * tps::pow(sampleTime * omega_c, 3) + 4 * tps::pow(sampleTime * omega_c, 2) + 8 * sampleTime * omega_c + 8))
			,
			(double)((3 * tps::pow(sampleTime * omega_c, 3) - 4 * tps::pow(sampleTime * omega_c, 2) - 8 * sampleTime * omega_c + 24)
			/
			(1 * tps::pow(sampleTime * omega_c, 3) + 4 * tps::pow(sampleTime * omega_c, 2) + 8 * sampleTime * omega_c + 8))
			,
			(double)((1 * tps::pow(sampleTime * omega_c, 3) - 4 * tps::pow(sampleTime * omega_c, 2) + 8 * sampleTime * omega_c - 8)
			/
			(1 * tps::pow(sampleTime * omega_c, 3) + 4 * tps::pow(sampleTime * omega_c, 2) + 8 * sampleTime * omega_c + 8))
		},
		xc{
			(double)(1 * tps::pow(sampleTime * omega_c, 3)
			/
			(1 * tps::pow(sampleTime * omega_c, 3) + 4 * tps::pow(sampleTime * omega_c, 2) + 8 * sampleTime * omega_c + 8))
			,
			(double)(3 * tps::pow(sampleTime * omega_c, 3)
			/
			(1 * tps::pow(sampleTime * omega_c, 3) + 4 * tps::pow(sampleTime * omega_c, 2) + 8 * sampleTime * omega_c + 8))
			,
			(double)(3 * tps::pow(sampleTime * omega_c, 3)
			/
			(1 * tps::pow(sampleTime * omega_c, 3) + 4 * tps::pow(sampleTime * omega_c, 2) + 8 * sampleTime * omega_c + 8))
			,
			(double)(1 * tps::pow(sampleTime * omega_c, 3)
			/
			(1 * tps::pow(sampleTime * omega_c, 3) + 4 * tps::pow(sampleTime * omega_c, 2) + 8 * sampleTime * omega_c + 8))
		}
		{
			if(omega_c < sampleTime){
				throw std::domain_error("LowPassFilter constructor error: tua_c is smaller than the sample time dt.");
			}
		}
	double update(double newValue) final{
		x.push(newValue);
		double y0 = 0;
		const double* doubleP = xc;
		for (auto it = x.rbegin(); it != x.rend(); it++)
		{
			y0 += *it * *doubleP++;
		}
		doubleP = yc + 1;
		for (auto it = y.rbegin(); it != y.rend(); it++)
		{
			y0 -= *it * *doubleP++;
		}
		return y.push(yc[0] * y0);
	}
	double getOutput() final{return y.get(0);}
private:
	const double yc[4];
	const double xc[4];
	CircularDelay<double, 3> y;
	CircularDelay<double, 4> x;
};

/**
 * @brief      Class for third order high pass filter. This is designed using
 *             the matched Z transform.
 */
class LowPassFilter3MatchedZ : public DigitalFilter<double> {
public:
	LowPassFilter3MatchedZ(long double sampleTime, long double omega_c):
		amplFac(-(2*(expl(3 * omega_c * sampleTime) - expl(2 * omega_c * sampleTime))*cosl(sqrtl(3) * omega_c * sampleTime / 2) - expl(7 * omega_c * sampleTime / 2) + expl(3 * omega_c * sampleTime / 2))*expl(-7 * omega_c * sampleTime / 2)),
		yc{
			(double)(-(2 * cosl(sqrtl(3) * omega_c * sampleTime / 2) * expl(omega_c * sampleTime * 5 / 2) + expl(2 * omega_c * sampleTime)) * expl(-3 * omega_c * sampleTime))
			,
			(double)((2 * cosl(sqrtl(3) * omega_c * sampleTime / 2) * expl(omega_c * sampleTime * 3 / 2) + expl(2 * omega_c * sampleTime)) * expl(-3 * omega_c * sampleTime))
			,
			(double)(-expl(-2 * omega_c * sampleTime))
		}
		{
			if(omega_c / (2 * M_PI) < sampleTime){
				throw std::domain_error("LowPassFilter3MatchedZ constructor error: tua_c is smaller than the sample time dt.");
			}
		}
	double update(double newValue) final{
		double y0 = newValue * amplFac;
		const double* doubleP = yc;
		for (auto it = y.rbegin(); it != y.rend(); it++)
		{
			y0 -= *it * *doubleP++;
		}
		return y.push(y0);
	}
	double getOutput() final{return y.get(0);}
private:
	const double amplFac;
	const double yc[3];
	CircularDelay<double, 3> y;
};

/**
 * @brief      Class for third order high pass filter. This is designed using
 *             the approximated differtial approuch where s=(Z-1)/(Z*T).
 */
class LowPassFilter3DiffApprox : public DigitalFilter<double> {
public:
	LowPassFilter3DiffApprox(double sampleTime, double omega_c, double ioutput = 0):
		xc{
			1 * tps::pow(sampleTime * omega_c, 3)
			,
			0
			,
			0
			,
			0
		},
		yc{
			1 / (1 * tps::pow(sampleTime * omega_c, 3) + 2 * tps::pow(sampleTime * omega_c, 2) + 2 * sampleTime * omega_c + 1),
			     0 * tps::pow(sampleTime * omega_c, 3) - 2 * tps::pow(sampleTime * omega_c, 2) - 4 * sampleTime * omega_c - 3,
			     0 * tps::pow(sampleTime * omega_c, 3) + 0 * tps::pow(sampleTime * omega_c, 2) + 2 * sampleTime * omega_c + 3,
			     0 * tps::pow(sampleTime * omega_c, 3) - 0 * tps::pow(sampleTime * omega_c, 2) + 0 * sampleTime * omega_c - 1
		}
		{
			if(omega_c < sampleTime){
				throw std::domain_error("LowPassFilter constructor error: tua_c is smaller than the sample time dt.");
			}
		}
	double update(double newValue) final{
		x.push(newValue);
		double y0 = 0;
		const double* doubleP = xc;
		for (auto it = x.rbegin(); it != x.rend(); it++)
		{
			y0 += *it * *doubleP++;
		}
		doubleP = yc + 1;
		for (auto it = y.rbegin(); it != y.rend(); it++)
		{
			y0 -= *it * *doubleP++;
		}
		return y.push(yc[0] * y0);
	}
	double getOutput() final{return y.get(0);}
private:
	const double xc[4];
	const double yc[4];
	CircularDelay<double, 3> y;
	CircularDelay<double, 4> x;
};

template<size_t size>
class MovingAvarageFilter{
public:
	double update(double input){
		input *= 1000;
		output += int64_t(input) - *buffer.rbegin();
		buffer.push(input);
		return double(output) / (1000);
	}
private:
	int64_t output = 0;
	CircularDelay<int64_t, size> buffer;
};

#endif
