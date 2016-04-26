#ifndef _LOW_PASS_FILTER_H_
#define _LOW_PASS_FILTER_H_

/**
 * \brief Implements a generic 2nd order low-pass filter.
 */
template<typename T>
class LowPassFilter
{
public:
    /**
     * \brief Initializes a new instance of the \see LowPassFilter class.
     *
     * \param smoothing Filter smoothing factor, between 0 (no filtering) and 1.0 (full filtering).
     */
    LowPassFilter(const T smoothing)
        : smoothing(smoothing), valueCount(0)
    {
    }

    /**
     * \brief Computes a time step of the low-pass filter using the provided value.
     *
     * \param Raw input value for this time step.
     *
     * \return Filtered value for this time step.
     */
    T nextStep(const T newValue)
    {
        if (valueCount == 2)
        {
            T result = newValue * (1 - smoothing) + (pValue * (smoothing / 2)) + (ppValue * (smoothing / 2));
            ppValue = pValue;
            pValue = result;
            return result;
        }
        else if (valueCount == 1)
        {
            pValue = newValue * (1 - smoothing) + ppValue * smoothing;
            valueCount++;
            return pValue;
        }
        else
        {
            ppValue = newValue;
            valueCount++;
            return ppValue;
        }
    }

private:
    /// 2nd previous value.
    T ppValue;
    /// 1st previous value.
    T pValue;
    /// Smoothing factor.
    const T smoothing;
    /// Number of processed values.
    int valueCount;
};

#endif
