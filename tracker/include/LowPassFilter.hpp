#ifndef _LOW_PASS_FILTER_H_
#define _LOW_PASS_FILTER_H_

template<typename T>
class LowPassFilter
{
public:
    LowPassFilter(const T smoothing)
        : smoothing(smoothing), valueCount(0)
    {
    }

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
    T ppValue;
    T pValue;
    const T smoothing;
    int valueCount;
};

#endif
