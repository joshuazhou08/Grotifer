// Filter used by Torp Homing
class MovingAverage
{
public:
  MovingAverage(int filterLength);
  double addSample(double newValue);
  double getValue();
  void dumpFilter();

private:
  double *filterPointer;
  int filterLength;
  double lastValue;

  void initFilter();
  void shiftFilter(double nextValue);
  void computeAverage();
};