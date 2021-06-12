package frc.robot.utilities;

public class MovingAverage {

  private final CircularBufferBHR m_inputs;

  public MovingAverage(int taps) {
    m_inputs = new CircularBufferBHR(taps);
  }

  /** Reset the filter state. */
  public void reset() {
    m_inputs.clear();
  }

  /**
   * Calculates the next value of the filter.
   *
   * @param input Current input value.
   * @return The filtered value at this step
   */
  public double calculate(double input) {
    double retVal = 0.0;

    // Rotate the inputs
    m_inputs.addFirst(input);

    // Calculate the new value
    for (int i = 0; i < m_inputs.size(); i++) {
      retVal += m_inputs.get(i);
    }
   
    return retVal/m_inputs.size();
  }

  public static void main(String... args)
  {
      MovingAverage filter = new MovingAverage(5);
      double test = filter.calculate(10);
      System.out.println("Test 1 = " + test);
      test = filter.calculate(10);
      System.out.println("Test 2 = " + test);
      test = filter.calculate(10);
      System.out.println("Test 3 = " + test);
      test = filter.calculate(10);
      System.out.println("Test 4 = " + test);
      test = filter.calculate(10);
      System.out.println("Test 5 = " + test);
      test = filter.calculate(10);
      System.out.println("Test 6 = " + test);
  }


}
