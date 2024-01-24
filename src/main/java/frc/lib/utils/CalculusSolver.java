package frc.lib.utils;

import java.util.ArrayList;

public class CalculusSolver {
  ArrayList<Double> xValues = new ArrayList<Double>();
  ArrayList<Double> yValues = new ArrayList<Double>();

  int maxToKeep;

  public CalculusSolver(int maxToKeep) {
    this.maxToKeep = maxToKeep;
  }

  public void addPoint(double x, double y) {
    xValues.add(x);
    yValues.add(y);
    if (xValues.size() > maxToKeep * 3) {
      // remove the first two thirds of the values
      ArrayList<Double> newxValues = new ArrayList<Double>();
      ArrayList<Double> newyValues = new ArrayList<Double>();

      for (int i = xValues.size() * 2 / 3; i < xValues.size(); i++) {
        newxValues.add(xValues.get(i));
        newyValues.add(yValues.get(i));
      }
      xValues = newxValues;
      yValues = newyValues;

    }
  }

  public double getInstantaneousDerivative() {
    if (xValues.size() < 2) {
      return 0.0;
    }
    return (yValues.get(yValues.size() - 1) - yValues.get(yValues.size() - 2))
        / (xValues.get(xValues.size() - 1) - xValues.get(xValues.size() - 2));
  }

  public double getIntegralOverTime() {
    if (xValues.size() < 2) {
      return 0.0;
    }
    double sum = 0.0;
    for (int i = 1; i < xValues.size(); i++) {
      sum += (yValues.get(i) + yValues.get(i - 1)) / 2.0 * (xValues.get(i) - xValues.get(i - 1));
    }
    return sum;
  }

  public double getIntegralOverTimePeriod(double xStart, double xEnd) {
    if (xValues.size() < 2) {
      return 0.0;
    }
    if (xStart > xEnd) {
      return 0.0;
    }
    double sum = 0.0;

    int startIndex = getClosestValueX(xStart, false);
    int endIndex = getClosestValueX(xEnd, true);

    for (int i = startIndex + 1; i < endIndex; i++) {
      sum += (yValues.get(i) + yValues.get(i - 1)) / 2.0 * (xValues.get(i) - xValues.get(i - 1));
    }

    return sum;
  }

  public double getDoubleIntegral() {
    if (xValues.size() < 2) {
      return 0.0;
    }
    double sum = 0.0;
    ArrayList<Double> yValues2 = new ArrayList<Double>();
    for (int i = 1; i < xValues.size(); i++) {
      yValues2.add((yValues.get(i) + yValues.get(i - 1)) / 2.0 * (xValues.get(i) - xValues.get(i - 1)));
    }

    for (int i = 1; i < xValues.size(); i++) {
      sum += (yValues2.get(i) + yValues2.get(i - 1)) / 2.0 * (xValues.get(i) - xValues.get(i - 1));
    }
    return sum;
  }

  public int getClosestValueX(double x, boolean goOver) {
    // use binary search to find the closest value to x
    // if goOver is true, then return the closest value that is greater than x
    if (xValues.size() == 0) {
      return 0;
    }
    if (xValues.indexOf(x) != -1) {
      return xValues.indexOf(x);
    }
    int low = 0;
    int high = xValues.size() - 1;
    int mid = 0;

    while (low < high) {
      mid = (low + high) / 2;
      if (xValues.get(mid) < x) {
        low = mid + 1;
      } else if (xValues.get(mid) > x) {
        high = mid - 1;
      } else {
        return mid;
      }
    }
    if (goOver) {
      if (xValues.get(low) > x) {
        return low;
      } else {
        return low + 1;
      }
    } else {
      if (xValues.get(low) < x) {
        return low;
      } else {
        return low - 1;
      }
    }
  }
}
