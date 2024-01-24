package frc.lib.utils;

public class EricNubControls {
    double k;

    public EricNubControls() {
        k = 1.0;
    }

    public double addDeadzoneBasic(double input, double deadzone) {
        if (Math.abs(input) < deadzone) {
            return 0;
        } else {
            return input;
        }
    }

    public double addDeadzoneScaled(double input, double deadzone) {
        if (Math.abs(input) < deadzone) {
            return 0;
        } else {
            return (input - Math.signum(input) * deadzone) / (1 - deadzone);
        }
    }

    public double addEricCurve(double input) {
        if (input < 0.8 && input > 0) {
            return k * (((1.0 / 3.0) * input) + (1.0 / 9.0) * (input * input) + (65.0 / 72.0) * input * input * input);
        } else if (input > 0) {
            return k * ((1.0 / 5.0) * Math.pow((5.0 * input) - 5.0, 3) + 1.0);
        } else if (input > -.8) {
            return k * (((1.0 / 3.0) * input - (1.0 / 9.0) * input * input + (65.0 / 72.0) * input * input * input));
        } else {
            return k * ((1.0 / 5.0) * Math.pow((5.0 * input) + 5.0, 3) - 1.0);
        }
    }

    // public static void main(String[] args) {
    //     EricNubControls u = new EricNubControls();
    //     System.out.println(u.addEricCurve(.1));
    //     System.out.println(u.addEricCurve(.2));
    //     System.out.println(u.addEricCurve(.3));
    //     System.out.println(u.addEricCurve(.4));
    //     System.out.println(u.addEricCurve(.5));
    //     System.out.println(u.addEricCurve(.6));
    //     System.out.println(u.addEricCurve(.7));
    //     System.out.println(u.addEricCurve(.8));
    //     System.out.println(u.addEricCurve(.9));
    //     System.out.println(u.addEricCurve(1.0));
    //     System.out.println(u.addEricCurve(-0.1));
    //     System.out.println(u.addEricCurve(-0.2));
    //     System.out.println(u.addEricCurve(-0.3));
    //     System.out.println(u.addEricCurve(-0.4));
    //     System.out.println(u.addEricCurve(-0.5));
    //     System.out.println(u.addEricCurve(-0.6));
    //     System.out.println(u.addEricCurve(-0.7));
    //     System.out.println(u.addEricCurve(-0.8));
    //     System.out.println(u.addEricCurve(-0.9));
    //     System.out.println(u.addEricCurve(-1.0));

    // }
}
