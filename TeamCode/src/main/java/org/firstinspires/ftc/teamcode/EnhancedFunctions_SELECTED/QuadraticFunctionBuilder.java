package org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED;

import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.jetbrains.annotations.Contract;

import java.util.ArrayList;

public final strictfp class QuadraticFunctionBuilder {

    /// @return a, b, and c from created quadratic equation
    /// @exception IllegalArgumentException Both X_DATA and Y_DATA must have 3 elements each!
    @Contract("_, _ -> new")
    public static double[] getQuadraticInformationFromData(double[] X_DATA, double[] Y_DATA) {

        if (X_DATA.length != 3 || Y_DATA.length != 3) {
            throw new IllegalArgumentException("Both X_DATA and Y_DATA must have 3 elements each!");
        }

        // a, b, and c values from ax² + bx + c

        /*
         * y1 = ax1^2 + bx1 + c
         * y2 = ax2^2 + bx2 + c
         * y3 = ax3^2 + bx3 + c
         */

        double a_x1 = Math.pow(X_DATA[0], 2);
        double b_x1 = X_DATA[0];
        double a_x2 = Math.pow(X_DATA[1], 2);
        double b_x2 = X_DATA[1];
        double a_x3 = Math.pow(X_DATA[2], 2);
        double b_x3 = X_DATA[2];

        double y1 = Y_DATA[0];
        double y2 = Y_DATA[1];
        double y3 = Y_DATA[2];

        // Get rid of c

        // y1 = (x1^2)a + x1b
        // y2 = (x2^2)a + x2b

        ArrayList<Double> equation1 = new ArrayList<>();
        ArrayList<Double> equation2 = new ArrayList<>();

        equation1.add(MathUtil.toDouble(y1 - y2));
        equation1.add(MathUtil.toDouble(a_x1 - a_x2));
        equation1.add(MathUtil.toDouble(b_x1 - b_x2));

        equation2.add(MathUtil.toDouble(y2 - y3));
        equation2.add(MathUtil.toDouble(a_x2 - a_x3));
        equation2.add(MathUtil.toDouble(b_x2 - b_x3));

        // Solve for a and b by elimination

        //*a*
        ArrayList<Double> modifiedEquation1 = new ArrayList<>();
        ArrayList<Double> modifiedEquation2 = new ArrayList<>();

        modifiedEquation1.add(b_x2 - b_x3 * equation1.get(0));
        modifiedEquation1.add(b_x2 - b_x3 * equation1.get(1));

        modifiedEquation2.add(b_x1 - b_x2 * equation2.get(0));
        modifiedEquation2.add(b_x1 - b_x2 * equation2.get(1));

        // y1 - y2 = ((x1^2) - (x2^2))a + (x1 - x2)b
        // y1 - y2 = ((x1^2) - (x2^2))a
        // ((x1^2) - (x2^2))a = y1 - y2
        // a = (y1 - y2) / ((x1^2) - (x2^2))

        double a = (modifiedEquation1.get(0) - modifiedEquation2.get(0)) / (modifiedEquation1.get(1) - modifiedEquation2.get(1));

        //*b*
        modifiedEquation1.set(0, a_x2 - a_x3 * equation1.get(0));
        modifiedEquation1.set(1, a_x2 - a_x3 * equation1.get(2));

        modifiedEquation2.set(0, a_x1 - a_x2 * equation2.get(0));
        modifiedEquation2.set(1, a_x1 - a_x2 * equation2.get(2));

        // y1 - y2 = ((x1^2) - (x2^2))a + (x1 - x2)b
        // y1 - y2 = (x1 - x2)b
        // (x1 - x2)b = y1 - y2
        // b = (y1 - y2) / (x1 - x2)

        double b = (modifiedEquation1.get(0) - modifiedEquation2.get(0)) / (modifiedEquation1.get(1) - modifiedEquation2.get(1));

        //*c*
        // go back to original equations
        // y1 = ax1^2 + bx1 + c
        // -c = ax1^2 + bx1 - y1
        // c = -ax1^2 - bx1 + y1 |OR| c = -(ax1^2 + bx1 - y1)

        double c = -(a * a_x1 + b * b_x1 - y1);

        return new double[] {a, b, c};
    }

}