package javakuka;

import java.text.DecimalFormat;
import kinematik.LA;

//v. 10.08.2016 
//modified 11.07.2020
//javakuka.org, whitegreen@163.com

public class Rotary {
	private boolean windows = false; // true:Window, false: Mac
	private KRLwriter wt;

	private final double[] P = { 436.39, -1414.71, 763.37 };
	private final double[] Px = { 341.55, -1315.84, 762.09 };
	private final double[] Pxy = { 341.52, -1516.66, 764.59 };
	private final double[] Op = { 425.80, -1412.27, 763.27 }; // +90
	private final double[] On = { 434.58, -1425.12, 763.49 }; // -90

	private double[] O;
	private double[][] T1;
	private double[][] T2;

	private float R = 138.5f;
	private float H = -400;
	private float H0 = 20;
	private static int num = 10;
	private int steps = 21;
	private float[][][] splines = new float[num][steps][];
	private float speed = 0.12f;// velocity tuned for 3% speed (set in pendant) AUT mode

	public static void main(String[] args) {
		new Rotary();
	}

	public Rotary() {
		calibrateRotaryTable();
		createSpline();
		writeKRL();
	}

	private void calibrateRotaryTable() {
		O = between(Op, On, 0.5);
		T1 = LA.matrixBy2Axis(LA.sub(P, O), LA.sub(Op, O));
		double[] rela_ax = LA.mul(LA.transpose(T1), LA.sub(Px, P));
		double[] rela_ay = LA.mul(LA.transpose(T1), LA.sub(Pxy, P));
		double d = LA.dist(O, P);
		T2 = LA.matrixBy2Axis(d, 0, 0, rela_ax, rela_ay); // relative to T1;
	}

	private void createSpline() {
		for (int i = 0; i < num; i++) {
			double ang = angle(i);
			for (int j = 0; j < steps; j++) {
				double s = (double) j / (steps - 1);
				double x = 109 * Math.sin(s * 1.75f * Math.PI);
				double y = 50 - R;
				double cos = Math.cos(-ang);
				double sin = Math.sin(-ang);
				double nx = cos * x - sin * y;
				double ny = sin * x + cos * y;
				float[] p = { (float) nx, (float) ny, (float) (H0 + s * H) };
				splines[i][j] = p;
			}
		}
	}

	private static double angle(int i) {
		return (-0.5 + (i + 0.5) / num) * 2 * Math.PI;
	}

	private void writeKRL() {
		wt = new KRLwriter(windows, "rotary.src");
		wt.println("$TOOL=TOOL_DATA[3]"); //the tool measures the 5 points
		wt.println("PTP  XHOME");

		float delta_ang = 32.5f;
		for (int i = 0; i < num; i++) {
			double ang = angle(i);
			float E2 = (float) (ang * 180 / Math.PI);
			float minA = 72 - E2 - delta_ang; // ***
			float maxA = 72 - E2 + delta_ang; // ***
			float[] ABC0 = { minA, 0, -90 };
			float[] ABC1 = { maxA, 0, -90 };

			float[][] spline = splines[i];
			double[][] base = rotateTable(ang);
			wt.BASE(LA.XYZABC(base));

			wt.PTP(spline[0], ABC0, 0, E2);
			wt.println("SPLINE WITH $VEL.CP=" + speed); // cut downward
			wt.SPL(spline[0], ABC0, 0, E2);
			for (int j = 1; j < spline.length; j++)
				wt.SPL(spline[j]);

			wt.SLIN(spline[spline.length - 1], ABC1); // change orientation

			for (int j = spline.length - 1; j >= 0; j--)// cut upward
				wt.SPL(spline[j]);

			wt.println("ENDSPLINE");
		}
		wt.println("PTP  XHOME");
		wt.println("END");
		wt.close();
		System.out.println("src completed");
	}

	private double[][] rotateTable(double angle) {
		double c = Math.cos(angle);
		double s = Math.sin(angle);
		double[] rx = { c, s, 0 };
		double[] ry = { -s, c, 0 };
		double[] nx = LA.mul(T1, rx);
		double[] ny = LA.mul(T1, ry);
		double[][] T1 = LA.matrixBy2Axis(O[0], O[1], O[2], nx, ny);
		return LA.mul(T1, T2);
	}

	private static double[][] mul(double[][] a, double[][] b) { // multiple two 3*3 matrices
		double[][] re = new double[a.length][b[0].length];
		for (int i = 0; i < a.length; i++) {
			for (int j = 0; j < b[0].length; j++) {
				double v = 0;
				for (int k = 0; k < a[0].length; k++)
					v += a[i][k] * b[k][j];
				re[i][j] = v;
			}
		}
		return re;
}
	private static double[] between(double[] a, double[] b, double s) {
		double[] re = new double[a.length];
		for (int i = 0; i < a.length; i++)
			re[i] = a[i] * (1 - s) + b[i] * s;
		return re;
	}

}
