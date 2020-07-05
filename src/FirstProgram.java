package javakuka;

import java.io.PrintWriter;
import java.text.DecimalFormat;

//v. 22.06.2016  tested in Windows and Mac, runs in KRC2 and KRC4;
//javakuka.com, whitegreen@163.com

public class FirstProgram {
	private boolean windows = false; // true:Window, false: Mac
	private boolean external_axes = false; // true: Kuka with external axes, false: without external axes
	private DecimalFormat df = new DecimalFormat("###.#");
	private PrintWriter printer;
	private final int num_pts = 48;
	private final float X = 298; // foam's length in the x-axis of BASE.

	public static void main(String[] args) {
		new FirstProgram();
	}

	public FirstProgram() {
		float[][] spline = createSpline();
		writeKRL(spline);
	}

	private float[][] createSpline() {
		float Y0 = 23;
		float Y1 = 17;
		float[][] spline = new float[num_pts][];
		float margin = 0.2F;
		for (int i = 0; i < num_pts; i++) {
			float theta = (float) (i * 7 * Math.PI) / (num_pts - 1);
			float x = -margin * X + i * (1 + 2 * margin) * X / (num_pts - 1);
			float y = Y0 + Y1 * (float) Math.cos(theta - Math.PI / 2);
			float z = -Y0 - Y1 * (float) Math.cos(theta + Math.PI);
			float[] point = { x, y, z };
			spline[i] = point;
		}
		return spline;
	}

	private void writeKRL(float[][] spline) {
		try {
			printer = new PrintWriter("first_program.src");// create .src file
			header();
			println("$TOOL=TOOL_DATA[1]"); // defined with pendant
			println("$BASE=BASE_DATA[1]"); // defined with pendant
			println("PTP  XHOME");

			float[] ABC;
			ABC = new float[] { 90, 0, -90 };// Post "XYZABC" explains ABC.
			SPLINE(spline, ABC); // 1st  cut
			ABC = new float[] { 0, 90, 180 };
			SPLINE(reverse(spline), ABC); // 2nd cut

			println("PTP  XHOME");
			println("END");
			printer.close();
		} catch (Exception e) {
			e.printStackTrace();
		}
		System.out.println("src completed");
	}

	private static float[][] reverse(float[][] spline) {
		float[][] arr = new float[spline.length][];
		for (int i = 0; i < arr.length; i++)
			arr[i] = spline[arr.length - 1 - i];
		return arr;
	}

	private void SPLINE(float[][] spl, float[] abc) {
		println("$VEL.CP=0.12"); // velocity m/s
		PTP(spl[0], abc);
		println("SPLINE WITH $VEL.CP=0.12"); // velocity tuned for 5% speed (set in pendant) AUT mode
		if (external_axes)
			SPL(spl[0], abc, 0, 0);
		else
			SPL(spl[0], abc);
		for (int i = 1; i < spl.length; i++) {
			float[] xyz = spl[i];
			SPL(xyz);
		}
		println("ENDSPLINE");
	}

	private void PTP(float[] xyz, float[] abc) {
		println("PTP {X " + df.format(xyz[0]) + ",Y " + df.format(xyz[1]) + ",Z " + df.format(xyz[2]) + ",A " + df.format(abc[0]) + ",B " + df.format(abc[1]) + ",C "
				+ df.format(abc[2]) + "}");
	}

	private void SPL(float[] xyz) {
		println("SPL {X " + df.format(xyz[0]) + ",Y " + df.format(xyz[1]) + ",Z " + df.format(xyz[2])  + "}");
	}

	private void SPL(float[] xyz, float[] abc) {
		println("SPL {X " + df.format(xyz[0]) + ",Y " + df.format(xyz[1]) + ",Z " + df.format(xyz[2]) + ",A " + df.format(abc[0]) + ",B " + df.format(abc[1]) + ",C "
				+ df.format(abc[2]) + "}");
	}
	
	private void SPL(float[] xyz, float[] abc, float E1, float E2) {
		println("SPL {X " + df.format(xyz[0]) + ",Y " + df.format(xyz[1]) + ",Z " + df.format(xyz[2]) + ",A " + df.format(abc[0]) + ",B " + df.format(abc[1]) + ",C "
				+ df.format(abc[2]) + ",E1 " + df.format(E1) + ",E2 " + df.format(E2) + "}");
	}

	private void header() {
		printer.println("&ACCESS RVP");
		printer.println("&PARAM TEMPLATE = C:\\KRC\\Roboter\\Template\\vorgabe");
		printer.println("&PARAM EDITMASK = *");
		printer.println("DEF javakuka( )");
		printer.println(";FOLD INI;%{PE}");
		printer.println(";FOLD BASISTECH INI");
		printer.println("GLOBAL INTERRUPT DECL 3 WHEN $STOPMESS==TRUE DO IR_STOPM ( )");
		printer.println("INTERRUPT ON 3 ");
		printer.println("BAS (#INITMOV,0 )");
		printer.println(";ENDFOLD (BASISTECH INI)");
		printer.println(";FOLD USER INI\r");
		printer.println(";Make your modifications here");
		printer.println(";ENDFOLD (USER INI)");
		printer.println(";ENDFOLD (INI)");
		printer.println("");
	}

	private void println(String s) {
		if (windows)
			printer.println(s);
		else
			printer.println(s + "\r"); // for Mac
	}
}
