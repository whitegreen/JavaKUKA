package kuka;

import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.text.DecimalFormat;

//v. 29.12.2017 
//javakuka.org, whitegreen@163.com

public class KRLwriter {
	private boolean windows; // true:Window, false: Mac
	private DecimalFormat df = new DecimalFormat("###.#");
	private DecimalFormat pref = new DecimalFormat("###.###");
	private PrintWriter printer;

	public KRLwriter(boolean w, String filename, Integer tool, Integer base) {
		windows = w;
		try {
			printer = new PrintWriter(filename);
		} catch (FileNotFoundException e) {
			e.printStackTrace();
		}
		header();
		if (null != tool) {
			println("$TOOL=TOOL_DATA[" + tool + "]");
			println("$ACT_TOOL=" + tool);
		}
		if (null != base)
			println("$BASE=BASE_DATA[" + base + "]");
		println("PTP  XHOME");
	}
	public KRLwriter(boolean w, String filename) { //****
		windows = w;
		try {
			printer = new PrintWriter(filename);
		} catch (FileNotFoundException e) {
			e.printStackTrace();
		}
		header();
		println("PTP  XHOME2");
	}
	
	public void close() {
		println("PTP  XHOME");
		println("END");
		printer.close();
		System.out.println("src file completed");
	}
	public void close2() {
		println("PTP  XHOME2");
		println("END");
		printer.close();
		System.out.println("src file completed");
	}

	public void PTP6(float[] p) {
		println("PTP {X " + df.format(p[0]) + ",Y " + df.format(p[1]) + ",Z " + df.format(p[2]) + ",A " + df.format(p[3]) + ",B " + df.format(p[4]) + ",C " + df.format(p[5]) + "}");
	}

	public void PTP(float[] xyz, float[] abc) {
		println("PTP {X " + df.format(xyz[0]) + ",Y " + df.format(xyz[1]) + ",Z " + df.format(xyz[2]) + ",A " + df.format(abc[0]) + ",B " + df.format(abc[1]) + ",C "
				+ df.format(abc[2]) + "}");
	}

	public void PTP(float x, float y, float z, float a, float b, float c) {
		println("PTP {X " + df.format(x) + ",Y " + df.format(y) + ",Z " + df.format(z) + ",A " + df.format(a) + ",B " + df.format(b) + ",C " + df.format(c) + "}");
	}

	public void LIN_CDIS(double[] p, double[] abc) {
		println("LIN {X " + df.format(p[0]) + ",Y " + df.format(p[1]) + ",Z " + df.format(p[2]) + ",A " + df.format(abc[0]) + ",B " + df.format(abc[1]) + ",C " + df.format(abc[2])
				+ " } C_DIS");
	}

	public void LIN_CDIS(double[] p) {
		println("LIN {X " + df.format(p[0]) + ",Y " + df.format(p[1]) + ",Z " + df.format(p[2]) + "} C_DIS");
	}

	public void CIRC(float[] pa, float[] pb) {
		println("CIRC {X " + df.format(pa[0]) + ",Y " + df.format(pa[1]) + ",Z " + df.format(pa[2]) + "},{X " + df.format(pb[0]) + ",Y " + df.format(pb[1]) + ",Z "
				+ df.format(pb[2]) + "}");
	}

	public void CIRC_CDIS(double[] pa, double[] pb) {
		println("CIRC {X " + df.format(pa[0]) + ",Y " + df.format(pa[1]) + ",Z " + df.format(pa[2]) + "},{X " + df.format(pb[0]) + ",Y " + df.format(pb[1]) + ",Z "
				+ df.format(pb[2]) + " } C_DIS");
	}


	private String POS(double[] p) {
		return "{X " + df.format(p[0]) + ",Y " + df.format(p[1]) + ",Z " + df.format(p[2]) + ",A " + df.format(p[3]) + ",B " + df.format(p[4]) + ",C " + df.format(p[5]) + "}";
	}

	public void TOOL(double[] p) {
		println("$TOOL=" + POS(p));
	}

	public void TOOL(int i) {
		println("$TOOL=TOOL_DATA[" + i + "]");
		println("$ACT_TOOL=" + i);
	}

	public void BASE(int i) {
		println("$BASE=BASE_DATA[" + i + "]");
	}

	public void BASE(double[] p) {
		println("$BASE=" + POS(p));
	}

	public void VEL(float s) {
		float cp = 2 * s;
		float o1 = 200 * s;
		float o2 = 200 * s;
		println("$VEL.CP=" + pref.format(cp)); // velocity m/s
		println("$VEL.ORI1=" + pref.format(o1)); // velocity degree/s
		println("$VEL.ORI2=" + pref.format(o2));
	}

	public void header() {
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

	public void println(String s) {
		if (windows)
			printer.println(s);
		else
			printer.println(s + "\r"); // for Mac
	}
}
