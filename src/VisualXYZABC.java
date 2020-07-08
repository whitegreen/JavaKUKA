package kinematik;

import peasy.PeasyCam;
import processing.core.PApplet;

//external lib: 1. Processing 1.5; 2. PeasyCam, both can be downloaded from javakuka.com. 

public class VisualXYZABC extends PApplet { //PApplet (Processing) for 3D display
	private final int len = 50;
	private static final float sz = -1; // Kuka uses right-handed coordinates, while Processing left-handed.
	private double[] TCP = { 50, 50, 50 };
	private double[] xaxis_tool = { len, 0, 0 };
	private double[] yaxis_tool = { 0, len, 0 };
	private double[] zaxis_tool = { 0, 0, len };
	private double[] xaxis_tool_rotated, yaxis_tool_rotated, zaxis_tool_rotated;

	public void setup() {
		size(400, 400, P3D);
		double[][] matrix = LA.matrix(TCP[0], TCP[1], TCP[2], 90, -90, -90);
		xaxis_tool_rotated = LA.mul(matrix, xaxis_tool);
		yaxis_tool_rotated = LA.mul(matrix, yaxis_tool);
		zaxis_tool_rotated = LA.mul(matrix, zaxis_tool);
		new PeasyCam(this, 300);
	}

	public void draw() {
		smooth();
		background(255);

		// draw the BASE frame
		noFill();
		stroke(180);
		strokeWeight(1);
		pushMatrix();
		translate(len, len, sz * len);
		box(2 * len, 2 * len, sz * 2 * len);
		popMatrix();
		fill(0);
		text("x", 2 * len, 0, 0);
		text("y", 0, 2 * len, 0);
		text("z", 0, 0, sz * 2 * len);

		// draw tool
		strokeWeight(4);
		stroke(255, 0, 0);
		fill(255, 0, 0);
		dotline(TCP, xaxis_tool_rotated);
		text("x", xaxis_tool_rotated);

		stroke(0, 255, 0);
		fill(0, 255, 0);
		dotline(TCP, yaxis_tool_rotated);
		text("y", yaxis_tool_rotated);

		stroke(0, 0, 255);
		fill(0, 0, 255);
		dotline(TCP, zaxis_tool_rotated);
		text("z", zaxis_tool_rotated);
	}

	private void dotline(double[] a, double[] b) {
		int n = 10;
		for (int i = 0; i < n; i++) {
			float s0 = (i + 0F) / n;
			float s1 = (i + 0.7F) / n;
			line(between(s0, a, b), between(s1, a, b));
		}
	}

	private void text(String s, double[] v) {
		text(s, 4 + (float) v[0], -4 + (float) v[1], sz * (float) v[2]);
	}

	private void line(double[] a, double[] b) {
		line((float) a[0], (float) a[1], sz * (float) a[2], (float) b[0], (float) b[1], sz * (float) b[2]);
	}

	private static double[] between(double s, double[] a, double[] b) {
		double[] v = new double[a.length];
		for (int i = 0; i < a.length; i++)
			v[i] = (1 - s) * a[i] + s * b[i];
		return v;
	}
}
