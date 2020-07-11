package testForward;

import java.io.BufferedReader;
import java.io.IOException;
import java.util.ArrayList;
import kinematik.LA;
import kinematik.Robot;
import peasy.PeasyCam;
import processing.core.PApplet;
//v. 07.07.2016
//javakuka.com, whitegreen@163.com
//external lib: 1. Processing 1.5 core; 2. PeasyCam, both available in javakuka.com. 

public class DisplayKuka extends PApplet {
	private int tid = 2;
	private double[] degs;
	private double[][][] T ; //static
	private double[][][] M = new double[6][][]; //sequence
	private  float[][][] arms=new float[7][][];
    private Robot kr;
	
	public void setup() {
		size(900, 800, P3D);
		new PeasyCam(this, 300);
		
		for (int i = 0; i < 7; i++)
			arms[i] = read("/KUKA_KR6_R900/m" + i + ".stl"); 
		//download the 3D model from javakuka.com and put it in bin folder (in Eclipse) 
		
		degs = new double[]{0, -90, 90, 0, 0, 0};
//		degs = new double[]{10, -80, 70, 10, -20, 30};
		double[] a = { 25, 455, 35, 0,  0, 0 };   
		double[] d = { 400,  0,   0, 420, 0, 80 };  
		kr=new Robot(a,d);
		double[] rad= Robot.DegToRad(new double[]{0,0,0,0,0,0}, "KUKA");
		T=kr.forwardSequence(rad);
		for(int i=0;i<6;i++){
			T[i]= LA.inverse(T[i]);
		}
		M=kr.forwardSequence(Robot.DegToRad(degs, "KUKA"));
	}

	private float[][] read(String file) {  //input the geometry
		BufferedReader br = createReader(file);
		String line;
		ArrayList<float[]> list = new ArrayList<float[]>();
		float[][] ps = null;
		try {
			while ((line = br.readLine()) != null) {
				String[] dat = line.split(" + ");
				if (dat.length >= 3) {
					float x = Float.parseFloat(dat[2]);
					float y = Float.parseFloat(dat[3]);
					float z = Float.parseFloat(dat[4]);
					list.add(new float[] { x, y, z });
				}
			}
			ps = new float[list.size()][];
			for (int i = 0; i < ps.length; i++) {
				ps[i] = list.get(i);
			}
			br.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
		return ps;
	}

	private static float[] mul34(double[][] a, float[] b) { // 3*4 matrix
		float[] re = new float[3];
		for (int i = 0; i < 3; i++)
			re[i] = (float) (a[i][0] * b[0] + a[i][1] * b[1] + a[i][2] * b[2] + a[i][3]);
		return re;
	}

	public void draw() {
		background(255);
		lights();
		scale(0.2f, -0.2f, 0.2f);
		translate(0,0,-400);
		stroke(0);
		float[] ep = { 0, 0, 0 };
		float h = 120;
		for (int i = 0; i < 6; i++) {
			float[][] pas = { { 0, 0, 0 }, { h, 0, 0 }, { 0, h, 0 }, { 0, 0, h } };
			float[][] pbs = new float[pas.length][];
			for (int j = 0; j < pas.length; j++)
				pbs[j] = mul34(M[i], pas[j]);
			stroke(0);
			line(pbs[0], ep);
			stroke(255, 0, 0);
			line(pbs[0], pbs[1]);
			stroke(0, 255, 0);
			line(pbs[0], pbs[2]);
			stroke(0, 0, 255);
			line(pbs[0], pbs[3]);
			ep = pbs[0].clone(); // for next
		}

		h = 300;
		strokeWeight(2);
		stroke(255, 0, 0);
		line(h, 0, 0, 0, 0, 0);
		stroke(0, 255, 0);
		line(0, h, 0, 0, 0, 0);
		stroke(0, 0, 255);
		line(0, 0, h, 0, 0, 0);
		
		noStroke();
		float[][] arm = arms[0];
		beginShape(TRIANGLES);
		for (int i = 0; i < arm.length; i++) {
			float[] v = arm[i];
			vertex(v[0], v[1], v[2]);
		}
		endShape();

		for (int j = 1; j < 7; j++) {
			if (j % 2 == 1)
				fill(255, 80, 0);
			else
				fill(200);
			double[][] mat = LA.mul(M[j - 1], T[j - 1]);
			arm = arms[j];
			if (arm == null)
				continue;
			beginShape(TRIANGLES);
			for (int i = 0; i < arm.length; i++) {
				float[] v = arm[i];
				v = mul34(mat, v);
				vertex(v[0], v[1], v[2]);
			}
			endShape();
		}
	}

	public void line(float[] a, float[] b) {
		line(a[0], a[1], a[2], b[0], b[1], b[2]);
	}

	public void keyPressed() {
		double s = 1;
		if (keyCode == UP) {
			tid = (tid + 1) % 6;
		} else if (keyCode == DOWN) {
			tid = (tid + 5) % 6;
		}
		if (keyCode == LEFT) {
			degs[tid] += s;
			M = kr.forwardSequence(Robot.DegToRad(degs, "KUKA"));
		} else if (keyCode == RIGHT) {
			degs[tid] -= s;
			M = kr.forwardSequence(Robot.DegToRad(degs, "KUKA"));
		}
		println("joint "+(tid + 1));
	}

}
