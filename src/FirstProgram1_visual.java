package javakuka;

import peasy.PeasyCam;
import processing.core.PApplet;
//v. 22.06.2016
//javakuka.com, whitegreen@163.com
//external lib: 1. Processing 1.5 core; 2. PeasyCam, both available in javakuka.com. 

public class FirstProgram1_visual extends PApplet {  
    private final int  num_pts=15*8;
    private final float X=298; //foam's length in the x-axis of BASE.
    private float[][] spline;
	
	public void setup(){
		size(800,600,P3D);
		new PeasyCam(this,200);
		
		float Y0 = 23;
		float Y1 = 17;
		spline = new float[num_pts][];
	    float margin=0.2F;
		for (int i = 0; i < num_pts; i++) {
			float theta = (float) (i * 7 * PI) / (num_pts - 1);
			float x = -margin * X + i * (X+2*margin*X) / (num_pts - 1);
			float y = Y0 + Y1 * cos(theta-PI/2);
			float z = -Y0 - Y1 * cos(theta+PI);
			float[] point = { x, y, z };
			spline[i] = point;
		}
	}

	public void draw() {
		smooth();
		background(255);
		scale(1, 1, -1);

		strokeWeight(1); 
		stroke(200);
		fill(200,100);
		pushMatrix();
		translate(X/2, 100, -80);
		box(X, 200, 160);
		popMatrix();
		
		strokeWeight(1); 
		stroke(0);
		for(float[] p: spline){  //draw tool path
			line(p[0], p[1], p[2], p[0], 0, p[2] );
			line(p[0], p[1], p[2], p[0], p[1], 0);
		}

		strokeWeight(3); //draw BASE 
		stroke(255, 0, 0);
		line(0, 0, 0, 100, 0, 0);
		stroke(0, 255, 0);
		line(0, 0, 0, 0, 100, 0);
		stroke(0, 0, 255);
		line(0, 0, 0, 0, 0, 100);
	}
}
