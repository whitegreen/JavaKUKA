package testInverse;

import java.text.DecimalFormat;
import kinematik.Robot;
//v. 07.07.2016
//javakuka.com, whitegreen@163.com
public class TestInverse {

	public static void main(String[] args) {
		new TestInverse();
	}

	public TestInverse() {
		double[] a = { 25, 455, 35, 0, 0, 0 };
		double[] d = { 400, 0, 0, 420, 0, 80 };
		Robot kr = new Robot(a, d); // KR6 R900
		
		double[] position = { 525, 0, 890, 0, 90, 0 }; // xyzabc, HOME position,  flange's position in WORLD
		double[] theta = kr.inverse(position, new Double(0), null, null); 
		double[] angles = Robot.RadToDeg(theta, "KUKA");
		
		double[][] kuka_soft_lims = { { -170, 170 }, { -190, 45 }, { -120, 156 }, { -185, 185 }, { -120, 120 }, { -350, 350 } };
		double[][] java_lims = { { -170, 170 }, { -190, 45 }, { 5, 156 }, { -185, 185 }, { -120, 120 }, { -180, 180 } };
		DecimalFormat df = new DecimalFormat("###.##");
		for (int i = 0; i < angles.length; i++) {
			if (angles[i] < java_lims[i][0])
				angles[i] += 360;
			else if (angles[i] > java_lims[i][01])
				angles[i] -= 360;
			System.out.println("A" + (i + 1) + ": " + df.format(angles[i]));
		}
	}

}
