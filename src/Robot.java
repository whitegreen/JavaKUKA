package kuka;

//6-axis Robot kinematics
//v. 22.06.2016,
//javakuka.org
//Hao Hua, Southeast University, whitegreen@163.com

public class Robot {
	private static final double PI = Math.PI;
	private static final double HP = 0.5 * Math.PI;
	private final double[] a,d;   //DH parameters
	private final double l2, ad2; //for inverse kinematics
	
	public Robot(double[] a, double[] d) { // a,d: DH parameters describe the robot's geometry
		this.a=a;
		this.d=d;
		l2 = Math.sqrt(a[2] * a[2] + d[3] * d[3]);
		ad2 = Math.atan2(a[2], d[3]);
	}
	
	public double[] inverse(double[] v, Double A4_deg, double[][] base, double[][] tool) {
		double[][] Pos = LA.matrix(v[0], v[1], v[2], v[3], v[4], v[5]);
		double[][] goal;
		if(base==null)
			goal=Pos;
		else
			goal= LA.mul(base, Pos); // in WORLD frame BASE*POS= GOAL
		double[][] T6;
		if (tool == null) 
			T6 = goal;
		else 
			T6 = LA.mul(goal,  LA.inverse(tool)); // T6*TOOL = GOAL= BASE*POS(w.r.t. BASE)
		double[] inreDeg = inverse(T6, A4_deg);// (T6, thetaDeg[3]);
		return inreDeg;
	}
	
	public double[][] forward(double[] ts) {  //arguments: joint angles in radius
		double[] c=new double[6];
		double[] s=new double[6];
		for(int i=0;i<6;i++){
			c[i]= Math.cos(ts[i]);
			s[i]= Math.sin(ts[i]);
		}
		double[][] m123=new double[3][];
		m123[0] = new double[] { c[0] * (c[1] * c[2] - s[1] * s[2]),     s[0],   c[0] * (c[1] * s[2] + s[1] * c[2]),      c[0] * (a[2] * (c[1] * c[2] - s[1] * s[2]) + a[1] * c[1]) + a[0] * c[0] };
		m123[1] = new double[] { s[0]*  (c[1] * c[2] - s[1] * s[2]),   -c[0],    s[0] * (c[1] * s[2] + s[1] * c[2]),      s[0]* (a[2] * (c[1] * c[2] - s[1] * s[2]) + a[1] * c[1]) + a[0] * s[0]};
		m123[2] = new double[] { s[1] * c[2] +c[1] * s[2],                0,    s[1] * s[2] - c[1] * c[2],                 a[2] * (s[1] * c[2] + c[1] * s[2]) + a[1] * s[1] +d[0]};
		double[][] m456=new double[3][];
		m456[0] = new double[] {c[3]*c[4]*c[5]-s[3]*s[5],   -c[3]*c[4]*s[5]-s[3]*c[5]     , c[3] * s[4],     c[3]*s[4]*d[5] };
		m456[1] = new double[] {s[3]*c[4]*c[5]+c[3]*s[5],   -s[3]*c[4]*s[5]+c[3]*c[5]     , s[3] * s[4],     s[3]*s[4]*d[5] };
		m456[2] = new double[] { -s[4]*c[5],                  s[4]*s[5],                     c[4],             c[4]*d[5]+d[3]};
		double[][] arr = LA.mul(m123, m456);
		return arr;
	}
	
	public double[][][] forwardSequence(double[] ts) {// arguments: joint angles in radius
		double[] c = new double[6];
		double[] s = new double[6];
		for (int i = 0; i < 6; i++) {
			c[i] = Math.cos(ts[i]);
			s[i] = Math.sin(ts[i]);
		}
		double[][] a0 = new double[3][];
		a0[0] = new double[] { c[0], 0, s[0], a[0] * c[0] };
		a0[1] = new double[] { s[0], 0, -c[0], a[0] * s[0] };
		a0[2] = new double[] { 0, 1, 0, d[0] };
		double[][] a1 = new double[3][];
		a1[0] = new double[] { c[1], -s[1], 0, a[1] * c[1] };
		a1[1] = new double[] { s[1], c[1], 0, a[1] * s[1] };
		a1[2] = new double[] { 0, 0, 1, 0 };
		double[][] a2 = new double[3][];
		a2[0] = new double[] { c[2], 0, s[2], a[2] * c[2] };
		a2[1] = new double[] { s[2], 0, -c[2], a[2] * s[2] };
		a2[2] = new double[] { 0, 1, 0, 0 };
		double[][] a3 = new double[3][];
		a3[0] = new double[] { c[3], 0, -s[3], 0 };
		a3[1] = new double[] { s[3], 0, c[3], 0 };
		a3[2] = new double[] { 0, -1, 0, d[3] };
		double[][] a4 = new double[3][];
		a4[0] = new double[] { c[4], 0, s[4], 0 };
		a4[1] = new double[] { s[4], 0, -c[4], 0 };
		a4[2] = new double[] { 0, 1, 0, 0 };
		double[][] a5 = new double[3][];
		a5[0] = new double[] { c[5], -s[5], 0, 0 };
		a5[1] = new double[] { s[5], c[5], 0, 0 };
		a5[2] = new double[] { 0, 0, 1, d[5] };
		double[][][] M = new double[6][][];
		M[0] = a0;
		M[1] = LA.mul(M[0], a1);
		M[2] = LA.mul(M[1], a2);
		M[3] = LA.mul(M[2], a3);
		M[4] = LA.mul(M[3], a4);
		M[5] = LA.mul(M[4], a5);
		return M;
	}

	public double[] forward(double[] degs, double[][] base, double[][] tool) {
		double[][] T6 = forward(degs);
		double[][] goal;
		if (tool == null) {
			goal = T6;
		} else {
			goal = LA.mul(T6, tool);
		}
		double[][] pos;
		if (base == null) {
			pos = goal;
		} else {
			double[][] inbase = LA.inverse(base);
			pos = LA.mul(inbase, goal);
		}
		double[] as = LA.ABC(pos);
		return new double[] { pos[0][3], pos[1][3], pos[2][3], as[0], as[1], as[2] };
	}

	public double[] inverse(double[][] T6, Double A4) { //A4 in radius (could be null), return joint angles in radius
		double[] theta=new double[6]; 
		double[] center=  LA.mul(T6, new double[]{0,0, -d[5]});
		theta[0] = Math.atan2(center[1], center[0]); // or -atan2     choice one possibility

		double ll = Math.sqrt(center[0] * center[0] + center[1] * center[1]);
		double[] p1 = { a[0] * center[0] / ll, a[0] * center[1] / ll, d[0] };
		double l3 =LA.dist(center, p1);
		double l1 = a[1];
		double beta = Math.acos((l1 * l1 + l3 * l3 - l2 * l2) / (2 * l1 * l3));
		double ttl = Math.sqrt((center[0] - p1[0]) * (center[0] - p1[0]) + (center[1] - p1[1]) * (center[1] - p1[1]));
		if (p1[0] * (center[0] - p1[0]) < 0) // opposite side
			ttl = -ttl;
		double al = Math.atan2(center[2] - p1[2], ttl);
		theta[1] =beta+al; // choice one possibility
		double gama = Math.acos((l1 * l1 + l2 * l2 - l3 * l3) / (2 * l1 * l2));
		theta[2] = gama - ad2 - HP;

		double[][] arr = new double[4][];
		double[] c = new double[3];
		double[] s=new double[3];
		for(int i=0;i<3;i++){
			c[i]= Math.cos(theta[i]);
			s[i]= Math.sin(theta[i]);
		}
		arr[0] = new double[] { c[0] * (c[1] * c[2] - s[1] * s[2]),     s[0],   c[0] * (c[1] * s[2] + s[1] * c[2]),      c[0] * (a[2] * (c[1] * c[2] - s[1] * s[2]) + a[1] * c[1]) + a[0] * c[0] };
		arr[1] = new double[] { s[0]*  (c[1] * c[2] - s[1] * s[2]),   -c[0],    s[0] * (c[1] * s[2] + s[1] * c[2]),      s[0]* (a[2] * (c[1] * c[2] - s[1] * s[2]) + a[1] * c[1]) + a[0] * s[0]};
		arr[2] = new double[] { s[1] * c[2] +c[1] * s[2],                0,    s[1] * s[2] - c[1] * c[2],                 a[2] * (s[1] * c[2] + c[1] * s[2]) + a[1] * s[1] +d[0]};
		double[][] in123= LA.inverse(arr);
		double[][] mr = LA.mul(in123, T6);//***
		double c5 = mr[2][2];
		if (Math.abs(c5 - 1) < 0.00000001) { //singularity
			//double A4=-PI*A4_deg/180; //see deg_rad
			System.out.println("****");
			double c4 = Math.cos(A4);
			double s4 = Math.sin(A4);
			double s6 = c4 * mr[1][0] - s4 * mr[0][0];
			double c6;
			if (Math.abs(c4) > Math.abs(s4))
				c6 = (mr[0][0] + s4 * s6) / c4;
			else
				c6 = (mr[1][0] - c4 * s6) / s4;
			theta[3] = A4;
			theta[4] = 0;
			theta[5] =   Math.atan2(s6, c6);
			if (Math.abs(c6) > 1 || Math.abs(s6) > 1)
				throw new RuntimeException();
		} else {
			double ang = Math.atan2(mr[1][2], mr[0][2]);
			theta[3] = ang;
			theta[4] = Math.acos(c5); // *********
			theta[5] = Math.atan2(mr[2][1], -mr[2][0]);
		}
		return theta;
	}
	
	public static double[] flipAxis456(double[] deg){ //in radius
		double[] nds = { deg[0], deg[1], deg[2], 0, -deg[4], 0 };
		nds[3] = deg[3] > 0 ? (deg[3] - PI) : (deg[3] + PI);
		nds[5] = deg[5] > 0 ? (deg[5] - PI) : (deg[5] + PI);
		return nds;
	}
	
	public static double[] DegToRad(double[] ds, String robotType) {// "KUKA", "ABB", "ABB2"
		double[] rd = new double[6];
		if (robotType.equals("KUKA")) {
			for (int i = 0; i < 6; i++)
				rd[i] = ds[i] * PI / 180;
			rd[2] -= HP;
			rd[5] += PI;
			for (int i = 0; i < 6; i++)
				rd[i] = -rd[i];
		} else if (robotType.equals("ABB")) {
			for (int i = 0; i < 6; i++)
				rd[i] = ds[i] * Math.PI / 180;
			rd[1] -= HP;
			rd[5] -= PI;
			rd[1] = -rd[1];
			rd[2] = -rd[2];
			rd[4] = -rd[4];
		} else if (robotType.equals("ABB2")) { // such as IRB2400 IRB4400
			for (int i = 0; i < 6; i++)
				rd[i] = ds[i] * Math.PI / 180;
			rd[2] -= rd[1];
			rd[1] -= HP;
			rd[5] -= PI;
			rd[1] = -rd[1];
			rd[2] = -rd[2];
			rd[4] = -rd[4];
		}
		return rd;
	}

	public static double[] RadToDeg(double[] ds, String robotType) {// "KUKA", "ABB", "ABB2"
		double[] rd = new double[6];
		if (robotType.equals("KUKA")) {
			for (int i = 0; i < 6; i++)
				rd[i] = -ds[i];
			rd[2] += HP;
			rd[5] -= PI;
			for (int i = 0; i < 6; i++)
				rd[i] = rd[i] * 180 / PI;
		} else if (robotType.equals("ABB")) {
			rd[1] = -ds[1];
			rd[2] = -ds[2];
			rd[4] = -ds[4];
			rd[0] = ds[0];
			rd[3] = ds[3];
			rd[5] = ds[5];
			rd[1] += HP;
			rd[5] += PI;
			for (int i = 0; i < 6; i++)
				rd[i] = rd[i] * 180 / PI;
		} else if (robotType.equals("ABB2")) { // such as IRB2400 IRB4400
			rd[1] = -ds[1];
			rd[2] = -ds[2];
			rd[4] = -ds[4];
			rd[0] = ds[0];
			rd[3] = ds[3];
			rd[5] = ds[5];
			rd[1] += HP;
			rd[5] += PI;
			rd[2] += rd[1];
			for (int i = 0; i < 6; i++)
				rd[i] = rd[i] * 180 / PI;
		}
		return rd;
	}

}
