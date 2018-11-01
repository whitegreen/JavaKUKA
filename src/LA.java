package kuka;

//v. 14.08.2017, javakuka.com
////whitegreen@163.com, hua@arch.ethz.ch

public class LA {  //linear algebra
	private static final double PI = Math.PI;
	//private static final double HP = 0.5 * Math.PI;
	
	//calculate matrix 
	public static double[][] matrix(double x, double y, double z, double aDeg, double bDeg, double cDeg) {
		//ABC: Euler angles, A: round z-axis     B: round y-axis        C: round y-axis
		double a = -aDeg * PI / 180;
		double b = -bDeg * PI / 180;
		double c = -cDeg * PI / 180;
		double ca = Math.cos(a);
		double sa = Math.sin(a);
		double cb = Math.cos(b);
		double sb = Math.sin(b);
		double cc = Math.cos(c);
		double sc = Math.sin(c);
		double[][] tt = new double[3][];
		tt[0] = new double[] { ca * cb, sa * cc + ca * sb * sc, sa * sc - ca * sb * cc, x };
		tt[1] = new double[] { -sa * cb, ca * cc - sa * sb * sc, ca * sc + sa * sb * cc, y };
		tt[2] = new double[] { sb, -cb * sc, cb * cc, z };
		return tt;
	}

	public static double[][] matrix(double aDeg, double bDeg, double cDeg) {
		//ABC: Euler angles, A: round z-axis     B: round y-axis        C: round y-axis
		double a = -aDeg * PI / 180;
		double b = -bDeg * PI / 180;
		double c = -cDeg * PI / 180;
		double ca = Math.cos(a);
		double sa = Math.sin(a);
		double cb = Math.cos(b);
		double sb = Math.sin(b);
		double cc = Math.cos(c);
		double sc = Math.sin(c);
		double[][] tt = new double[3][];
		tt[0] = new double[] { ca * cb, sa * cc + ca * sb * sc, sa * sc - ca * sb * cc };
		tt[1] = new double[] { -sa * cb, ca * cc - sa * sb * sc, ca * sc + sa * sb * cc};
		tt[2] = new double[] { sb, -cb * sc, cb * cc };
		return tt;
	}

	public static double[][] matrix_rad(double a, double b, double c) {
		double ca = Math.cos(a);
		double sa = Math.sin(a);
		double cb = Math.cos(b);
		double sb = Math.sin(b);
		double cc = Math.cos(c);
		double sc = Math.sin(c);
		double[][] tt = new double[3][];
		tt[0] = new double[] { ca * cb, sa * cc + ca * sb * sc, sa * sc - ca * sb * cc };
		tt[1] = new double[] { -sa * cb, ca * cc - sa * sb * sc, ca * sc + sa * sb * cc};
		tt[2] = new double[] { sb, -cb * sc, cb * cc };
		return tt;
	}
	
	public static double[][] matrixByQuaternion(double[] q) {
		double qw = q[0];
		double qx = q[1];
		double qy = q[2];
		double qz = q[3];
		double[][] m = new double[3][3];
		m[0] = new double[] { 1 - 2 * qy * qy - 2 * qz * qz, 2 * qx * qy - 2 * qz * qw, 2 * qx * qz + 2 * qy * qw };
		m[1] = new double[] { 2 * qx * qy + 2 * qz * qw, 1 - 2 * qx * qx - 2 * qz * qz, 2 * qy * qz - 2 * qx * qw };
		m[2] = new double[] { 2 * qx * qz - 2 * qy * qw, 2 * qy * qz + 2 * qx * qw, 1 - 2 * qx * qx - 2 * qy * qy };
		return m;
	}

	public static double[][] matrixBy3Axis(double[] dx, double[] dy, double[] dz) {
		//dx: vector (x,y,z) representing the new x-axis
		//dy: vector (x,y,z) representing the new y-axis
		//dz: vector (x,y,z) representing the new z-axis
		double[][] m = new double[3][3];
		m[0] = new double[] { dx[0], dy[0], dz[0] };
		m[1] = new double[] { dx[1], dy[1], dz[1] };
		m[2] = new double[] { dx[2], dy[2], dz[2] };
		return m;
	}
	
	public static double[][] matrixBy3Axis(double cx, double cy, double cz, double[] dx, double[] dy, double[] dz) {
		//dx: vector (x,y,z) representing the new x-axis
		//dy: vector (x,y,z) representing the new y-axis
		// dz: vector (x,y,z) representing the new z-axis
		double[][] m = new double[3][4];
		m[0] = new double[] { dx[0], dy[0], dz[0], cx };
		m[1] = new double[] { dx[1], dy[1], dz[1], cy };
		m[2] = new double[] { dx[2], dy[2], dz[2], cz };
		return m;
	}
	
	//calculate XYZABC
	public static double[] XYZABC(double[][] m) {   //3*4 matrix
		double[] t= ABC( m);
		double x = m[0][3];
		double y = m[1][3];
		double z = m[2][3];
		return new double[] { x, y, z, t[0], t[1], t[2] };
	}
	//calculate Euler angles
	public static double[] ABCby2Axis(double[] _dx, double[] _dxy) { //get Euler angles from two axes 
		//_dx:  x-axis    _dxy: a vector  on XY plane     (positive y)  
		//ABC: Euler angles, A: round z-axis     B: round y-axis        C: round y-axis
		double[][] axes= matrixby2Axis(_dx, _dxy);
		return ABC(axes);
	}
	

	public static double[] ABC(double[][] m) { //Euler angles from 3*3 matrix
		//ABC: Euler angles, A: round z-axis     B: round y-axis        C: round y-axis
		double sb = m[2][0];
		double cb;
		if (1 - sb * sb < 0)
			cb = 0;
		else
			cb = Math.sqrt(1 - sb * sb); 
		double ca = m[0][0];
		double sa = -m[1][0];
		double cc = m[2][2];
		double sc = -m[2][1];
		if (Math.abs(m[0][0]) < 1E-7 && Math.abs(m[1][0]) < 1E-7) {
			cc = m[1][1]; 
			sc =m[1][2]; 
		}
		double a = Math.atan2(sa, ca) * -180 / PI;
		double b = Math.atan2(sb, cb) * -180 / PI;
		double c = Math.atan2(sc, cc) * -180 / PI;
		return new double[] { a, b, c };
	}
	
	public static double[] ABCbyQuaternion(double[] q) { //Euler angles from Quaternion
		//ABC: Euler angles, A: round z-axis     B: round y-axis        C: round y-axis
		double qw = q[0];
		double qx = q[1];
		double qy = q[2];
		double qz = q[3];
		double sb = 2 * qx * qz - 2 * qy * qw;
		double ca = 1 - 2 * qy * qy - 2 * qz * qz;
		double sa = -(2 * qx * qy + 2 * qz * qw);
		double cc = 1 - 2 * qx * qx - 2 * qy * qy;
		double sc = -(2 * qy * qz + 2 * qx * qw);
		double a = Math.atan2(sa, ca) * -180 /PI;
		double b = Math.asin(sb) * -180 / PI;
		double c = Math.atan2(sc, cc) * -180 / PI;
		//singularity problems?************************************
		return new double[] { a, b, c };
	}
	
	public static double[] flipABC( double[] abc){
		return flipABC(abc[0], abc[1], abc[2]);
	}
	
	public static double[] flipABC( double a, double b, double c){
		double na= a>0? (a-180):(a+180);
		double nb= b>0?  (180-b):(-180-b);
		double nc= c>0? (c-180):(c+180);
		return new double[]{na, nb, nc};
	}
	
	//calculate  Quaternion
	public static double[] Quaternionby3Axis(double[] x, double[] y, double[] z){//ABB RAPID convention
		//x: vector (x,y,z) representing the new x-axis
		//y: vector (x,y,z) representing the new y-axis
		//z: vector (x,y,z) representing the new z-axis
		double q1 = 0.5 * Math.sqrt(x[0] + y[1] + z[2] + 1);
		double q2 = 0.5 * Math.sqrt(x[0] - y[1] - z[2] + 1);
		double q3 = 0.5 * Math.sqrt(y[1] - x[0] - z[2] + 1);
		double q4 = 0.5 * Math.sqrt(z[2] - x[0] - y[1] + 1);
		if(y[2]-z[1]<0)
			q2=-q2;
		if(z[0]-x[2]<0)
			q3=-q3;
		if(x[1]-y[0]<0)
			q4=-q4;
		return new double[]{q1,q2,q3,q4};
	}

	public static double[] QuaternionbyABC(double aDeg, double bDeg, double cDeg ){ //by Euler angles
		//ABC: Euler angles, A: round z-axis     B: round y-axis        C: round y-axis
	double[][] m= matrix(aDeg, bDeg, cDeg);
	double[] x={m[0][0], m[1][0],m[2][0]};
	double[] y={m[0][1], m[1][1],m[2][1]};
	double[] z={m[0][2], m[1][2],m[2][2]};
	return Quaternionby3Axis(x,y,z);
	}

	public static double[][] matrixby2Axis(double[] _dx, double[] _dxy){ //transpose of matrixBy3Axis()
		//_dx:  x-axis    _dxy: a vector  on XY plane  (positive y)  
		double[] dx= LA.normalize(_dx);
		double[] tt= LA.mul(dx, LA.dot(_dxy, dx));
		double[] _dy= LA.sub(_dxy, tt);
		double[] dy= LA.normalize(_dy);
		double[] dz= LA.cross(dx, dy);
		//double new double[][]{dx,dy,dz};
		double[][] m=new double[3][3];
		for (int i = 0; i < 3; i++) {
                 m[i][0]=dx[i];
                 m[i][1]=dy[i];
                 m[i][2]=dz[i];
		}
		return m;
	}

	public static double[][] matrixby2Axis(double cx, double cy, double cz, double[] _dx, double[] _dxy){ //transpose of matrixBy3Axis()
		//_dx:  x-axis    _dxy: a vector  on XY plane  (positive y)  
		double[] dx= LA.normalize(_dx);
		double[] tt= LA.mul(dx, LA.dot(_dxy, dx));
		double[] _dy= LA.sub(_dxy, tt);
		double[] dy= LA.normalize(_dy);
		double[] dz= LA.cross(dx, dy);
		//double new double[][]{dx,dy,dz};
		double[][] m=new double[3][4];
		for (int i = 0; i < 3; i++) {
                 m[i][0]=dx[i];
                 m[i][1]=dy[i];
                 m[i][2]=dz[i];
		}
		m[0][3]=cx;
		m[1][3]=cy;
		m[2][3]=cz;
		return m;
	}
	
	//basis algebra
	public static double dist(double[] a, double[] b) {
		double r = 0;
		for (int i = 0; i < a.length; i++)
			r += (a[i] - b[i]) * (a[i] - b[i]);
		return Math.sqrt(r);
	}
	
	public static double[] add(double[] a, double[] b) {
		double[] re=new double[a.length];
		for (int i = 0; i < a.length; i++)
			re[i]= a[i] + b[i];
		return re;
	}
	
	public static double[] add(double[] a,double sa,  double[] b, double sb) {
		double[] re=new double[a.length];
		for (int i = 0; i < a.length; i++)
			re[i]= sa*a[i] + sb*b[i];
		return re;
	}
	
	public static double[] between(double[] a, double[] b, double s) {
		double[] re=new double[a.length];
		for (int i = 0; i < a.length; i++)
			re[i]= a[i] *(1-s)+ b[i]*s;
		return re;
	}
	
	public static double[] sub(double[] a, double[] b) {
		double[] re = new double[a.length];
		for (int i = 0; i < a.length; i++)
			re[i] = a[i] - b[i];
		return re;
	}

	public static double mag(double[] a) {
		double r = 0;
		for (int i = 0; i < a.length; i++)
			r += a[i] * a[i];
		r=Math.sqrt(r);
		return r;
	}
	
	public static double[] normalize(double[] a) {
		double r = mag(a);
		double[] re=new double[a.length];
		for (int i = 0; i < a.length; i++)
			re[i]= a[i]/r;
		return re;
	}
	
	public static double dot(double[] a, double[] b) {
		double re = 0;
		for (int i = 0; i < a.length; i++)
			re+= a[i] * b[i];
		return re;
	}
	
	public static double[] mul(double[] a, double s) {
		double[] re=new double[a.length];
		for (int i = 0; i < a.length; i++)
			re[i]= a[i] *s;
		return re;
	}
	
	public static double[] cross(double[] b, double[] c) {
		double[] re = new double[b.length];
		re[0] = b[1] * c[2] - b[2] * c[1];
		re[1] = b[2] * c[0] - b[0] * c[2];
		re[2] = b[0] * c[1] - b[1] * c[0];
		return re;
	}
	//*********************for 3*4 matrices
	public static double[][] mul(double[][] a, double[][] b) { // multiple two 3*4 matrices, assuming the 4th row is {0,0,0,1}
			double[][] re = new double[3][4];
			for (int i = 0; i < 3; i++) {
				for (int j = 0; j < 4; j++) {
					double b3j = (j == 3 ? 1 : 0);
					re[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j] + a[i][3] * b3j;
				}
		}
		return re;
	}

	public static double[][] transpose(double[][] a) {
		double[][] b = new double[a[0].length][a.length];
		for (int i = 0; i < a.length; i++) {
			for (int j = 0; j < a.length; j++)
				b[j][i] = a[i][j];
		}
		return b;
	}

	public static double[] mul(double[][] a, double[] b) { // multiple a 3*4 (or 3*3) matrix with a vector
		double[] re = new double[3];
		int len = a[0].length;
		if (len == 4) {
			for (int i = 0; i < 3; i++)
				re[i] = a[i][0] * b[0] + a[i][1] * b[1] + a[i][2] * b[2] + a[i][3];
		}
		else if (len == 3) {
			for (int i = 0; i < 3; i++)
				re[i] = a[i][0] * b[0] + a[i][1] * b[1] + a[i][2] * b[2];
		}
		return re;

	}
	
	public static double[][] inverse(double[][] m) { //inverse a 3*4 matrice whose det=1,  assuming 4th row  is {0,0,0,1}
		double[][] v = new double[3][4];
		v[0][0] = -m[1][2] * m[2][1] + m[1][1] * m[2][2];
		v[0][1] = m[0][2] * m[2][1] - m[0][1] * m[2][2];
		v[0][2] = -m[0][2] * m[1][1] + m[0][1] * m[1][2];
		v[0][3] = m[0][3] * m[1][2] * m[2][1] - m[0][2] * m[1][3] * m[2][1] - m[0][3] * m[1][1] * m[2][2] + m[0][1] * m[1][3] * m[2][2] + m[0][2] * m[1][1] * m[2][3] - m[0][1]
				* m[1][2] * m[2][3];
		v[1][0] = m[1][2] * m[2][0] - m[1][0] * m[2][2];
		v[1][1] = -m[0][2] * m[2][0] + m[0][0] * m[2][2];
		v[1][2] = m[0][2] * m[1][0] - m[0][0] * m[1][2];
		v[1][3] = m[0][2] * m[1][3] * m[2][0] - m[0][3] * m[1][2] * m[2][0] + m[0][3] * m[1][0] * m[2][2] - m[0][0] * m[1][3] * m[2][2] - m[0][2] * m[1][0] * m[2][3] + m[0][0]
				* m[1][2] * m[2][3];
		v[2][0] = -m[1][1] * m[2][0] + m[1][0] * m[2][1];
		v[2][1] = m[0][1] * m[2][0] - m[0][0] * m[2][1];
		v[2][2] = -m[0][1] * m[1][0] + m[0][0] * m[1][1];
		v[2][3] = m[0][3] * m[1][1] * m[2][0] - m[0][1] * m[1][3] * m[2][0] - m[0][3] * m[1][0] * m[2][1] + m[0][0] * m[1][3] * m[2][1] + m[0][1] * m[1][0] * m[2][3] - m[0][0]
				* m[1][1] * m[2][3];
		return v;
	}
	//*********************for 3*4 matrices
}
