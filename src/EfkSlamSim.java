import java.util.ArrayList;
import java.util.Random;

import org.opencv.core.*;

public class EfkSlamSim 
{
	/**
	 * @param args
	 */
	public static void main(String[] args) 
	{
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
		test();
	}
	/**
	 * @param args
	 */
	public static void test()
	{
		//INPUTS:
		//set of landmarks
		//set of waypoints
		
		//***for testing only***
		Mat wp = new Mat(2, 4, CvType.CV_64F); //initialize 4 waypoints
		//initial wp = [x1 x2 x3 ... xn]
		//             [y1 y2 y3 ... yn]
		Mat lm = new Mat(2, 3, CvType.CV_64F); //initialize 3 landmarks
		//initial lm = [x1 x2 x3 ... xn]
		//             [y1 y2 y3 ... yn]
		
		//1st waypoint
		wp.put(0, 0, 0);
		wp.put(1, 0, 0);
		//2nd waypoint
		wp.put(0, 1, -19.4700);
		wp.put(1, 1, -20.1780);
		//3rd waypoint
		wp.put(0, 2, -30.0691);
		wp.put(1, 2, 19.2878);
		//4th waypoint
		wp.put(0, 3, 0.1152);
		wp.put(1, 3, 39.4659);
		
		//1st landmark
		lm.put(0, 0, -19.7005);
		lm.put(1, 0, 0.2967);
		//2nd landmark
		lm.put(0, 1, -19.4700);
		lm.put(1, 1, 19.5846);
		//3rd landmark
		lm.put(0, 2, 0.3456);
		lm.put(1, 2, 19.5846);
		//***for testing only***
		
		//setup plots
		Mat veh = new Mat(2, 3, CvType.CV_64F);
		veh.put(0, 0, 0);
		veh.put(0, 1, -ConfigFile.WHEELBASE);
		veh.put(0, 2, -ConfigFile.WHEELBASE);
		veh.put(1, 0, 0);
		veh.put(1, 1, -2);
		veh.put(1, 2, 2);
		//initial veh = [0 -WB -WB]
		//              [0 -2   2 ]
		
		System.out.println("veh");
		System.out.println(veh.dump());
		System.out.println();
		
		//initialize states
		Mat xtrue = Mat.zeros(3, 1, CvType.CV_64F);
		//initial xtrue = [0]
		//                [0]
		//                [0]
		
		System.out.println("xtrue");
		System.out.println(xtrue.dump());
		System.out.println();
		
		Mat x = Mat.zeros(3, 1, CvType.CV_64F);
		//initial x = [0] xr
		//            [0] yr
		//            [0] steer angle
		
		System.out.println("x");
		System.out.println(x.dump());
		System.out.println();
		
		Mat P = Mat.zeros(3, 3, CvType.CV_64F);
		//initial P = [0 0 0]
		//            [0 0 0]
		//            [0 0 0]
		
		System.out.println("P");
		System.out.println(P.dump());
		System.out.println();
		
		//initialize other variables and constants
		double dt = ConfigFile.DT_CONTROLS;//change in time between predicts
		double dtsum = 0;//change in time since last observation
		Mat ftag= new Mat(1, lm.cols(), CvType.CV_64F); // identifier for each landmark
		//initial ftag = [lm1 lm2 lm3 ... lmn]
		
		for(int i = 0; i < lm.cols(); i++)
		{
			ftag.put(0, i, i);
		}
		
		Mat da_table= Mat.zeros(1, lm.cols(), CvType.CV_64F); // data association table
		//initial da_table = [lm1 lm2 lm3 ... lmn]
		
		int iwp = 0; //index to first waypoint (0 based index)
		double G = 0; //initial steer angle
		Mat QE= ConfigFile.Q(); 
		//initial QE = [SigmaV^2       0]
		//             [0       SigmaG^2]
		
		Mat RE= ConfigFile.R(); 
		//initial RE = [SigmaR^2       0]
		//             [0       SigmaB^2]
		
		if (ConfigFile.SWITCH_INFLATE_NOISE)
		{
			Core.multiply(ConfigFile.Q(), new Scalar(2), QE);
			Core.multiply(ConfigFile.R(), new Scalar(8), RE);
			//inflate estimated noises (ie, add stabilising noise)
		}
		
		int iteration = 0;
		
		while(iwp != -1)
		{
			iteration++;
			//compute true data
			Object[] retValue = compute_steering(xtrue, wp, iwp, ConfigFile.AT_WAYPOINT, G, ConfigFile.RATEG, ConfigFile.MAXG, dt);
			G = (double)retValue[0];
			iwp = (int)retValue[1];
			
			if (iwp == -1 && ConfigFile.NUMBER_LOOPS > 1) //perform loops: if final waypoint reached, go back to first
			{
				iwp=0; 
				ConfigFile.NUMBER_LOOPS = ConfigFile.NUMBER_LOOPS - 1; 
			}
			
			xtrue= vehicle_model(xtrue, ConfigFile.V, G, ConfigFile.WHEELBASE, dt);
			
			double Vn;
			double Gn;
			double[] retValue1 = add_control_noise(ConfigFile.V, G, ConfigFile.Q(), ConfigFile.SWITCH_CONTROL_NOISE);
			Vn = retValue1[0];
			Gn = retValue1[1];
			
			//EKF predict step
		    Mat[] retValue2 = predict (x,P, Vn, Gn, QE, ConfigFile.WHEELBASE, dt);
		    x = retValue2[0];
		    P = retValue2[1];
		    
		    //if heading known, observe heading
			Mat[] retValue3 = observe_heading(x, P, xtrue.get(2, 0)[0], ConfigFile.SWITCH_HEADING_KNOWN);
		    
		    //EKF update step
		    dtsum = dtsum + dt;
		    if(dtsum >= ConfigFile.DT_OBSERVE)
		    {
		    	dtsum = 0;
		    	
		    	Mat[] retValue4 = get_observations(xtrue, lm, ftag, ConfigFile.MAX_RANGE);
		    	Mat z = retValue4[0];
		    	Mat ftag_visible = retValue4[1];
		    	
		    	Mat zf = null;
		    	Mat idf = null;
		    	Mat zn = null;
		    	
		    	z = add_observation_noise(z, ConfigFile.R(), ConfigFile.SWITCH_SENSOR_NOISE);
		    	
		    	//compute data association (association not known, estimated using gates)
	    		Mat[] retValue5 = data_associate(x, P, z, RE, ConfigFile.GATE_REJECT, ConfigFile.GATE_AUGMENT); 
	    		zf = retValue5[0];
	    		idf = retValue5[1];
	    		zn = retValue5[2];
		    	
		    	//update step
	    		Mat[] retValue6 = update(x, P, zf, RE, idf); 
	    		x = retValue6[0];
	    		P = retValue6[1];
		    	
				//augment step
		    	Mat[] retValue7 = augment(x, P, zn, RE); 
		    	x = retValue7[0];
	    		P = retValue7[1];
		    }
		    
		    System.out.println("iteration: " + iteration);
		    System.out.println("============");
		    
		    System.out.println("xtrue");
			System.out.println(xtrue.dump());
			System.out.println();
			
		    System.out.println("x");
			System.out.println(x.dump());
			System.out.println();
			
			System.out.println("P");
			System.out.println(P.dump());
			System.out.println();
		}
	}
	
	public static Mat[] augment(Mat x, Mat P, Mat z, Mat R)
	{
		Mat[] retValue = new Mat[2];//x, P
		retValue[0] = x;
		retValue[1] = P;
		
		//add new features to state
		for(int i = 0; i < z.cols(); i++)
		{
			retValue = add_one_z(x, P, z.submat(0, z.rows(), i, i + 1), R);
		}
		
		return retValue;
	}
	
	public static Mat[] add_one_z(Mat x, Mat P, Mat z, Mat R)
	{
		Mat[] retValue = new Mat[2];//x, P
		
		int len = length(x);
		
		//TODO: double check r and b
		double r = z.get(0, 0)[0];
		double b = z.get(1, 0)[0];
		double s = Math.sin(x.get(2, 0)[0] + b);
		double c = Math.cos(x.get(2, 0)[0] + b);
		
		//augment x
		Mat x1 = Mat.zeros(2, 1, CvType.CV_64F);
		x1.put(0, 0, x.get(0, 0)[0] + r * c);
		x1.put(1, 0, x.get(1, 0)[0] + r * s);
		
		x.push_back(x1);
		
		//jacobians
		Mat Gv = Mat.zeros(2, 3, CvType.CV_64F);
		Gv.put(0, 0, 1);
		Gv.put(0, 1, 0);
		Gv.put(0, 2, -r * s);
		Gv.put(1, 0, 0);
		Gv.put(1, 1, 1);
		Gv.put(1, 2, r * c);
		
		Mat Gz = Mat.zeros(2, 2, CvType.CV_64F);
		Gz.put(0, 0, c);
		Gz.put(0, 1, -r * s);
		Gz.put(1, 0, s);
		Gz.put(1, 1, r * c);
		
		//augment P
		Mat expanded_P = Mat.zeros(len + 2, len + 2, CvType.CV_64F);
		Mat orig_P = expanded_P.submat(0, len, 0, len);
		P.copyTo(orig_P);
		P = expanded_P;
		
		Mat temp1 = P.submat(len, len + 2, len, len + 2);
		
		Mat t1 = Mat.zeros(Gv.rows(), 3, CvType.CV_64F);
		Core.gemm(Gv, P.submat(0, 3, 0, 3), 1, new Mat(), 0, t1);
		Mat t2 = Mat.zeros(t1.rows(), Gv.rows(), CvType.CV_64F);
		Core.gemm(t1, Gv.t(), 1, new Mat(), 0, t2);
		
		Mat t3 = Mat.zeros(Gz.rows(), R.cols(), CvType.CV_64F);
		Core.gemm(Gz, R, 1, new Mat(), 0, t3);
		Mat t4 = Mat.zeros(t3.rows(), Gz.rows(), CvType.CV_64F);
		Core.gemm(t3, Gz.t(), 1, new Mat(), 0, t4);
		
		Core.add(t2, t4, temp1);// feature cov
		
		Mat temp2 = P.submat(len, len + 2, 0, 3);
		Core.gemm(Gv, P.submat(0, 3, 0, 3), 1, new Mat(), 0, temp2);//vehicle to feature xcorr
		
		Mat temp3 = P.submat(0, 3, len, len + 2);
		P.submat(len, len + 2, 0, 3).t().copyTo(temp3);
		
		if(len > 3)
		{
			Mat temp4 = P.submat(len, len + 2, 3, len);
			Core.gemm(Gv, P.submat(0, 3, 3, len), 1, new Mat(), 0, temp4);//map to feature xcorr
			
			Mat temp5 = P.submat(3, len, len, len + 2);
			temp4.t().copyTo(temp5);
		}
		
		retValue[0] = x;
		retValue[1] = P;
		return retValue;
	}
	
	public static Mat[] update(Mat x, Mat P, Mat z, Mat R, Mat idf)
	{
		Mat[] retValue = new Mat[2];//x, P
		retValue = single_update(x, P, z, R, idf);
		
		return retValue;
	}
	
	public static Mat[] single_update(Mat x, Mat P, Mat z, Mat R, Mat idf)
	{
		Mat[] retValue = new Mat[2];
		
		int lenz = z.cols();
		
		for(int i = 0; i < lenz; i++)
		{
			Mat[] retValue1 = observe_model(x, (int)idf.get(0, i)[0]);
			Mat zp = retValue1[0];
			Mat H = retValue1[1];
			
			Mat v = Mat.zeros(2, 1, CvType.CV_64F);
			v.put(0, 0, z.get(0, i)[0] - zp.get(0, 0)[0]);
			v.put(1, 0, pi_to_pi(z.get(1, i)[0] - zp.get(1, 0)[0]));
			
			Mat[] retValue2 = KF_cholesky_update(x, P, v, R, H);
			x = retValue2[0];
			P = retValue2[1];
		}
		
		retValue[0] = x;
		retValue[1] = P;
		return retValue;
	}
	
	public static Mat[] KF_cholesky_update(Mat x, Mat P, Mat v, Mat R, Mat H)
	{
		Mat[] retValue = new Mat[2];//x, P
		Mat PHt = Mat.zeros(P.rows(), H.rows(), CvType.CV_64F);
		Core.gemm(P, H.t(), 1, new Mat(), 0, PHt);
		
		Mat S = Mat.zeros(H.rows(), PHt.cols(), CvType.CV_64F);
		Core.gemm(H, PHt, 1, R, 1, S);
		
		Core.add(S, S.t(), S);
		Core.multiply(S, new Scalar(0.5), S);//make symmetric
		
		double[][] S_double_array = new double[S.rows()][S.cols()];
		
		for(int i = 0; i < S.rows(); i++)
			for(int j = 0; j < S.cols(); j++)
				S_double_array[i][j] = S.get(i, j)[0];
		
		double[][] Schol_double_array = Cholesky.cholesky(S_double_array);
		
		Mat Schol = Mat.zeros(Schol_double_array.length, Schol_double_array[0].length, CvType.CV_64F);
		
		for(int i = 0; i < Schol.rows(); i++)
			for(int j = 0; j < Schol.cols(); j++)
				Schol.put(i, j, Schol_double_array[i][j]);
		
		//transpose to get upper triangular
		Schol = Schol.t();
		Mat SCholInv = Schol.inv(); //triangular matrix
		
		Mat W1 = Mat.zeros(PHt.rows(), SCholInv.cols(), CvType.CV_64F);
		Core.gemm(PHt, SCholInv, 1, new Mat(), 0, W1);
		
		Mat W = Mat.zeros(W1.rows(), SCholInv.rows(), CvType.CV_64F);
		Core.gemm(W1, SCholInv.t(), 1, new Mat(), 0, W);
		
		Core.gemm(W, v, 1, new Mat(), 0, W);
		Core.add(x, W, x);
		
		Mat t1 = Mat.zeros(W1.rows(), W1.rows(), CvType.CV_64F);
		Core.gemm(W1, W1.t(), 1, new Mat(), 0, t1);
		Core.subtract(P, t1, P);
		
		retValue[0] = x;
		retValue[1] = P;
		
		return retValue;
	}
	
	public static Mat[] observe_model(Mat x, int idf)
	{
		Mat[] retValue = new Mat[2];//z, H
		int Nxv = 3;
		int fpos = Nxv - 1 + (idf + 1) * 2 - 1;
		
		Mat H = Mat.zeros(2, length(x), CvType.CV_64F);
		
		//auxiliary values
		double dx = x.get(fpos, 0)[0] - x.get(0, 0)[0];
		double dy = x.get(fpos + 1, 0)[0] - x.get(1, 0)[0];
		double d2 = Math.pow(dx, 2) + Math.pow(dy, 2);
		double d = Math.sqrt(d2);
		double xd = dx/d;
		double yd = dy/d;
		double xd2 = dx/d2;
		double yd2 = dy/d2;
		
		//predict z
		Mat z = new Mat(2, 1, CvType.CV_64F);
		z.put(0, 0, d);
		z.put(1, 0, Math.atan2(dy, dx) - x.get(2, 0)[0]);
		
		//calculate H
		Mat temp1 = H.submat(0, H.rows(), 0, 3);
		temp1.put(0, 0, -xd);
		temp1.put(0, 1, -yd);
		temp1.put(0, 2, 0);
		temp1.put(1, 0, yd2);
		temp1.put(1, 1, -xd2);
		temp1.put(1, 2, -1);
		
		Mat temp2 = H.submat(0, H.rows(), fpos, fpos + 2);
		temp2.put(0, 0, xd);
		temp2.put(0, 1, yd);
		temp2.put(1, 0, -yd2);
		temp2.put(1, 1, xd2);
		
		retValue[0] = z;
		retValue[1] = H;
		
		return retValue;
	}
	
	public static int length(Mat m)
	{
		int ret = m.rows();
		
		if(m.cols() > ret)
			ret = m.cols();
		
		return ret;
	}
	
	//TODO: not done yet
	public static Mat[] data_associate(Mat x, Mat P, Mat z, Mat R, double gate1, double gate2)
	{
		Mat[] retValue = new Mat[3];
		
		Mat zf = new Mat(0, 0, CvType.CV_64F);
		Mat zn = new Mat(0, 0, CvType.CV_64F);
		Mat idf = new Mat(0, 0, CvType.CV_64F);
		
		retValue[0] = zf;
		retValue[1] = idf;
		retValue[2] = zn;
		
		int Nxv = 3; //number of vehicle pose states
		int Nf = (x.rows() - Nxv)/2; //number of features already in map
		
		// linear search for nearest-neighbour, no clever tricks (like a quick
		// bounding-box threshold to remove distant features; or, better yet,
		// a balanced k-d tree lookup). TODO: implement clever tricks.
		
		for(int i = 0; i < z.cols(); i++)
		{
			double jbest = -1;
			double nbest = Double.POSITIVE_INFINITY;
			double outer = Double.POSITIVE_INFINITY;
			
			//search for neighbours
			for(int j = 0; j < Nf; j++)
			{
				Mat[] retValue1 = compute_association(x, P, z.submat(0, z.rows(), i, i + 1), R, j);
				Mat nis = retValue1[0];
				Mat nd = retValue1[1];
				
				if(nis.get(0, 0)[0] < gate1 && nd.get(0, 0)[0] < nbest)//if within gate, store nearest-neighbour
				{
					nbest = nd.get(0, 0)[0];
					jbest = j;
				}
				else if(nis.get(0, 0)[0] < outer)//else store best nis value
				{
					outer = nis.get(0, 0)[0];
				}
			}
			
			//add nearest-neighbour to association list
			if(jbest != -1)
			{
				Mat temp1 = new Mat(z.rows(), zf.cols() + 1, CvType.CV_64F);
				//copy zf
				for(int a = 0; a < zf.rows(); a++)
					for(int b = 0; b < zf.cols(); b++)
						temp1.put(a, b, zf.get(a, b)[0]);
				
				//add all rows from z column i
				for(int a = 0; a < z.rows(); a++)
					temp1.put(a, temp1.cols() - 1, z.get(a, i)[0]);
				
				zf = temp1;
				retValue[0] = zf;
				
				Mat temp2 = new Mat(1, idf.cols() + 1, CvType.CV_64F);
				
				for(int a = 0; a < idf.rows(); a++)
					for(int b = 0; b < idf.cols(); b++)
						temp2.put(a, b, idf.get(a, b)[0]);
				
				temp2.put(0, temp2.cols() - 1, jbest);
				
				idf = temp2;
				retValue[1] = idf;
			}
			else if(outer > gate2)
			{
				Mat temp3 = new Mat(z.rows(), zn.cols() + 1, CvType.CV_64F);
				//copy zn
				for(int a = 0; a < zn.rows(); a++)
					for(int b = 0; b < zn.cols(); b++)
						temp3.put(a, b, zn.get(a, b)[0]);
				
				//add all rows from z column i
				for(int a = 0; a < z.rows(); a++)
					temp3.put(a, temp3.cols() - 1, z.get(a, i)[0]);
				
				zn = temp3;
				retValue[2] = zn;
			}
		}
				
		return retValue;
	}
	
	public static Mat[] compute_association(Mat x, Mat P, Mat z, Mat R, int idf)
	{
		Mat[] retValue = new Mat[2];
		//return normalised innovation squared (ie, Mahalanobis distance) and normalised distance
		Mat[] retValue1 = new Mat[2];
		retValue1 = observe_model(x, idf);
		Mat zp = retValue1[0];
		Mat H = retValue1[1];
		
		Mat v = new Mat(z.rows(), z.cols(), CvType.CV_64F);
		Core.subtract(z, zp, v);
		v.put(1, 0, pi_to_pi(v.get(1, 0)[0]));
		
		Mat S = new Mat(H.rows(), H.rows(), CvType.CV_64F);
		Core.gemm(H, P, 1, new Mat(), 0, S);
		Core.gemm(S, H.t(), 1, new Mat(), 0, S);
		Core.add(S, R, S);
		
		Mat nis = new Mat(v.cols(), v.cols(), CvType.CV_64F);
		Core.gemm(v.t(), S.inv(), 1, new Mat(), 0, nis);
		Core.gemm(nis, v, 1, new Mat(), 0, nis);
		
		Mat nd = new Mat(nis.rows(), nis.cols(), CvType.CV_64F);
		Core.add(nis, new Scalar(Math.log(Core.determinant(S))), nd);
		
		retValue[0] = nis;
		retValue[1] = nd;
		return retValue;
	}
	
	public static Mat add_observation_noise(Mat z, Mat R, boolean addnoise)
	{
		Mat retValue;
		
		if(addnoise)
		{
			int len = z.cols();
			if(len > 0)
			{
				Mat randn1 = Mat.zeros(1, z.cols(), CvType.CV_64F);
				Core.randn(randn1, 0, 1);
				Core.multiply(randn1, new Scalar(Math.sqrt(R.get(0, 0)[0])), randn1);
				
				Mat randn2 = Mat.zeros(1, z.cols(), CvType.CV_64F);
				Core.randn(randn2, 0, 1);
				Core.multiply(randn2, new Scalar(Math.sqrt(R.get(1, 1)[0])), randn2);
				
				Core.add(z.submat(0, 1, 0, z.cols()), randn1, z.submat(0, 1, 0, z.cols()));
				Core.add(z.submat(1, 2, 0, z.cols()), randn2, z.submat(1, 2, 0, z.cols()));
			}
		}
		
		retValue = z;
		return retValue;
	}
	
	public static Mat[] get_observations(Mat x, Mat lm, Mat idf, double rmax)
	{
		Mat[] retValue = new Mat[2];
		Mat z = new Mat(2, 1, CvType.CV_64F);
		retValue[0] = z;
		retValue[1] = idf;
		
		Mat lm_copy = new Mat();
		lm.copyTo(lm_copy);
		
		Object[] retValue1 = get_visible_landmarks(x, lm_copy, idf, rmax);
		lm_copy = (Mat)retValue1[0];
		idf = (Mat)retValue1[1];
		retValue1[0] = lm_copy;
		retValue[1] = idf;
		
		z = compute_range_bearing(x, lm_copy);
		retValue[0] = z;
		
		return retValue;
	}
	
	public static Mat compute_range_bearing(Mat x, Mat lm)
	{
		//Compute exact observation
		Mat dx = lm.submat(0, 1, 0, lm.cols());
		Core.subtract(lm.submat(0, 1, 0, lm.cols()), new Scalar(x.get(0, 0)[0]), dx);
		Mat dy = lm.submat(1, 2, 0, lm.cols());
		Core.subtract(lm.submat(1, 2, 0, lm.cols()), new Scalar(x.get(1, 0)[0]), dy);
		double phi = x.get(2, 0)[0];
		
		Mat z = Mat.zeros(2, lm.cols(), CvType.CV_64F);
		
		Mat temp1 = new Mat(dx.rows(), dx.cols(), CvType.CV_64F);
		Core.add(dx.mul(dx), dy.mul(dy), temp1);
		Core.sqrt(temp1, temp1);
		
		Mat temp2 = new Mat(dx.rows(), dx.cols(), CvType.CV_64F);
		for(int i = 0; i < temp2.rows(); i++)
		{
			for(int j = 0; j < temp2.cols(); j++)
			{
				temp2.put(i, j, Math.atan2(dy.get(i, j)[0], dx.get(i, j)[0]) - phi);
			}
		}
		
		temp1.copyTo(z.submat(0, 1, 0, lm.cols()));
		temp2.copyTo(z.submat(1, 2, 0, lm.cols()));
		return z;
	}
	
	public static Object[] get_visible_landmarks(Mat x, Mat lm, Mat idf, double rmax)
	{
		Mat[] retValue = new Mat[2];
		
		retValue[0] = lm;
		retValue[1] = idf;
		
		Mat lm_copy = new Mat();
		lm.copyTo(lm_copy);
		
		//Select set of landmarks that are visible within vehicle's semi-circular field-of-view
		Mat dx = lm_copy.submat(0, 1, 0, lm_copy.cols());
		Core.subtract(dx, new Scalar(x.get(0, 0)[0]), dx);
		
		Mat dy = lm_copy.submat(1, 2, 0, lm_copy.cols());
		Core.subtract(dy, new Scalar(x.get(1, 0)[0]), dy);
		
		double phi = x.get(2, 0)[0];
		
		ArrayList ii = new ArrayList();
		
		//incremental tests for bounding semi-circle
		for(int i = 0; i < dx.cols(); i++)
		{
			if(Math.abs(dx.get(0, i)[0]) < rmax && Math.abs(dy.get(0, i)[0]) < rmax //bounding box
					&& (dx.get(0, i)[0] * Math.cos(phi) + dy.get(0, i)[0] * Math.sin(phi)) > 0 //bounding line
					&& Math.pow(dx.get(0, i)[0], 2) + Math.pow(dy.get(0, i)[0], 2) < Math.pow(rmax, 2)) //bounding circle
			{
				ii.add(i);
			}
		}
		
		Mat lm1 = new Mat(lm.rows(), ii.size(), CvType.CV_64F);
		Mat idf1 = new Mat(idf.rows(), ii.size(), CvType.CV_64F);
		
		for(int i = 0; i < ii.size(); i++)
		{
			lm1.put(0, i, lm.get(0, (int)ii.get(i))[0]);
			lm1.put(1, i, lm.get(1, (int)ii.get(i))[0]);
			idf1.put(0, i, idf.get(0, (int)ii.get(i))[0]);
		}
		
		retValue[0] = lm1;
		retValue[1] = idf1;		
		return retValue;
	}
	
	public static Mat[] observe_heading(Mat x, Mat P, double phi, boolean useheading)
	{
		Mat[] retValue = new Mat[2];
		retValue[0] = x;
		retValue[1] = P;
		
		if(!useheading)
			return retValue;
		
		double sigmaPhi = 0.01 * Math.PI/180;
		Mat H = Mat.zeros(1, Math.max(x.rows(), x.cols()), CvType.CV_64F);
		H.put(0, 2, 1);
		double v = pi_to_pi(phi - x.get(2, 0)[0]);
		
		//TODO: KF_joseph_update is not found. Not used if useheading is false
		//Mat[] retValue1 = KF_joseph_update(x, P, v, sigmaPhi^2, H);
		return retValue;
	}
	
	public static Mat[] predict(Mat x, Mat P, double v, double g, Mat Q, double WB, double dt)
	{
		Mat[] retValue = new Mat[2];
		double s = Math.sin(g + x.get(2, 0)[0]);
		double c = Math.cos(g + x.get(2, 0)[0]);
		double vts = v * dt * s;
		double vtc = v * dt * c;
		
		//jacobians
		Mat Gv = new Mat(3, 3, CvType.CV_64F);
		Gv.put(0, 0, 1);
		Gv.put(0, 1, 0);
		Gv.put(0, 2, -vts);
		Gv.put(1, 0, 0);
		Gv.put(1, 1, 1);
		Gv.put(1, 2, vtc);
		Gv.put(2, 0, 0);
		Gv.put(2, 1, 0);
		Gv.put(2, 2, 1);
		
		Mat Gu = new Mat(3, 2, CvType.CV_64F);
		Gu.put(0, 0, dt * c);
		Gu.put(0, 1, -vts);
		Gu.put(1, 0, dt * s);
		Gu.put(1, 1, vtc);
		Gu.put(2, 0, dt * Math.sin(g)/WB);
		Gu.put(2, 1, v * dt * Math.cos(g)/WB);
		
		//predict covariance
		Mat p1 = P.submat(0, 3, 0, 3);
		Mat temp1 = new Mat(3, 3, CvType.CV_64F);
		Core.gemm(Gv, P.submat(0, 3, 0, 3), 1, new Mat(), 0, temp1);
		Core.gemm(temp1, Gv.t(), 1, new Mat(), 0, temp1);
		
		Mat temp2 = new Mat(3, 3, CvType.CV_64F);
		Core.gemm(Gu, Q, 1, new Mat(), 0, temp2);
		Core.gemm(temp2, Gu.t(), 1, new Mat(), 0, temp2);
		
		Core.add(temp1, temp2, p1);
	
		if(P.rows() > 3)
		{
			Mat p2 = P.submat(0, 3, 3, P.cols());
			Core.gemm(Gv, p2, 1, new Mat(), 0, p2);
			
			Mat p3 = P.submat(3, P.rows(), 0, 3);
			p2.t().copyTo(p3);
		}
		
		//predict state
		x.put(0, 0, x.get(0, 0)[0] + vtc);
		x.put(1, 0, x.get(1, 0)[0] + vts);
		x.put(2, 0, pi_to_pi(x.get(2, 0)[0] + v * dt * Math.sin(g)/WB));
		
		retValue[0] = x;
		retValue[1] = P;
		
		return retValue;
	}
	
	public static double[] add_control_noise(double V, double G, Mat Q, boolean addnoise)
	{
		double[] retValue = new double[2];
		retValue[0] = V;
		retValue[1] = G;
		
		if(addnoise)
		{
			Random r = new Random();
			retValue[0] = V = V + r.nextGaussian() * Math.sqrt(Q.get(0, 0)[0]);
			retValue[1] = G = G + r.nextGaussian() * Math.sqrt(Q.get(1, 1)[0]);
		}
		
		return retValue;
	}
	
	public static Mat vehicle_model(Mat xv, double V, double G, double WB, double dt)
	{
		xv.put(0, 0, xv.get(0, 0)[0] + V * dt * Math.cos(G + xv.get(2, 0)[0]));
		xv.put(1, 0, xv.get(1, 0)[0] + V * dt * Math.sin(G + xv.get(2, 0)[0]));
		xv.put(2, 0, pi_to_pi(xv.get(2, 0)[0] + V * dt * Math.sin(G) / WB));
		
		return xv;
	}
	
	public static Object[] compute_steering(Mat x, Mat wp, int iwp, double minD, double G, double rateG, double maxG, double dt)
	{
		Object[] retValue = new Object[2];
		retValue[0] = G;
		retValue[1] = iwp;
		
		Mat cwp = wp.submat(0, wp.rows(), iwp, iwp + 1);
		double d2 = (Math.pow(cwp.get(0, 0)[0]-x.get(0, 0)[0], 2) + Math.pow(cwp.get(1, 0)[0] - x.get(1, 0)[0], 2));
		
		if(d2 < Math.pow(minD, 2))
		{
			retValue[1] = ++iwp; //switch to next
			if(iwp > wp.cols() - 1)
			{
				retValue[1] = iwp = -1;
				return retValue;
			}
			cwp = wp.submat(0, cwp.rows(), iwp, iwp + 1);
		}
		
		// compute change in G to point towards current waypoint
		double deltaG = pi_to_pi(Math.atan2(cwp.get(1, 0)[0] - x.get(1, 0)[0], cwp.get(0, 0)[0] - x.get(0, 0)[0])  - x.get(2, 0)[0] - G);
		
		//limit rate
		double maxDelta= rateG*dt;
		if (Math.abs(deltaG) > maxDelta)
		{
		    deltaG= sign(deltaG)*maxDelta;
		}
		
		//limit angle
		G= G + deltaG;
		if (Math.abs(G) > maxG)
		    G= sign(G) * maxG;
		
		retValue[0] = G;
		
		return retValue;
	}
	
	public static double sign(double x)
	{
		double sign = 0;
		if(x < 0)
			sign = -1;
		else if(x > 0)
			sign = 1;
		else
			sign = 0;
		
		return sign;
	}
	
	public static double pi_to_pi(double angle)
	{
		double[] angles = {angle};
		angles = pi_to_pi(angles);
		
		return angles[0];
	}
	
	public static double[] pi_to_pi(double[] angle)
	{
		for(int i = 0; i < angle.length; i++)
			angle[i] = angle[i]%(2*Math.PI);
		
		for(int i = 0; i < angle.length; i++)
		{
			if(angle[i] > Math.PI)
				angle[i] = angle[i] - 2*Math.PI;
		}
		
		for(int i = 0; i < angle.length; i++)
		{
			if(angle[i] < -Math.PI)
				angle[i] = angle[i] + 2*Math.PI;
		}
		
		return angle;
	}
}