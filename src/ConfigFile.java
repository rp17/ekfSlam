import org.opencv.core.CvType;
import org.opencv.core.Mat;

public class ConfigFile 
{
	//control parameters
	public static double V = 3; //m/s
	public static double MAXG= 30*Math.PI/180;// radians, maximum steering angle (-MAXG < g < MAXG)
	public static double RATEG= 20*Math.PI/180;// rad/s, maximum rate of change in steer angle
	public static double WHEELBASE= 4;// metres, vehicle wheel-base
	public static double DT_CONTROLS= 0.025;// seconds, time interval between control signals
	
	//control noises
	public static double sigmaV= 0.3;// m/s
	public static double sigmaG= (3.0*Math.PI/180); //radians
	public static Mat Q()//[sigmaV^2 0; 0 sigmaG^2];
	{
		//initial Q = [SigmaV^2       0]
		//            [0       SigmaG^2]
		Mat q = new Mat(2, 2, CvType.CV_64F);
		q.put(0, 0, sigmaV*sigmaV);
		q.put(0, 1, 0);
		q.put(1, 0, 0);
		q.put(1, 1, sigmaG*sigmaG);
		return q;
	} 

	//observation parameters
	public static double MAX_RANGE= 30.0; // metres
	public static double DT_OBSERVE= 8*DT_CONTROLS; // seconds, time interval between observations

	//observation noises
	public static double sigmaR= 0.1; // metres
	public static double sigmaB= (1.0*Math.PI/180); // radians
	public static Mat R()//[sigmaR^2 0; 0 sigmaB^2];
	{
		//initial R = [SigmaR^2       0]
		//            [0       SigmaB^2]
		Mat q = new Mat(2, 2, CvType.CV_64F);
		q.put(0, 0, sigmaR*sigmaR);
		q.put(0, 1, 0);
		q.put(1, 0, 0);
		q.put(1, 1, sigmaB*sigmaB);
		return q;
	} 

	//data association innovation gates (Mahalanobis distances)
	public static double GATE_REJECT= 4.0; // maximum distance for association
	public static double GATE_AUGMENT= 25.0; // minimum distance for creation of new feature
	// For 2-D observation:
	//   - common gates are: 1-sigma (1.0), 2-sigma (4.0), 3-sigma (9.0), 4-sigma (16.0)
	//   - percent probability mass is: 1-sigma bounds 40%, 2-sigma 86%, 3-sigma 99%, 4-sigma 99.9%.

	//waypoint proximity
	public static double AT_WAYPOINT= 1.0; // metres, distance from current waypoint at which to switch to next waypoint
	public static double NUMBER_LOOPS= 1; // number of loops through the waypoint list

	//switches
	public static boolean SWITCH_CONTROL_NOISE= false; // if 0, velocity and gamma are perfect
	public static boolean SWITCH_SENSOR_NOISE = false; // if 0, measurements are perfect
	public static boolean SWITCH_INFLATE_NOISE= false; // if 1, the estimated Q and R are inflated (ie, add stabilising noise)
	public static boolean SWITCH_HEADING_KNOWN= false; // if 1, the vehicle heading is observed directly at each iteration
	public static boolean SWITCH_SEED_RANDOM= false; // if not 0, seed the randn() with its value at beginning of simulation (for repeatability)
}