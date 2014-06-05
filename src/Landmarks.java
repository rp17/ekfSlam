
import java.awt.Point;
import java.util.ArrayList;
import java.util.List;



	 public class Landmarks {
			private List<Landmark> landmarks = new ArrayList<Landmark>(10);
			
			public void addLandmark(Vector2D pos) {
				landmarks.add(new Landmark(pos));
			}
			public void clearLandmarks(){
				landmarks.clear();
			}
			public boolean isEmpty(){return landmarks.isEmpty();}
			public int size(){return landmarks.size();}
			public Landmark get(int i) {return landmarks.get(i);}
			public Landmark remove(int i) {return landmarks.remove(i);}
			public boolean add(Landmark landmark) {return landmarks.add(landmark);}
			
			public void render() {		
				
			}	

		public class Landmark{
			public Point mapLocation;
			public Point updatedLocation = null;
			public double length;
			public boolean observed;
			public Vector2D pos;
			public double x,y;
			
			public Landmark(Point loc,double len){
				this.mapLocation = loc;
				this.length = len;
				this.observed = false;
			}
			
			public Landmark(Point loc,double len, Vector2D vector2D){
				this.mapLocation = loc;
				this.length = len;
				this.observed = false;
				this.pos = vector2D;
				this.x = vector2D.x;
				this.y = vector2D.y;
			}
			
			public Landmark(Vector2D vector2D){
				this.observed = false;
				this.pos = vector2D;
				this.x = vector2D.x;
				this.y = vector2D.y;
				//this.mapLocation = new Point(this.x, this.y);
			}
			
			
			public void updateLocation(Point updated){
				this.updatedLocation = updated;
			}
			
			public Point getNewLocation(){
				return updatedLocation;
			}
			
			public void isObserved(){
				this.observed = true;
			}
		}
	 }
		