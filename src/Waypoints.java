import java.awt.Color;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;

public class Waypoints {
	private List<Wpt> wpts = new ArrayList<Wpt>(1000);
	private Color color = Color.GREEN;
	private boolean drawPoint = true;
	
	public class Wpt {
		public Vector2D pos;
		public String name;
		public double x,y;
		public Wpt(Vector2D pos) {
			this.pos = pos;
			x = pos.x;
			y = pos.y;
			name = "WP" + wpts.size();
		}
	}
	
	public void setColor(Color color)
	{
		this.color = color;
	}
	
	public void setDrawPoint(boolean drawPoint)
	{
		this.drawPoint = drawPoint;
	}
	
	public void addWpt(Vector2D pos) {
		wpts.add(new Wpt(pos));
	}
	public void clearWpts(){
		wpts.clear();
	}
	public int size(){return wpts.size();}
	public Waypoints.Wpt get(int i) {return wpts.get(i);}
	public void render() {	
		
	}
}
