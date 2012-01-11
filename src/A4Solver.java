
import java.awt.Point;
import java.io.BufferedInputStream;
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Set;
import org.omg.CORBA.Environment;

/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
/**
 *
 * @author Vivek
 */
class Point2D {

    static Point2D minus(Point2D predPt, Point2D obpt) {

        Point2D p = new Point2D();

        p.x = predPt.x - obpt.x;
        p.y = predPt.y - obpt.y;

        return p;

    }

    static double magnitude(Point2D p) {
        return p.x * p.x + p.y * p.y;
    }
    public double x;
    public double y;

    public Point2D() {
        x = 0.0;
        y = 0.0;
    }

    public Point2D(double _x, double _y) {
        x = _x;
        y = _y;
    }

    Point2D(Point2D _p1) {
        x = _p1.x;
        y = _p1.y;
    }
}

class Point3D {

    public double x;
    public double y;
    public double z;

    public Point3D(double _x, double _y, double _z) {
        x = _x;
        y = _y;
        z = _z;
    }

    Point3D() {
        x = 0.0;
        y = 0.0;
        z = 0.0;
    }

    Point3D minus(Point3D p1) {
        Point3D p = new Point3D();

        p.x = x - p1.x;
        p.y = y - p1.y;
        p.z = z - p1.z;

        return p;
    }

    double DotProd(Point3D p1) {
        Point3D p = new Point3D();

        p.x = x * p1.x;
        p.y = y * p1.y;
        p.z = z * p1.z;

        return p.x + p.y + p.z;

    }

    Point3D multiply(double z) {
        Point3D p = new Point3D();

        p.x = this.x * z;
        p.y = this.y * z;
        p.z = this.z * z;

        return p;
    }
}

class Line {

    public Point2D p1;
    public Point2D p2;

    //public double slope;
    Line(Point2D _p1, Point2D _p2) {
        p1 = new Point2D(_p1);
        p2 = new Point2D(_p2);

        // slope = (p2.y - p1.y) / (p2.x - p1.x);
    }

    boolean isPointAboveLine(Point2D pt) {
        double y = pt.y;
        double x = pt.x;

        double x1 = p1.x;
        double x2 = p2.x;

        double y1 = p1.y;
        double y2 = p2.y;

        double l = (y - y1) * (x2 - x1);
        double r = (y2 - y1) * (x - x1);

        if ((Double.compare((l - r), 0.00)) < 0) {
            return true;
        } else {
            return false;
        }
    }
}

public class A4Solver {

    final Integer NUM_OF_TOKENS_IN_HYPO = 9;
    final Integer NUM_OF_BOUNDARY_POINTS = 4;
    final Integer NUM_OF_FRAMES = 301;
    String dataPath;
    Double sd;
    Map<Integer, ArrayList<Point2D>> hypoMap;
    ArrayList<Motion> motionHolder;
    ArrayList<Map<Integer, Point2D>> featurePointsHolder;
    Integer numberOfHypotheses;
    final double u_off = 672.605430;
    final double v_off = 387.235803;
    final double f_u = 1389.182714;
    final double f_v = 1394.598277;

    //Map<PointLoc, Line> boundaryLines;
    private PointLoc CheckWhichPlane(Point2D _point2d, Map<PointLoc, Line> boundaryLines) {

        Iterator it = (Iterator) boundaryLines.entrySet().iterator();

        PointLoc p = PointLoc.NONE;
        Line l = null;

//        while (it.hasNext()) {
//            Map.Entry pair = (Map.Entry) it.next();
//
//            l = (Line) pair.getValue();
//
//            if (checkPointWithinLine(_point2d, l, (PointLoc) pair.getKey())) {
//                p = (PointLoc) pair.getKey();
//                break;
//            }
//        }

        Line l1 = boundaryLines.get(PointLoc.LWALL);

        if (_point2d.x <= l1.p2.x) {
            p = PointLoc.LWALL;
            l = l1;
        }

        l1 = boundaryLines.get(PointLoc.RWALL);

        if (_point2d.x >= l1.p1.x) {
            p = PointLoc.RWALL;
            l = l1;
        }

        l1 = boundaryLines.get(PointLoc.MWALL);

        if (_point2d.x > l1.p1.x && _point2d.x < l1.p2.x) {
            p = PointLoc.MWALL;
            l = l1;
        }


        assert p != PointLoc.NONE || l != null : "Something wrong";

        //check ground or that plane
        if (!l.isPointAboveLine(_point2d)) {
            p = PointLoc.GROUND;
        }

        return p;

    }

    private Point3D Calc3DPointGround(Point3D _point3d) {
        //throw new UnsupportedOperationException("Not yet implemented");
        Point3D pg = new Point3D();

        pg.x = _point3d.x / _point3d.y;

        pg.y = 1;

        pg.z = 1 / _point3d.y;

        return pg;
    }

    private Point3D Calc3DPointInWall(Point3D _point3d, Line l) {


        Point3D p1 = Calc3DPointGround(Normalize2DPoint(l.p1));
        Point3D p2 = Calc3DPointGround(Normalize2DPoint(l.p2));

        Point3D v_b = p2.minus(p1);

        Point3D n_w = new Point3D(v_b.z, 0, -v_b.x);

        double d_w = n_w.DotProd(p1);

        double t = n_w.DotProd(_point3d);

        double z = d_w / t;

        Point3D p_wall = _point3d.multiply(z);

        return p_wall;

    }

    private Point3D Normalize2DPoint(Point2D _point2d) {


        Point3D p_3d = new Point3D();

        p_3d.x = (_point2d.x - u_off) / f_u;
        p_3d.y = (_point2d.y - v_off) / f_v;
        p_3d.z = 1;

        return p_3d;
    }

    private Map<Integer, Point2D> RotateAndTranslate(Map<Integer, Point3D> frame03DPoints, int frame_id) {

        Motion m = motionHolder.get(frame_id);

        Matrix r_t = new Matrix(3, 3);

        BuildRotationMatrix(r_t, m.theta);

        Matrix trans = new Matrix(3, 1, 0);

        trans.element[0][0] = m.x;
        trans.element[2][0] = m.z;

        Iterator it = (Iterator) frame03DPoints.entrySet().iterator();

        Map<Integer, Point2D> rtPoints = new LinkedHashMap<Integer, Point2D>();

        while (it.hasNext()) {
            Map.Entry pair = (Map.Entry) it.next();

            Integer feature_index = (Integer) pair.getKey();
            Point3D P = (Point3D) pair.getValue();

            Matrix P_matrix = ConvertPointToMatrix(P);

            Matrix Q_t = Matrix.multiply(r_t, P_matrix);

            Q_t = Matrix.add(Q_t, trans);

            Matrix K = BuildNormMatrix();

            Matrix q_t = Matrix.multiply(K, Q_t);

            Matrix qt_image = Matrix.multiply(1 / q_t.element[2][0], q_t);


            assert qt_image.element[2][0] > 0.99998 : " z not norm, " + qt_image.element[2][0];

            Point2D pt = new Point2D(qt_image.element[0][0], qt_image.element[1][0]);

            rtPoints.put(feature_index, pt);
        }

        return rtPoints;

    }

    private void CalculateLikeHood_1(ArrayList<Double> hypoProb, Map<Integer, Point2D> predictedPoints, int frame_id, int h_id) {



        Iterator it = (Iterator) predictedPoints.entrySet().iterator();
        // ArrayList<Double> featureMax = new ArrayList<Double> ();

        double temp = 0.0;

        while (it.hasNext()) {
            Map.Entry pair = (Map.Entry) it.next();

            Integer feature_idx = (Integer) pair.getKey();
            Point2D predPt = (Point2D) pair.getValue();

            if (featurePointsHolder.get(frame_id).containsKey(feature_idx)) {
                Point2D obpt = featurePointsHolder.get(frame_id).get(feature_idx);

                Point2D tpt = Point2D.minus(predPt, obpt);

                double numerator = -Point2D.magnitude(tpt);
                double denom = 2 * sd * sd;

                temp += (numerator / denom);

                //featureMax.add(temp);

                //temp = temp * Math.exp(numerator / denom);
            }
        }

        // Collections.sort(featureMax);
        hypoProb.add(temp);

        // likelihood.hypoProb.put(h_id, temp);
    }

    private Map<PointLoc, Line> GetBoundaryLines(int h_id) {

        assert hypoMap.containsKey(h_id) : "Boundary Lines h_id not found";

        ArrayList<Point2D> points = hypoMap.get(h_id);

        Map<PointLoc, Line> bline = new LinkedHashMap<PointLoc, Line>();
        for (int i = 0; i < points.size() - 1; i++) {
            switch (i) {
                case 0:
                    bline.put(PointLoc.LWALL, new Line(points.get(i), points.get(i + 1)));
                    continue;
                case 1:
                    bline.put(PointLoc.MWALL, new Line(points.get(i), points.get(i + 1)));
                    continue;
                case 2:
                    bline.put(PointLoc.RWALL, new Line(points.get(i), points.get(i + 1)));
                    continue;
            }
        }

        return bline;
    }
//
//    private boolean checkPointWithinLine(Point2D _point2d, Line l, PointLoc p) {
//
//        if (p == PointLoc.LWALL) {
//            if( _point2d.x <= l.p2.x)
//                return true;
//        }
//        else if( p == PointLoc.RWALL)
//        {
//            if ( _point2d.x)
//        }
//
//        return false;
//    }

    private void BuildRotationMatrix(Matrix r_t, double theta) {

        r_t.element[0][0] = Math.cos(theta);
        r_t.element[0][1] = 0;
        r_t.element[0][2] = Math.sin(theta);
        r_t.element[1][0] = 0;
        r_t.element[1][1] = 1;
        r_t.element[1][2] = 0;
        r_t.element[2][0] = -Math.sin(theta);
        r_t.element[2][1] = 0;
        r_t.element[2][2] = Math.cos(theta);

    }

    private Matrix ConvertPointToMatrix(Point3D P) {
        Matrix m = new Matrix(3, 1);

        m.element[0][0] = P.x;
        m.element[1][0] = P.y;
        m.element[2][0] = P.z;

        return m;

    }

    private Matrix BuildNormMatrix() {

        Matrix r_t = new Matrix(3, 3);

        r_t.element[0][0] = f_u;
        r_t.element[0][1] = 0;
        r_t.element[0][2] = u_off;
        r_t.element[1][0] = 0;
        r_t.element[1][1] = f_v;
        r_t.element[1][2] = v_off;
        r_t.element[2][0] = 0;
        r_t.element[2][1] = 0;
        r_t.element[2][2] = 1;

        return r_t;
    }

    private ArrayList<Double> Smoothing(ArrayList<Double> hypoPost) {

        ArrayList<Double> d = new ArrayList();

        for (int i = 0; i < hypoPost.size(); i++) {
            if (hypoPost.get(i) < 0.00001) {
                d.add(0.00001);
            } else {
                d.add(hypoPost.get(i));
            }

        }

        return d;
    }

    private ArrayList<Double> Normalize(ArrayList<Double> hypoPost) {

        double sum = 0.0;

        for (double d : hypoPost) {
            sum += d;
        }

        ArrayList<Double> d = new ArrayList();

        if (Double.compare(sum, 0.0) > 0) {
            for (int i = 0; i < hypoPost.size(); i++) {
                d.add(hypoPost.get(i) / sum);
            }
        } else {
            d.addAll(hypoPost);
        }

        return d;
    }

    private void CalculateLikelihood_2(Likelihood likelihood, ArrayList<Double> hypoProb) {

        // Collections.sort(hypoProb);

        Double max = Double.NEGATIVE_INFINITY;

        for (Double h : hypoProb) {
            if (h > max) {
                max = h;
            }
        }

        ArrayList<Double> newProb = new ArrayList();

        for (Double h : hypoProb) {
            newProb.add(h - max);
        }

        int h_id = 0;
        for (Double d : newProb) {
            likelihood.hypoProb.put(h_id++, Math.exp(d));
        }
    }

    enum PointLoc {

        GROUND, LWALL, MWALL, RWALL, NONE
    };

    public A4Solver(String datapath, Double d) {

        this.dataPath = datapath;
        this.sd = d;

        hypoMap = new LinkedHashMap<Integer, ArrayList<Point2D>>();
        motionHolder = new ArrayList<Motion>();
        featurePointsHolder = new ArrayList<Map<Integer, Point2D>>(NUM_OF_FRAMES);

        //  boundaryLines = new LinkedHashMap<PointLoc, Line>();

    }

    public void solve() throws FileNotFoundException, IOException, java.lang.NumberFormatException {
        //Populate the data structures

        ReadAndPopulateFromFiles();

        Map<Integer, ArrayList<Double>> posterior = CalculatePosterior();

        double max = Double.NEGATIVE_INFINITY;
        int maxHypo = -1;



        String hypofile = null;
        String hypofile1 = "_" + sd.longValue() + "_Posterior.txt";

        String dirPath = A4Main.dirPath;
        
        if (dirPath.contains("\\")) {
            hypofile = dirPath.substring(dirPath.lastIndexOf("\\")+1);
        }
        else
        {
            hypofile = dirPath;
        }
        
        hypofile += hypofile1;
        
        // Create file 
        FileWriter fstream = new FileWriter(hypofile);
        BufferedWriter out = new BufferedWriter(fstream);




        int j = 1;

        while (j <= posterior.size()) {
            String toWrite = "";
            toWrite += j;
            //print to file
            for (int i = 0; i < numberOfHypotheses; i++) {
                int h = i;

                double data = posterior.get(j).get(i);

                if (j == 300) {
                    if (Double.compare(data, max) > 0) {
                        max = data;
                        maxHypo = i;
                    }
                }

                toWrite += "\t" + data;


            }
            j++;

            out.write(toWrite);
            out.write(System.getProperty("line.separator"));;
        }
        out.close();

        System.out.println(" Max Hypo is " + maxHypo + " ,Probabilty is " + max);
    }

    private void ReadAndPopulateFromFiles() throws FileNotFoundException, IOException, java.lang.NumberFormatException {


        //Read hypothesis.txt

        String hypoFilePath = dataPath + "\\hypotheses.txt";

        File hypoFile = new File(hypoFilePath);

        FileReader hypoStream = new FileReader(hypoFile);

        BufferedReader hypoReader = new BufferedReader(hypoStream);

        numberOfHypotheses = Integer.parseInt(hypoReader.readLine().trim());

        int numberOfLines = 0;

        while (numberOfLines++ < numberOfHypotheses) {
            String line = hypoReader.readLine().trim();

            String tokens[] = line.split("\\s");

            assert tokens.length == NUM_OF_TOKENS_IN_HYPO : "Failure: Invalid number of tokens in hypo file";

            Integer h_id = Integer.parseInt(tokens[0]);

            ArrayList<Point2D> point_vec = new ArrayList<Point2D>(NUM_OF_BOUNDARY_POINTS);

            for (int i = 1; i < tokens.length; i += 2) {
                Double x = Double.parseDouble(tokens[i]);
                Double y = Double.parseDouble(tokens[i + 1]);

                point_vec.add(new Point2D(x, y));
            }

            assert point_vec.size() == NUM_OF_BOUNDARY_POINTS : "Failure: Invalid num of boundary";

            //Add to hypo map
            hypoMap.put(h_id, point_vec);

        }

        assert hypoMap.size() == numberOfHypotheses : "Failure: Invalid no. of hypothese in txt file";

        //read motion.txt
        String motionFilePath = dataPath + "\\motion.txt";

        File motionFile = new File(motionFilePath);

        FileReader motionStream = new FileReader(motionFile);

        BufferedReader motionReader = new BufferedReader(motionStream);

        String line = null;

        while ((line = motionReader.readLine()) != null) {
            String tokens[] = line.split("\\s");

            assert tokens.length == 4 : "Invalid no of tokens in motion file";

            Integer f_id = Integer.parseInt(tokens[0]);

            Motion m = new Motion();

            m.theta = Double.parseDouble(tokens[1]);
            m.x = Double.parseDouble(tokens[2]);
            m.z = Double.parseDouble(tokens[3]);

            motionHolder.add(m);
        }

        assert motionHolder.size() == NUM_OF_FRAMES : "Invalid no. of frames in motion file";

        //read features.txt
        String featuresFilePath = dataPath + "\\features.txt";

        File featuresFile = new File(featuresFilePath);

        FileReader featuresStream = new FileReader(featuresFile);

        BufferedReader featuresReader = new BufferedReader(featuresStream);

        while ((line = featuresReader.readLine()) != null) {

            Map m = new LinkedHashMap<Integer, Point2D>();;
            String tokens[] = line.split("\\s");

            assert tokens.length == 2 : "Failure: Invalid no in features";

            Integer numOfLinesToRead = Integer.parseInt(tokens[1]);

            for (int i = 0; i < numOfLinesToRead; i++) {
                line = featuresReader.readLine();

                assert line != null : "Invalid line in features";

                tokens = line.split("\\s");

                assert tokens.length == 3 : "Failure: Invalid no. of token in features";

                Integer feature_id = Integer.parseInt(tokens[0]);

                Point2D p = new Point2D(Double.parseDouble(tokens[1]), Double.parseDouble(tokens[2]));

                m.put(feature_id, p);

            }

            featurePointsHolder.add(m);

        }

        assert featurePointsHolder.size() == NUM_OF_FRAMES : "Invalid no. of frames in features";

    }

    //0/(h1,h2...hm)
    class Likelihood {

        public Map<Integer, Double> hypoProb;

        public Likelihood() {
            hypoProb = new LinkedHashMap<Integer, Double>();
        }
    }

    private Map<Integer, ArrayList<Double>> CalculatePosterior() {

        //posterior holder -> key is hypo id and vector of prob for all frames

        Map<Integer, ArrayList<Double>> posteriorHolder = new LinkedHashMap<Integer, ArrayList<Double>>();


        Likelihood likelihood[] = new Likelihood[NUM_OF_FRAMES];
        likelihood[0] = null;

        double prior = 1.0 / numberOfHypotheses;

        for (int i = 1; i < NUM_OF_FRAMES; i++) {

            likelihood[i] = new Likelihood();
            ArrayList<Double> hypoProb = new ArrayList<Double>();

            for (int j = 0; j < numberOfHypotheses; j++) {

                Map<Integer, Point3D> frame03DPoints = Calculate3DPoints(featurePointsHolder.get(0), j);


                Map<Integer, Point2D> predictedPoints = RotateAndTranslate(frame03DPoints, i);

                //likelihood -> Map with frame_id as key and arraylist of prob with each element being o_1/h_i
                CalculateLikeHood_1(hypoProb, predictedPoints, i, j);

                //likelihood.put(i, prob);
            }

            CalculateLikelihood_2(likelihood[i], hypoProb);
        }

        ArrayList<Double> temp = new ArrayList<Double>(numberOfHypotheses);
        //Collections.fill(temp, prior);

        for (int i = 0; i < numberOfHypotheses; i++) {
            temp.add(prior);
        }

        // ArrayList<Double> normalizedPost = new ArrayList<Double>();

        for (int m = 1; m < NUM_OF_FRAMES; m++) {

            ArrayList<Double> hypoPost = new ArrayList<Double>();

            for (int i = 0; i < numberOfHypotheses; i++) {

                assert likelihood[m].hypoProb.containsKey(i) : "SOmething worng!! ";

                double d = temp.get(i) * likelihood[m].hypoProb.get(i);

                hypoPost.add(d);
            }

            //double sum = Sum(hypoPost);

            ArrayList<Double> t2 = hypoPost;//Normalize(hypoPost);

            ArrayList<Double> t1 = Smoothing(t2);

            ArrayList<Double> t3 = Normalize(t1);

            temp = t3;

            posteriorHolder.put(m, t3);
        }

        return posteriorHolder;
    }

    private Map<Integer, Point3D> Calculate3DPoints(Map<Integer, Point2D> frame2DPoints, int h_id) {

        Map<Integer, Point3D> frame3DPoints = new LinkedHashMap<Integer, Point3D>();

        Iterator it = frame2DPoints.entrySet().iterator();

        while (it.hasNext()) {
            Map.Entry pair = (Map.Entry) it.next();

            Point3D p = ConvertPointTo3D((Point2D) (pair.getValue()), h_id);

            frame3DPoints.put((Integer) pair.getKey(), p);
        }

        return frame3DPoints;
    }

    private Point3D ConvertPointTo3D(Point2D _point2d, int h_id) {


        //calc boundary lines
        Map<PointLoc, Line> boundaryLines = GetBoundaryLines(h_id);

        Point3D p = new Point3D();

        Point3D normpt = Normalize2DPoint(_point2d);

        PointLoc loc = CheckWhichPlane(_point2d, boundaryLines);


        if (loc == PointLoc.GROUND) {
            p = Calc3DPointGround(normpt);
        } else {
            Line l = boundaryLines.get(loc);

            p = Calc3DPointInWall(normpt, l);
        }

        return p;
    }
}

class Motion {

    public double x;
    public double z;
    public double theta;
}
