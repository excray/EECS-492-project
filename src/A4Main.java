
import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;

/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
/**
 *
 * @author Vivek
 */
public class A4Main {
    
    public static String dirPath;

    //private double sd;
    public static void main(String args[]) {
        try {
            assert args.length == 2 : "Failure: Invalid number of arguments";

            dirPath = args[0].trim();

            File videoDir = new File(dirPath);

            assert videoDir.exists() : "Failure: Video Directory Pathname is not correct.";

            String datapath = dirPath + "\\data";

            Double d = Double.parseDouble(args[1].trim());

            A4Solver a4Solver = new A4Solver(datapath, d);

            a4Solver.solve();

        } catch (java.lang.NumberFormatException n) {
            System.out.println("Failure: Enter double as SD");
        } catch (java.io.FileNotFoundException f) {
            System.out.println("Failure: File not found, " + f.toString());
        } catch ( java.io.IOException e){
            System.out.println("Failure: IO Error, "+e.toString());
        }


    }
}
