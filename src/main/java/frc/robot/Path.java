package frc.robot;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Scanner; 
import java.util.logging.*;

public class Path{
    private ArrayList rightString = new ArrayList<>();
    private ArrayList leftString = new ArrayList<>();
    private static final Logger LOGGER = Logger.getLogger(Robot.class.getName());
    public Path(){}    

    public ArrayList returnLeftList() throws IOException{
            File file = new File("/paths/Unnamed.left.pf1.csv");
            Scanner sc = new Scanner(file);
            String[] testArray;
        
        sc.next();
        while(sc.hasNext()){
            testArray = sc.next().split(",");
            leftString.add(testArray[4]);
        }
        
        return leftString;
    }

    public ArrayList returnRightList() throws IOException{
        LOGGER.warning("break");
        File file = new File("/paths/Unnamed.right.pf1.csv");
        Scanner sc = new Scanner(file);
        String[] testArray;
        sc.next();
        while(sc.hasNext()){
            testArray = sc.next().split(",");
            rightString.add(testArray[4]);
        }
        return rightString;
    }
}