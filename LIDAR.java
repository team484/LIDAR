package frc.robot;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Class that communicates with a SWEEP LIDAR sensor plugged into
 * one of the USB ports on the roboRIO. Initialize this class and
 * call startLIDAR(). For best results, set the minDist to 6 and
 * the rotation rate to 10hz. Then call findNearbyObjects() to get
 * a list of objects near the robot.
 */
public class LIDAR {
    private boolean postToDashboard = false;
    private int rotationRate = 5;
    private int maxpts = 1200 / rotationRate;
    private SerialPort.Port port = SerialPort.Port.kUSB;
    private SerialPort sweep;
    private boolean run = true;
    private double[][] points_array = new double[maxpts][2]; // points to track [distance,angle]
    private int pos = 0; // position in points
    private int points = 0; // size of points array
    private long timeout = 10000;
    private long lastPointcloudUpdate = 0;
    private int minDistance = 3;

    public LIDAR() {
        sweep = new SerialPort(115200, this.port, 8);
    }

    public LIDAR(SerialPort.Port port) {
        this.port = port;
        sweep = new SerialPort(115200, this.port, 8);
    }

    /**
     * This thread is used to read data from the lidar.
     * The while loop continually collects lidar pointcloud
     * data. setting the run boolean in the lidar class to false
     * will end this thread.
     */
    private Thread thread = new Thread(() -> {
        int exceptionCounter = 0;
        while (this.run) { // Thread loop
            try {
                byte[] status_b = this.sweep.read(1);
                if (status_b.length == 0) {
                    throw new ArrayIndexOutOfBoundsException();
                }
                byte[] angle_b = this.sweep.read(2);
                byte[] dist_b = this.sweep.read(2);
                byte[] strength_b = this.sweep.read(1);
                byte[] checksum_b = this.sweep.read(1);
                while (true) { // Wait for good packet
                    int checksum = (status_b[0] & 0xFF) + (angle_b[0] & 0xFF) + (angle_b[1] & 0xFF) + (dist_b[0] & 0xFF)
                            + (dist_b[1] & 0xFF) + (strength_b[0] & 0xFF);
                    checksum = checksum % 255;
                    if (checksum == (checksum_b[0] & 0xFF)) { // This looks like a good packet
                        break;
                    }
                    // Bad packet, shift every byte over
                    status_b[0] = angle_b[0];
                    angle_b[0] = angle_b[1];
                    angle_b[1] = dist_b[0];
                    dist_b[0] = dist_b[1];
                    dist_b[1] = strength_b[0];
                    strength_b[0] = checksum_b[0];
                    checksum_b = this.sweep.read(1);
                }
                if (status_b[0] == 1) { // New Rotation
                    this.points = this.pos;
                    this.pos = 0;
                    if (postToDashboard) {
                        double[] angles = new double[points];
                        double[] distances = new double[points];
                        for (int i = 0; i < points; i++) {
                            distances[i] = this.points_array[i][0];
                            angles[i] = this.points_array[i][1];
                        }
                        SmartDashboard.putNumberArray("Angles", angles);
                        SmartDashboard.putNumberArray("Distances", distances);
                        SmartDashboard.putNumber("Points", this.points);
                    }
                    this.lastPointcloudUpdate = System.currentTimeMillis();
                }
                int angle_i = (((angle_b[1] & 0xFF) << 8) | (angle_b[0] & 0xFF));
                double angle = (double) (angle_i) / 16.0;
                int dist = ((dist_b[1] & 0xFF) << 8) | (dist_b[0] & 0xFF);
                if (dist >= minDistance) {
                    this.points_array[this.pos][1] = angle;
                    this.points_array[this.pos][0] = dist;
                    if (this.pos < maxpts - 1)
                        this.pos++;
                }
            } catch (Exception e) {
                e.printStackTrace();
                exceptionCounter++;
                if (exceptionCounter > 5) {
                    break;
                }
            }
        }
        points = 0;
        this.sweep.close();
    });

    // -----Lidar Functions-----
    /**
     * Tells the lidar to stop sending pointcloud data over the
     * serial bus. Should be done before modifying rotation rate or speed.
     * @return true if the command succeeded
     */
    private boolean disableDataCollection() {
        String responseString = "";
        sweep.readString();
        long startTime = System.currentTimeMillis();
        while (!responseString.startsWith("DX00") && System.currentTimeMillis() < startTime + timeout) {
            sweep.write("DX\n".getBytes(), 3);
            sweep.flush();
            responseString = new String(sweep.read(6));
            try {
                Thread.sleep(20);
            } catch (Exception e) {
            }
        }
        return responseString.startsWith("DX00");
    }

    /**
     * Sets the rotation rate of the lidar in hz.
     * @param speed - rate in hz (1-10)
     * @return true if the command succeeded
     */
    private boolean setLidarSpeed(int speed) {
        String speedString = "MS";
        String responseString = "";
        if (rotationRate >= 10) {
            speedString = speedString + "10\n";
        } else {
            speedString = speedString + "0" + rotationRate + "\n";
        }
        long startTime = System.currentTimeMillis();
        sweep.readString();
        while (!responseString.startsWith(speedString + "00") && System.currentTimeMillis() < startTime + timeout) {
            sweep.write(speedString.getBytes(), 5); // MS[1-10] which is 1-10 rotations per second
            sweep.flush();
            responseString = new String(sweep.read(9));
            try {
                Thread.sleep(20);
            } catch (Exception e) {
            }
        }
        return responseString.startsWith(speedString + "00");
    }

    /**
     * Sets the sample rate of the lidar (500-1000 hz)
     * @param rate data rate (use 1 for 500hz, 2 for 750hz, 3 for 1000hz)
     * @return true if the command succeeded
     */
    private boolean setDataRate(int rate) {
        // 1 - 500 samples/second, 2 - 750, 3 - 1000
        String dataRateString = "LR0";
        String responseString = "";
        if (rate >= 3) {
            dataRateString = dataRateString + "3\n";
        } else if (rate <= 1) {
            dataRateString = dataRateString + "1\n";
        } else {
            dataRateString = dataRateString + "2\n";
        }
        long startTime = System.currentTimeMillis();
        sweep.readString();
        while (!responseString.startsWith(dataRateString + "00") && System.currentTimeMillis() < startTime + timeout) {
            sweep.write(dataRateString.getBytes(), 5); // LR0[1-3] 500, 750, or 1000 samples per second
            sweep.flush();
            responseString = new String(sweep.read(9));
            try {
                Thread.sleep(20);
            } catch (Exception e) {
            }
        }
        return responseString.startsWith(dataRateString + "00");
    }

    /**
     * Begins lidar data collection. Once called, the lidar will send
     * pointcloud data over the serial bus.
     * @return true if command succeeded
     */
    private boolean startDataCollection() {
        String responseString = "";
        sweep.readString();
        long startTime = System.currentTimeMillis();
        while (!responseString.startsWith("DS00") && System.currentTimeMillis() < startTime + timeout) {
            sweep.write("DS\n".getBytes(), 3);
            sweep.flush();
            responseString = new String(sweep.read(6));
            try {
                Thread.sleep(20);
            } catch (Exception e) {
            }
        }
        return responseString.startsWith("DS00");
    }

    // -----User functions-----

    /**
     * Initiates the lidar scanning process.
     * @param rotationRate the rate (from 1-10) the lidar spins at (in hz) Default is 5
     */
    public void startLIDAR(int rotationRate) {
        if (rotationRate > 10) {
            rotationRate = 10;
            System.err.println("Lidar rotation rate set too high. Max value is 10");
        } else if (rotationRate < 1) {
            rotationRate = 1;
            System.err.println("Lidar rotation rate set too low. Min value is 1");
        }
        this.rotationRate = rotationRate;
        this.maxpts = 1200 / this.rotationRate;
        this.points_array = new double[maxpts][2];
        startLIDAR();
    }

    /**
     * Initiates the lidar scanning process.
     */
    public void startLIDAR() {
        boolean success = false;
        do {
            if (!disableDataCollection())
                break;
            if (!setLidarSpeed(rotationRate))
                break;
            if (!setDataRate(3))
                break;
            if (!startDataCollection())
                break;
            success = true;
        } while (false);

        if (success) {
            thread.start();
            return;
        }

        System.out.println("UNABLE TO START LIDAR!!! (Try restarting robot)");
        sweep.close();
    }

    /**
     * Closes the lidar object. Once closed, startLidar() cannot be called
     * again.
     */
    public void close() {
        this.run = false;
    }

    /**
     * When enabled, the point cloud will be posted to the smart dashbard as
     * number arrays. One array is named "Distances" the other is named 
     * "Angles". The size of the readable portion of the arrays is also sent
     * and is called "Points"
     * @param enabled set to true to enable dashboard output. False to disable.
     */
    public void enableDashboardOutput(boolean enabled) {
        postToDashboard = enabled;
    }

    /**
     * Sets the timeout period for commands being sent to the lidar. Default is
     * 10000ms.
     * @param timeout timeout period in ms
     */
    public void setTimeout(long timeout) {
        this.timeout = timeout;
    }

    /**
     * Returns the point cloud captured by the lidar in polar coordinates.
     * Each element in the array is structured as [distance, angle]
     * Use getSizeOfPoints() when reading this array. The array includes buffer
     * space and should not be read in its entirety.
     * @return a pointcloud as a 2D double array
     */
    public double[][] getPoints() {
        return this.points_array;
    }

    /**
     * Calculates the cartesian coordinates of each point in the pointcloud.
     * This returns a new pointcloud array using this cartesian coordinate system.
     * This array is already at the appropriate size so there is no need to use
     * getSizeOfPoints(). Each element in the array is structured as [x,y,distance]
     * Since this needs to be calculated, this call takes significantly longer than
     * getPoints().
     * @return a pointcloud as a 2D double array
     */
    public double[][] getCartesianPoints() {
        int size = this.points;
        double[][] cartesianArray = new double[size][3];
        double angle;
        double dist;
        for(int i = 0; i < size; i++) {
            angle = points_array[i][1] * 0.0174533;
            dist = points_array[i][0];
            cartesianArray[i][0] = dist * Math.cos(angle);
            cartesianArray[i][1] = dist * Math.sin(angle);
            cartesianArray[i][2] = dist;
        }
        return cartesianArray;
    }

    /**
     * Returns the number of elements in the points array that should be read.
     * Looking beyond this many elements may result in using old data or 0s.
     * @return 
     */
    public int getSizeOfPoints() {
        return this.points;
    }

    /**
     * Gets the current entry the lidar is modifying in the points array.
     * @return the index in the points array
     */
    public int getCurrentPosInPoints() {
        return this.pos;
    }

    /**
     * Returns the milliseconds since the last time a complete
     * 360 degrees of point cloud data was returned. If this value gets
     * significantly higher than 1/rotationRate there might be a problem.
     * @return the time in milliseconds since the last full lidar update
     */
    public long getLastUpdate() {
        return System.currentTimeMillis() - this.lastPointcloudUpdate;
    }

    /**
     * Sets the minimum distance for a valid point in the pointcloud.
     * If a point is closer to the lidar than this value, it is ignored.
     * Since the lidar has a radius of 3cm. 3 is the default value for this.
     * @param distance minimum valid distance for points in cm
     */
    public void setMinDistance(int distance) {
        this.minDistance = distance;
    }

    /**
     * Call this method as a simple way to implement object detection/avoidance
     * Input the maximum number of objects to detect and the minimum
     * distance between objects. It will return a list of the closest objects
     * that match the parameters. Each element in the list is structured
     * [x,y,distance]. A good implementation is to filter out far away
     * objects, then add up the polar vector created by the angle to the object
     * and the square of the distance to the object. Then have the robot move in
     * the opposite direction of the vector with a magnitude proportioanl to the
     * magnitude of the vector. NOTE: wait a couple seconds for the lidar to start
     * up before calling this method.
     * @param count The number of objects to detect (e.g. 5)
     * @param objSpacing The minimum spacing between detected objects in cm (e.g. 60)
     * @return a list of the closest objects
     */
    public double[][] findNearbyObjects(int count, double objSpacing) {
        double[][] objects = new double[count][3];
        double[][] ptcloud = getCartesianPoints();
        double objectDist2 = Math.pow(objSpacing,2);
        for(int i = 0; i < count; i++) {
            // Get closest item in ptcloud array
            double minDist = 9999;
            double[] newObj = ptcloud[0];
            for(int j = 0; j < ptcloud.length; j++) {
                double[] pt = ptcloud[j];
                if(pt[2] < minDist) { //Potential object
                    //Make sure it isn't too close to existing objects
                    boolean ptIsGood = true;
                    for(int k = 0; k < i; k++) {
                        double dist2 = Math.pow(objects[k][0]-pt[0], 2)
                                     + Math.pow(objects[k][1]-pt[1],2);
                        if (dist2 < objectDist2) {
                            ptIsGood = false;
                            break;
                        }
                    }
                    if(ptIsGood) {
                        minDist = pt[2];
                        newObj = pt;
                    }
                }
            }
            objects[i] = newObj;
        }
        return objects;
    }


}