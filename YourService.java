package jp.jaxa.iss.kibo.rpc.defaultapk;

import android.content.res.AssetManager;
import android.util.Log;

import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;

import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Size;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.android.Utils;
import org.opencv.imgproc.Imgproc;

import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.List;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */
public class YourService extends KiboRpcService {
    private final String TAG = this.getClass().getSimpleName();
    private String[] TEMPLATE_FILE_NAMES;

    @Override
    protected void runPlan1() {
        // write your plan 1 here
        //First start the mission
        Log.i(TAG, "START MISSION i");
        api.startMission();

        //First destination
        Point point1 = new Point(10.9d, -9.92284d, 5.195d);
        Quaternion qua1 = new Quaternion(0f, 0f, -0.707f, 0.707f);
        Result result = api.moveTo(point1, qua1, false);
        int loop_meter = 0, laps = 5;

        while (!result.hasSucceeded() && loop_meter < laps) {
            api.moveTo(point1, qua1, true);
            ++loop_meter;
        }

        // Image Recognition and capturing
        Mat image = api.getMatNavCam();

        Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        List<Mat> corners = new ArrayList<>();
        Mat markerIds = new Mat();
        Aruco.detectMarkers(image, dictionary, corners, markerIds);

        // Get camera matrix
        Mat cameraMatrix = new Mat(3, 3, CvType.CV_64F);
        cameraMatrix.put(0, 0, api.getNavCamIntrinsics()[0]);

        // Get lens distortion parameters
        Mat cameraCoefficients = new Mat(1, 5, CvType.CV_64F);
        cameraCoefficients.put(0, 0, api.getNavCamIntrinsics()[1]);
        cameraCoefficients.convertTo(cameraCoefficients, CvType.CV_64F);

        // Undistort image
        Mat undistortImg = new Mat();
        Calib3d.undistort(image, undistortImg, cameraMatrix, cameraCoefficients);

        // Load template images
        AssetManager assetManager = getAssets();
        try {
            TEMPLATE_FILE_NAMES = assetManager.list("templates"); // Assuming templates are stored in assets/templates directory
        } catch (IOException e) {
            e.printStackTrace();
        }

        Mat[] templates = new Mat[TEMPLATE_FILE_NAMES.length];

        for (int i = 0; i < TEMPLATE_FILE_NAMES.length; i++) {
            try {
                // Open the template image file in Bitmap from the file name and convert to Mat
                InputStream inputStream = assetManager.open("templates/" + TEMPLATE_FILE_NAMES[i]);
                Bitmap bitmap = BitmapFactory.decodeStream(inputStream);
                Mat mat = new Mat();
                Utils.bitmapToMat(bitmap, mat);

                // Convert to Grayscale
                Imgproc.cvtColor(mat, mat, Imgproc.COLOR_BGR2GRAY);

                // Assign to an array of templates
                templates[i] = mat;

                inputStream.close();

            } catch (IOException e) {
                e.printStackTrace();
            }
        }

        // The number of matches for each template
        int[] templateMatchCnt = new int[templates.length];

        // Get the number of template matches
        for (int temp_num = 0; temp_num < templates.length; temp_num++) {
            // Number of matches
            int match_cnt = 0;
            // Coordinates of the template location
            List<org.opencv.core.Point> matches = new ArrayList<>();

            // Loading template image and target image
            Mat template = templates[temp_num].clone();
            Mat targetImg = undistortImg.clone();

            // Pattern matching
            int widthMin = 20;
            int widthMax = 100;
            int changeWidth = 5;
            int changeAngle = 45;

            for (int i = widthMin; i <= widthMax; i += changeWidth) {
                for (int j = 0; j <= 360; j += changeAngle) {
                    Mat resizedTemp = resizeImg(template, i);
                    Mat rotResizedTemp = rotImg(resizedTemp, j);

                    Mat outcome = new Mat();
                    Imgproc.matchTemplate(targetImg, rotResizedTemp, outcome, Imgproc.TM_CCOEFF_NORMED);

                    // Get coordinates with similarity greater than or equal to the threshold
                    double threshold = 0.8;
                    Core.MinMaxLocResult mmlr = Core.minMaxLoc(outcome);
                    double maxVal = mmlr.maxVal;
                    if (maxVal >= threshold) {
                        // Extract only results greater than or equal to the threshold
                        Mat thresholdedResult = new Mat();
                        Imgproc.threshold(outcome, thresholdedResult, threshold, 1.0, Imgproc.THRESH_TOZERO);

                        // Get match counts
                        for (int y = 0; y < thresholdedResult.rows(); y++) {
                            for (int x = 0; x < thresholdedResult.cols(); x++) {
                                if (thresholdedResult.get(y, x)[0] > 0) {
                                    matches.add(new org.opencv.core.Point(x, y));
                                }
                            }
                        }
                    }
                }
            }

            // Avoid detecting the same location multiple times
            List<org.opencv.core.Point> filteredMatches = removeDuplicates(matches);
            match_cnt += filteredMatches.size();

            // Number of matches for each template
            templateMatchCnt[temp_num] = match_cnt;
        }

        // Naming the item and destination
        int mostMatchTemplateNum = getMaxIndex(templateMatchCnt);
        api.setAreaInfo(1, TEMPLATE_FILE_NAMES[mostMatchTemplateNum], templateMatchCnt[mostMatchTemplateNum]);

        api.flashlightControlFront(0.40000f);
        api.takeTargetItemSnapshot();
        api.flashlightControlFront(0);
        api.saveMatImage(image, "Second_capture_ever.png");
        runPlan2();
    }

    @Override
    protected void runPlan2() {
        // write your plan 2 here
        // Start the second mission
        Log.i(TAG, "START MISSION ii");
        api.startMission();

        // Second destination
        Point point2 = new Point(9.815d, -9.806d, 4.293d);
        Quaternion qua2 = new Quaternion(1, 0, 0.707f, 0.707f);
        Result result2 = api.moveTo(point2, qua2, false);
        int loop_meter = 0, laps = 5;

        while (!result2.hasSucceeded() && loop_meter < laps) {
            api.moveTo(point2, qua2, true);
            ++loop_meter;
        }

        if (result2.hasSucceeded()) {
            Log.i(TAG, "Arrived at the Area_2");
        }

        // Capture the image
        Mat image2 = api.getMatNavCam();

        api.flashlightControlFront(0.40000f);
        api.takeTargetItemSnapshot();
        api.flashlightControlFront(0);
        api.saveMatImage(image2, "Third_capture_ever.png");
    }

    @Override
    protected void runPlan3() {
        // write your plan 3 here
    }

    // Resize Image
    private Mat resizeImg(Mat img, int width) {
        int height = (int) (img.rows() * ((double) width / img.cols()));
        Mat resizedImg = new Mat();
        Imgproc.resize(img, resizedImg, new Size(width, height));
        return resizedImg;
    }

    // Rotate Image
    private Mat rotImg(Mat img, int angle) {
        org.opencv.core.Point center = new org.opencv.core.Point(img.cols() / 2.0, img.rows() / 2.0);
        Mat rotatedMat = Imgproc.getRotationMatrix2D(center, angle, 1.0);
        Mat rotatedImg = new Mat();
        Imgproc.warpAffine(img, rotatedImg, rotatedMat, img.size());
        return rotatedImg;
    }

    // Remove multiple detections
    private static List<org.opencv.core.Point> removeDuplicates(List<org.opencv.core.Point> points) {
        double length = 10; // width 10 px
        List<org.opencv.core.Point> filteredList = new ArrayList<>();

        for (org.opencv.core.Point point : points) {
            boolean isInclude = false;
            for (org.opencv.core.Point checkPoint : filteredList) {
                double distance = calculateDistance(point, checkPoint);

                if (distance <= length) {
                    isInclude = true;
                    break;
                }
            }

            if (!isInclude) {
                filteredList.add(point);
            }
        }
        return filteredList;
    }

    // Find the distance between two points
    private static double calculateDistance(org.opencv.core.Point p1, org.opencv.core.Point p2) {
        double dx = p1.x - p2.x;
        double dy = p1.y - p2.y;
        return Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));
    }

    // Get the max value of an array
    private int getMaxIndex(int[] array) {
        int max = 0;
        int maxIndex = 0;

        // Find the index of the element with the largest value
        for (int i = 0; i < array.length; i++) {
            if (array[i] > max) {
                max = array[i];
                maxIndex = i;
            }
        }
        return maxIndex;
    }
}
