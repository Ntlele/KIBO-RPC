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

import org.opencv.imgcodecs.Imgcodecs;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */
public class YourService extends KiboRpcService {
    private final String TAG = this.getClass().getSimpleName();

    @Override
    protected void runPlan1() {

        //start mission
        api.startMission();

        //Move to point
        Point point = new Point(10.9d, -9.92284d, 5.195d);
        Quaternion quaternion = new Quaternion(0f,0f,-0.707f,0.707f);
        Result destination = api.moveTo(point, quaternion, false);
        int loop_meter = 0, laps = 10;

        while (!destination.hasSucceeded() && loop_meter < laps) {
            api.moveTo(point, quaternion, true);
            ++loop_meter;
        }

        // Image Recognition and capturing
        Mat image = api.getMatNavCam();

        /*************************************************************************/
        /* write the code to recognise the type and number of items in each area */
        /*************************************************************************/

        //Detecting the AR tags
        Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        List<Mat> corners = new ArrayList<>();
        Mat markerIds = new Mat();

        Aruco.detectMarkers(image, dictionary, corners, markerIds);

        //correcting Image distortion

        //first we get the camera matrix
        Mat cameraMatrix = new Mat(3, 3, CvType.CV_64F);
        cameraMatrix.put(0, 0, api.getNavCamIntrinsics()[0]);

        //Getting the lens distortion
        Mat cameraCoefficients = new Mat(1, 5, CvType.CV_64F);
        cameraCoefficients.put(0, 0, api.getNavCamIntrinsics()[1]);
        cameraCoefficients.convertTo(cameraCoefficients, CvType.CV_64F);

        //Deforming the image captured
        Mat undistortImg = new Mat();
        Calib3d.undistort(image, undistortImg, cameraMatrix, cameraCoefficients);

        //Pattern matching

        //Load the Image files one at the time


        LoadImagesFromAssets("~/AndroidStudioProjects/templateApk/app/src/main/assets");

        Mat[] templates = new Mat[imageFileNames.length];
        for(int i = 0; i < imageFileNames.length; i++){
            try{
                //Opening the template image file in Bitmap from the file name and convert to Mat
                InputStream inputStream = getAssets().open(imageFileNames[i]);
                Bitmap bitmap = BitmapFactory.decodeStream(inputStream);
                Mat mat = new Mat();
                Utils.bitmapToMat(bitmap, mat);

                //Convert to grayscale
                Imgproc.cvtColor(mat, mat, Imgproc.COLOR_BGR2GRAY);

                //Assign to an array of templates
                templates[i] = mat;

                inputStream.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }

        //Number of matches for each template
        int templateMatchCnt[] = new int[10];

        //Get the number of template matches
        for(int tempNum = 0; tempNum < templates.length; tempNum++) {
            //Number of matches
            int matchCnt = 0;
            //Get the coordinates of the matched locations
            List<org.opencv.core.Point> matches = new ArrayList<>();

            //Loading template image and target image
            Mat template = templates[tempNum].clone();
            Mat targetImg = undistortImg.clone();

            //Pattern Matching
            int widthmin = 20, widthmax = 100, changewidth = 5;
            int changeAngle = 45;

            for(int i = widthmin; i < widthmax; i += changewidth) {
                for(int j = 0; j <= 360; j += changeAngle) {
                    Mat resizedTemp = resizeImg(template, i);
                    Mat rotResizedTemp = rotImg(resizedTemp, j);

                    Mat result = new Mat();
                    Imgproc.matchTemplate(targetImg, rotResizedTemp, result, Imgproc.TM_CCOEFF_NORMED);

                    //Get coordinates with similarity greater than or equal to the threshold
                    double threshold = 0.8;
                    Core.MinMaxLocResult mmlr = Core.minMaxLoc(result);
                    double maxVal = mmlr.maxVal;
                    if (maxVal >= threshold){
                        //We collect only results greater than or equal to the threshold
                        Mat thresholdedResult = new Mat();
                        Imgproc.threshold(result, thresholdedResult, threshold, 1.0, Imgproc.THRESH_TOZERO);

                        //Count the matches
                        for (int y = 0; y < thresholdedResult.rows(); y++){
                            for (int x = 0; x < thresholdedResult.cols(); x++){
                                if (thresholdedResult.get(y, x)[0] > 0){
                                    matches.add(new org.opencv.core.Point(x, y));
                                }
                            }
                        }
                    }

                }
            }

            // Avoid detecting the same Location multiple times
            List<org.opencv.core.Point> filteredMatches = removeDuplicates(matches);
            matchCnt += filteredMatches.size();

            // Number of matches for each template
            templateMatchCnt[tempNum] = matchCnt;
        }


        api.saveMatImage(undistortImg, "FirstUndistortedImg.png");


        int mostMatchTemplateNum = getMaxIndex(templateMatchCnt);
        //Set the type and the number of items Astrobee recognises
        api.setAreaInfo(1, "TEMPLATE_NAME[mostMatchTemplateNum]", templateMatchCnt[mostMatchTemplateNum]);



        /**************************************************/
        /* Lets move to each area and recognise the items */
        /**************************************************/



        //Phase 2

        //when moving to the front of the astronaut, report the rounding completion
        api.reportRoundingCompletion();

        /****************************************************/
        /* The code that reads what the astronaut is holding*/
        /****************************************************/

        //Let's notify the astronaut when we find the missing item
        api.notifyRecognitionItem();

        /**************************************************/
        /* move astrobee to the location of the lost item */
        /**************************************************/

        api.takeTargetItemSnapshot();

    }

//    private Mat loadTemplateImages() {
//        //List of image directories
//        String[] images = {
//                "beaker.png",
//                "goggle.png",
//                "hammer.png",
//                "kapton_tape.png",
//                "pipette.png",
//                "screwdriver.png",
//                "thermometer.png",
//                "top.png",
//                "watch.png",
//                "wrench.png"
//        };
//
//        List<String> templateImgPath = new ArrayList<>();
//        String path = "/home/pheellontlele/AndroidStudioProjects/templateApk/app/src/main/assets";
//        for (int i = 0; i < 10; i++){
//            templateImgPath.add(path + images[i]);
//        }
//
//        List<Mat> templateImages = new ArrayList<>();
//
//        for (String imagePath : templateImgPath) {
//            // Load each template image
//            Mat templateImage = Imgcodecs.imread(imagePath);
//
//            if (templateImage.empty()) {
//                Log.e(TAG, "Error: Could not load template image at " + imagePath);
//            } else {
//                Log.i(TAG, "Template image loaded successfully from " + imagePath);
//                templateImages.add(templateImage);
//
//            }
//        }
//    }

    //resize the image Captured by kibo
    private Mat resizeImg (Mat img, int width) {
        int height = (int) (img.rows() * ((double) width / img.cols()));
        Mat resizedImg = new Mat();

        Imgproc.resize(img, resizedImg, new Size(width, height));

        return resizedImg;
    }

    //Rotate the image
    private Mat rotImg (Mat img, int angle) {
        org.opencv.core.Point center = new org.opencv.core.Point(img.cols() / 2.0, img.rows() / 2.0);
        Mat rotatedMat = Imgproc.getRotationMatrix2D(center, angle, 1.0);
        Mat rotatedImg = new Mat();
        Imgproc.warpAffine(img, rotatedImg, rotatedMat, img.size());

        return rotatedImg;
    }
    //Remove multiple detections
    private static List<org.opencv.core.Point> removeDuplicates(List<org.opencv.core.Point> points){
        double length = 10;
        List<org.opencv.core.Point> filteredList = new ArrayList<>();

        for (org.opencv.core.Point point : points) {
            boolean isInclude = false;
            for (org.opencv.core.Point checkPoint : filteredList) {
                double distance = calc_distance(point, checkPoint);

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


    //FInd the distance between two points
    private static double calc_distance(org.opencv.core.Point p1, org.opencv.core.Point p2){
        double dist_x = p1.x - p2.x;
        double dist_y = p1.y - p2.y;
        return Math.sqrt(Math.pow(dist_x, 2) + Math.pow(dist_y, 2));
    }

    // Get the maximum value of an array
    private int getMaxIndex (int[] array) {
        int max = 0;
        int maxIndex = 0;

        //Find the index of the element with the largest value
        for (int i = 0; i < array.length; i++) {
            if (array[i] > max) {
                max = array[i];
                maxIndex = i;
            }
        }
        return maxIndex;
    }

    private String[] imageFileNames; // Array to store image file names


    private void LoadImagesFromAssets(String assetsFolderName) {
        // Get all image file names from the assets folder
        try {
            imageFileNames = getAssets().list(assetsFolderName);
        } catch (IOException e) {
            e.printStackTrace();
            imageFileNames = new String[0]; // Empty array in case of error
        }
    }

}
