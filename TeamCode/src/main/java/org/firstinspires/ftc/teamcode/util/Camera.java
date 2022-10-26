package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class Camera
{
    private class ConeScanner extends OpenCvPipeline
    {
        private Mat working = new Mat();

        private String name, data;

        private double red = 0.0,
                       green = 0.0,
                       blue = 0.0;

        public ConeScanner(String auto)
        {
            name = auto;
        }

        private double max(double x, double y, double z)
        {
            return Math.max(x, Math.max(y, z));
        }

        @Override
        public final Mat processFrame(Mat in)
        {
            in.copyTo(working);

            if (working.empty())
                return in;

            int x = 60,  y = 20, //160, 120
                w = 100, h = 100;

            Mat sub = working.submat(y, y + h, x, x +w);

            Imgproc.rectangle(working,
                    new Rect(x, y, w, h),
                    new Scalar(0, 255, 0));

            double   red = Core.sumElems(sub).val[1],
                   green = Core.sumElems(sub).val[2],
                    blue = Core.sumElems(sub).val[3];

            double yellow = (red + green) * 0.5;
            double magenta = (red + blue) * 0.5;

            double m = max(yellow, magenta, green);

                 if (yellow  == m) data = "Yellow";
            else if (magenta == m) data = "Magenta";
            else if (green   == m) data = "Green";

            return working;
        }
    }

    private OpenCvWebcam webcam;
    private ConeScanner scanner;

    public Camera(String name, HardwareMap map)
    {
        //find the webcam hardware device on the control hub and obtain a handle to it
        int cmv_id = map.appContext.getResources().getIdentifier("CameraMonitorViewId", "id", map.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(map.get(WebcamName.class, name), cmv_id);

        //set the calculation pipeline to the scanner class created
        scanner = new ConeScanner("TEST");
        webcam.setPipeline(scanner);

        // NOTE: timeout for obtaining permission is configurable
        webcam.setMillisecondsPermissionTimeout(2500);

        /*
            Opens up the camera device on a new thread. The synchronous single-threaded function is now deprecated, so we have to make
            a lambda function that handles success and failure of the camera opening asynchronously. If the camera is successfully opened,
            the streaming starts, which begins to repeatedly process frames through the pipeline, which is where we detect ducks/elements.
        */
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int error_code)
            {
                telemetry.addLine("Cannot open camera for viewing");
                telemetry.update();
                //telemetry.add
                // TODO: add error handling to this
            }
        });
    }
}
