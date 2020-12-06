package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.opencv.core.Mat;
import org.opencv.core.Size;

import ftc.vision.BeaconColorResult;
import ftc.vision.FrameGrabber;
import ftc.vision.ImageProcessorResult;

@Autonomous(name = "BeaconDetection", group = "Qualifier")
public class BeaconDetectionOp extends LinearOpMode  {

  @Override
  public void runOpMode() throws InterruptedException {
    waitForStart();
    FrameGrabber frameGrabber = FtcRobotControllerActivity.frameGrabber; //Get the frameGrabber

    frameGrabber.grabSingleFrame(); //Tell it to grab a frame
    while (!frameGrabber.isResultReady()) { //Wait for the result
      Thread.sleep(5); //sleep for 5 milliseconds
    }

    //Get the result
   //ImageProcessorResult imageProcessorResult = frameGrabber.getResult();
    //Mat frame = imageProcessorResult.getFrame();
    Mat frame = frameGrabber.getFrame();

    double[] pixel;
    Size size = frame.size();
    double height = size.height;
    double width = size.width;
    //pixel = frame.get(100,100);
    telemetry.addData("size", size ); //Display it on telemetry
    telemetry.addData("height", height ); //Display it on telemetry
    telemetry.addData("width", width ); //Display it on telemetry
    int startRow = 0;
    int endRow = 0;
    int startColumn = 0;

    for (int i=0; i < width; i++)
    {
      for (int j=0; j < height; j++)
      {

      }
    }

    for (int i = 0; i< pixel.length; i++)
    {
      telemetry.addData("Pixel ",  i );
      telemetry.addData(" ",  pixel[i] ); //Display it on telemetry
    }

    telemetry.update();
    sleep(100000);
    /*
    BeaconColorResult result = (BeaconColorResult) imageProcessorResult.getResult();

    BeaconColorResult.BeaconColor leftColor = result.getLeftColor();
    BeaconColorResult.BeaconColor rightColor = result.getRightColor();

    telemetry.addData("Result", result); //Display it on telemetry
    telemetry.update();
    //wait before quitting (quitting clears telemetry)
    Thread.sleep(5000);*/


  }
}
