package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.opencv.core.Mat;
import org.opencv.core.Size;

import ftc.vision.FrameGrabber;

@Autonomous(name = "RingDetection", group = "Qualifier")
public class RingDetectionOp extends LinearOpMode  {

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
    telemetry.update();

    int startRow = 0;
    int endRow = 0;
    int startColumn = 0;
    int endColumn = 0;
    double redValue = 0;
    double greenValue = 0;
    double blueValue = 0;
    for (int i=0; i < width; i++)
    {
      for (int j=0; j < height; j++)
      {
        pixel = frame.get(i,j);
        if (pixel != null && pixel.length > 0)
        {
          redValue = pixel[0];
          greenValue = pixel[1];
          blueValue= pixel[2];
          if (redValue > 150 && greenValue > 50 && blueValue < 50) {
            if ( startRow <= 0)
              startRow = i;
            if (startColumn <= 0)
              startColumn = j;

            if (endRow < i)
              endRow = i;

            if (endColumn < j)
              endColumn = j;
          }
        }
        else
        {
          telemetry.addData("Pixel Null at",  i );
          telemetry.addData(" ",  j );
          telemetry.update();
        }
      }
    }

      double ringWidth = endRow - startRow;
      double ringHeight = endColumn - startColumn;

      double ratio =  0;
      if (ringHeight > 0.0)
        ratio = ringWidth/ringHeight;

      int numRings = 0;
      if (ratio > 1)
       numRings = 4;
      else if (ratio > 0.1)
        numRings = 1;
      else
        numRings = 0;

      telemetry.addData("Start Row ",  startRow );
      telemetry.addData("End Row ",  endRow ); //Display it on telemetry
      telemetry.addData("Start Column ",  startColumn );
      telemetry.addData("End Column ",  endColumn );
      telemetry.addData("Ratio ",  ratio );
      telemetry.addData("Number of Rings ",  numRings );
    telemetry.update();
    sleep(100000);


  }
}
