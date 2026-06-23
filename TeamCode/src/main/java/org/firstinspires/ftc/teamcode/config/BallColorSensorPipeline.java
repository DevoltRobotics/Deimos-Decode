package org.firstinspires.ftc.teamcode.config;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class BallColorSensorPipeline extends OpenCvPipeline {
	private Mat inputRoi = null;
	
	private Mat inputRoiHSV = new Mat();
	
	private Scalar inputRoiHSVAvg = new Scalar(0, 0, 0, 0);
	
	private Scalar inputRoiAvg = new Scalar(0, 0, 0, 0);
	
	private Mat inputRects = new Mat();
	private double[] exportedData = new double[0];
	
	@Override
	public Mat processFrame(Mat input) {
		// "Extract Region (#176)"
		if(inputRoi != null) {
			inputRoi.release();
		}
	
		Rect regionRect = new Rect(330, 120, 230, 230);
		if(regionRect != null) {
			this.inputRoi = input.submat(regionRect);
		} else {
			this.inputRoi = input;
		}
	
		// "Convert Color (#183)"
		Imgproc.cvtColor(inputRoi, inputRoiHSV, Imgproc.COLOR_RGB2HSV);
	
		// "Average Color (#184)"
		this.inputRoiHSVAvg = Core.mean(inputRoiHSV);
	
		// "Average Color (#185)"
		this.inputRoiAvg = Core.mean(inputRoi);
	
		// "Draw Rectangles (#180)"
		input.copyTo(inputRects);
		Imgproc.rectangle(inputRects, new Rect(330, 120, 230, 230), inputRoiAvg, 17);
	
		setExportedData(new double[] { inputRoiHSVAvg.val[0], inputRoiHSVAvg.val[1], inputRoiHSVAvg.val[2] });
		return inputRects;
	}
	
	private synchronized void setExportedData(double[] data) {
		this.exportedData = data;
	}
	
	public synchronized double[] getExportedData() {
		return exportedData;
	}
	
	public synchronized double getExportedData(int index) {
		if(index >= exportedData.length) {
			return 0.0;
		} else {
			return exportedData[index];
		}
	}
}