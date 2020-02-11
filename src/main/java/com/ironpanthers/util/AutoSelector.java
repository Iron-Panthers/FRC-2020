package com.ironpanthers.util;

import com.ironpanthers.frc2020.Constants;

import edu.wpi.first.wpilibj.AnalogInput;

/**
 * Courtesy of M-SET Fish, Team 649, who inspired us to use a physical switch to
 * select autonomous modes. See
 * https://github.com/SaratogaMSET/Nigiri2019/blob/master/src/main/java/frc/robot/subsystems/AutoSelector.java
 * for the code. Here is the Chief Delphi post which we used for the sensor:
 * https://www.chiefdelphi.com/t/autonomous-switch/154958/18, and this is the
 * link to order the switch we used:
 */
public class AutoSelector {
	public static enum Side {
		LEFT, RIGHT
	}

	public static enum Control {
		AUTO, TELEOP
	}

	public static class PotValues {
		public static final double[] range1 = { 0.00, 0.92 };
		public static final double[] range2 = { 0.95, 0.995 };
		public static final double[] range3 = { 1.0, 1.1 };
		public static final double[] range4 = { 1.15, 1.23 };
		public static final double[] range5 = { 1.24, 1.29 };
		public static final double[] range6 = { 1.3, 1.33 };
		public static final double[] range7 = { 1.34, 1.37 };
		public static final double[] range8 = { 1.375, 1.39 };
		public static final double[] range9 = { 1.395, 1.42 };
		public static final double[] range10 = { 1.425, 1.45 };
		public static final double[] range11 = { 1.455, 1.47 };
		public static final double[] range12 = { 1.475, 1.52 };

	}

	public static class AutoSelectorValue {
		private Side autoSide;
		private int autoNumber;

		public AutoSelectorValue(Side side, int autoNumber) {
			this.autoSide = side;
			this.autoNumber = autoNumber;
		}

		public Side getSide() {
			return autoSide;
		}

		public int getAutoNumber() {
			return autoNumber;
		}

		public boolean equals(AutoSelectorValue val) {
			if (getAutoNumber() == val.getAutoNumber() && getSide() == val.getSide()) {
				return true;
			}
			return false;
		}

		public boolean equals(Side side, int num) {
			if (getAutoNumber() == num && getSide() == side) {
				return true;
			}
			return false;
		}
	}

	private AnalogInput rotary;

	public AutoSelector() {
		rotary = new AnalogInput(Constants.Auto.kAutoSelectorPort); // May need to change the offset value depending on
																	// pot
		// shift
	}

	// public String getAuto() {
	// // int auto = (int) rotary.get();
	// // if(auto == 10) auto--;
	// // chosenAuto = auto;
	// // return autos[auto];
	// }

	public double getPotVoltage() {
		return rotary.getVoltage();
	}

	public int getAutoPotNumber() {
		double val = getPotVoltage();
		if (inRange(val, PotValues.range1[0], PotValues.range1[1])) {
			return 1;
		} else if (inRange(val, PotValues.range2[0], PotValues.range2[1])) {
			return 2;
		} else if (inRange(val, PotValues.range3[0], PotValues.range3[1])) {
			return 3;
		} else if (inRange(val, PotValues.range4[0], PotValues.range4[1])) {
			return 4;
		} else if (inRange(val, PotValues.range5[0], PotValues.range5[1])) {
			return 5;
		} else if (inRange(val, PotValues.range6[0], PotValues.range6[1])) {
			return 6;
		} else if (inRange(val, PotValues.range7[0], PotValues.range7[1])) {
			return 7;
		} else if (inRange(val, PotValues.range8[0], PotValues.range8[1])) {
			return 8;
		} else if (inRange(val, PotValues.range9[0], PotValues.range9[1])) {
			return 9;
		} else if (inRange(val, PotValues.range10[0], PotValues.range10[1])) {
			return 10;
		} else if (inRange(val, PotValues.range11[0], PotValues.range11[1])) {
			return 11;
		} else if (inRange(val, PotValues.range12[0], PotValues.range12[1])) {
			return 12;
		} else {
			return 0;
		}
	}

	public void setAutoCommand() {
		switch (getAutoPotNumber()) {
		case 1:
			// if(Robot.autoCommandLeft != )
		}
	}

	public boolean inRange(double x, double lower, double upper) {
		if (x > lower && x < upper) {
			return true;
		}
		return false;
	}

}