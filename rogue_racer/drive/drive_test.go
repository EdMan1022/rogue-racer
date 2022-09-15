package drive

import "testing"

func TestDrive(t *testing.T) {
	testDrive := Drive{}
	if testDrive.x_acceleration != 0 {
		t.Errorf("Drive not initialized")
	}
}

func TestEngine(t *testing.T) {
	drive := &Drive{}
	engine := drive.NewGasEngine()
	if engine.angularVelocity != 0 {
		t.Errorf("Engine should be created with 0 RPM")
	}
}

func TestEngineThrottlePosition(t *testing.T) {
	drive := &Drive{}
	engine := drive.NewGasEngine()
	engine.angularVelocity = 24000.

	engine.updateThrottlePosition(.5)

	if engine.getOutputTorque() != 200. {
		t.Errorf("Incorrect torque, at half throttle should be 200 Nm, got %f Nm", engine.getOutputTorque())
	}

}

func TestOutputTorque(t *testing.T) {
	drive := &Drive{}
	engine := drive.NewGasEngine()

	outputTorque := engine.getOutputTorque()
	if outputTorque != 0.0 {
		t.Errorf("Output torque should be 0 at 0 rpm")
	}
	// Set engine RPM to half peak
	engine.throttlePercent = 1.
	engine.angularVelocity = 24000.
	outputTorque = engine.getOutputTorque()
	if outputTorque != 400. {
		t.Errorf("Incorrect torque output")
	}
}

func TestInputNetTorque(t *testing.T) {
	drive := &Drive{}
	engine := drive.NewGasEngine()
	engine.momentOfInertia = 20.
	engine.inputNetTorque(100)
	if engine.angularAcceleration != 5. {
		t.Errorf("Net torque not updating engine correctly")
	}

}

func TestTransmissionShiftUp(t *testing.T) {
	drive := &Drive{}
	transmission := drive.NewTransmission()
	transmission.currentGear = 1
	transmission.shiftUp()
	if transmission.currentGear != 2 {
		t.Errorf("transmission didn't shift correctly")
	}
}
func TestShiftTooHigh(t *testing.T) {
	drive := &Drive{}
	transmission := drive.NewTransmission()
	transmission.currentGear = 5
	transmission.shiftUp()
	output := transmission.currentRatio()
	if output != .67 {
		t.Errorf("Shouldn't be able to shift past 5")
	}
}

func TestTransmissionCurrentRatio(t *testing.T) {
	drive := &Drive{}
	transmission := drive.NewTransmission()
	transmission.currentGear = 1
	if transmission.currentRatio() != 3.38 {
		t.Errorf("transmission not providing correct gear ratio")
	}
}

func TestOpenDiffTorqueSplit(t *testing.T) {
	drive := &Drive{}
	diff := drive.NewOpenDiff()
	torqueSplit := diff.torqueToWheels(100.)
	if torqueSplit[0] != 0. || torqueSplit[1] != 0. {
		t.Errorf("Rear Wheel Drive shouldn't send torque to either front wheel")
	}
	if torqueSplit[2] != 50. || torqueSplit[3] != 50. {
		t.Errorf("Torque should be split evenly between rear wheels. Left Rear: %f Nm, Right Rear: %f Nm", torqueSplit[2], torqueSplit[3])
	}
}

func TestCreateWheels(t *testing.T) {
	drive := &Drive{}
	wheel1 := drive.NewWheel()
	wheel1.updateNormalForce(10.)

	wheel2 := drive.NewWheel()
	wheel2.updateNormalForce(5.)

	if wheel1.normalForce == wheel2.normalForce {
		t.Errorf("Wheels should have different normal forces")
	}

}

// A change in normal force on a wheel should change the contact patch
func TestUpdateWheelNormalForce(t *testing.T) {
	drive := &Drive{}
	wheel := drive.NewWheel()
	wheel.updateNormalForce(10.)

	if wheel.contactPatch != 5. {
		t.Errorf("Contact patch not updated correctly")
	}

}

// Test that the maximum torque provided by the wheel is correct
func TestWheelMaxTorque(t *testing.T) {
	drive := &Drive{}
	wheel := drive.NewWheel()

	wheel.updateNormalForce(10.)
	if wheel.getMaxTorque() != 3.2600000000000002 {
		t.Errorf("Wheel not calculating friction correctly: %f", wheel.getMaxTorque())
	}

}

func TestDragForce(t *testing.T) {
	drive := &Drive{}
	car := drive.NewCarBody()
	car.netVelocity = 1.
	expectedDrag := .171500
	if car.dragForce() != expectedDrag {
		t.Errorf("Not calculating drag correctly\nExpected: %f\nActual: %f", expectedDrag, car.dragForce())
	}
}

func TestCarNormalForce(t *testing.T) {
	drive := &Drive{}
	car := drive.NewCarBody()
	actualNormalForce := car.normalForce()
	expectedNormalForce := make(map[int]float64)
	expectedNormalForce[0] = 4201.75
	expectedNormalForce[1] = 4201.75
	expectedNormalForce[2] = 4201.75
	expectedNormalForce[3] = 4201.75
	if actualNormalForce[0] != expectedNormalForce[0] {
		t.Errorf("Not getting normal force correctly\nExpected: %f\nActual: %f", expectedNormalForce[0], actualNormalForce[0])
	}

}
