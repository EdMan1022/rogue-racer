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
