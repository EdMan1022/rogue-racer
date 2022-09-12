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
	if engine.cylinders != 8 {
		t.Errorf("Really need a v8 here")
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
	engine.rpm = 3000
	outputTorque = engine.getOutputTorque()
	if outputTorque != 0. {
		t.Errorf("Incorrect torque output")
	}
}

func TestInputNegativeTorque(t *testing.T) {
	drive := &Drive{}
	engine := drive.NewGasEngine()
	engine.giveNegativeTorque(100)

}

// Engine should return the current power output (a function of it's RPM and throttle position)
func TestGetEnginePower(t *testing.T) {
	drive := &Drive{}
	engine := drive.NewGasEngine()
	engine.rpm = 1000
	engine.throttlePosition = .5
	currentPower := engine.getEnginePower()
	expectedPower := float64(25.0)
	if currentPower != expectedPower {
		t.Errorf("Engine generating incorrect power. Expected: %f\n Actual: %f", expectedPower, currentPower)
	}
}
