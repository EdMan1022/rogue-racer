package drive

import "testing"

func TestDrive(t *testing.T) {
	testDrive := Drive{}
	if testDrive.x_acceleration != 0 {
		t.Errorf("Drive not initialized")
	}
}
