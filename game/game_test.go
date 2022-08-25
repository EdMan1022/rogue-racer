package game

import "testing"

func TestGame(t *testing.T) {
	want := "Rogue Racer"
	if got := Create(); got.name != want {
		t.Errorf("Hello() = %q, want %q", got.name, want)
	}
}
