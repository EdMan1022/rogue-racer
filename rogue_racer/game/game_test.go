package game

import (
	"testing"
	"time"
)

type SpyGameMode struct {
	Calls int
}

func (spy *SpyGameMode) Start(game *Game) {
	spy.Calls++
}
func (spy *SpyGameMode) Update(game *Game, deltaTime time.Duration) {
	spy.Calls++
}
func (spy *SpyGameMode) Cleanup(game *Game) {
	spy.Calls++
}

func TestGame(t *testing.T) {
	want := "Rogue Racer"
	spyGameMode := &SpyGameMode{}
	got := Create(spyGameMode)
	if got.name != want {
		t.Errorf("Hello() = %q, want %q", got.name, want)
	}
}
