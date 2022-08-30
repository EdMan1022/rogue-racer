package main

import (
	"github.com/EdMan1022/rogue-racer/rogue_racer/drive"
	"github.com/EdMan1022/rogue-racer/rogue_racer/game"
)

func main() {

	// Create and run game
	game.Create(&drive.Drive{}).Run()
}
