package drive

import (
	"math"
	"time"

	"github.com/EdMan1022/rogue-racer/rogue_racer/game"
	"github.com/g3n/engine/camera"
	"github.com/g3n/engine/core"
	"github.com/g3n/engine/geometry"
	"github.com/g3n/engine/graphic"
	"github.com/g3n/engine/light"
	"github.com/g3n/engine/material"
	"github.com/g3n/engine/math32"
	"github.com/g3n/engine/util/helper"
	"github.com/g3n/engine/window"
)

const (
	CMD_FORWARD = iota
	CMD_BACKWARD
	CMD_LEFT
	CMD_RIGHT
	CMD_LAST
)

type Drive struct {
	game            *game.Game
	x_velocity      float32        // linear velocity in x direction (m/s)
	x_acceleration  float32        // acceleration value for the x direction (m/s^2)
	x_decceleration float32        // Negative acceleration value for the x direction, car should brake differently than accelerating (m/s^2)
	z_velocity      float32        // linear velocity in z direction (m/s)
	model           *CarModel      // car model
	commands        [CMD_LAST]bool // commands states
}

// Start is called once at the start of a drive
func (drive *Drive) Start(game *game.Game) {
	drive.game = game
	game.Orbit().SetEnabled(^camera.OrbitKeys)

	// Add directional white light
	light_1 := light.NewDirectional(&math32.Color{1, 1, 1}, 1.0)
	light_1.SetPosition(10, 10, 10)
	game.Scene().Add(light_1)

	// Show grid helper
	grid := helper.NewGrid(100, 1, &math32.Color{0.4, 0.4, 0.4})
	game.Scene().Add(grid)

	// Sets camera position
	game.Camera().SetPosition(0, 4, 10)
	pos := game.Camera().Position()
	game.Camera().UpdateSize(pos.Length())
	game.Camera().LookAt(&math32.Vector3{0, 0, 0}, &math32.Vector3{0, 1, 0})

	// Creates car model
	drive.model = drive.newCarModel()
	drive.x_acceleration = 5.0
	drive.x_decceleration = 10.0
	drive.x_velocity = 0.0
	drive.z_velocity = 0.0
	game.Scene().Add(drive.model.node)

	// Subscribes game to keyboard events
	game.Subscribe(window.OnKeyDown, drive.onKey)
	game.Subscribe(window.OnKeyUp, drive.onKey)
}

// Update is called to update the physics every frame
func (drive *Drive) Update(game *game.Game, deltaTime time.Duration) {

	if drive.commands[CMD_FORWARD] {
		x_velocity_delta := drive.x_acceleration * float32(deltaTime.Seconds())
		drive.x_velocity = drive.x_velocity + x_velocity_delta
	}
	if drive.commands[CMD_BACKWARD] {
		x_velocity_delta := drive.x_decceleration * float32(deltaTime.Seconds())
		drive.x_velocity = drive.x_velocity - x_velocity_delta
		drive.x_velocity = math32.Max(drive.x_velocity, 0.0)
	}

	// Calculates distance to move
	x_dist := drive.x_velocity * float32(deltaTime.Seconds())
	// Calculates wheel rotation
	var wheelRotation = -x_dist / 0.5

	// Get car world direction
	var quat math32.Quaternion
	drive.model.node.WorldQuaternion(&quat)
	direction := math32.Vector3{1, 0, 0}
	direction.ApplyQuaternion(&quat)
	direction.Normalize()
	direction.MultiplyScalar(x_dist)
	// Get car world position
	var position math32.Vector3
	drive.model.node.WorldPosition(&position)
	position.Add(&direction)
	drive.model.node.SetPositionVec(&position)
	// Rotate the wheel caps
	for _, wcap := range drive.model.caps {
		wcap.RotateZ(wheelRotation)
	}
}

// Cleanup is called when a drive ends
func (drive *Drive) Cleanup(game *game.Game) {}

// Process keyboard events
func (drive *Drive) onKey(evname string, ev interface{}) {
	var state bool
	if evname == window.OnKeyDown {
		state = true
	} else {
		state = false
	}
	kev := ev.(*window.KeyEvent)
	switch kev.Key {
	case window.KeyW:
		drive.commands[CMD_FORWARD] = state
	case window.KeyS:
		drive.commands[CMD_BACKWARD] = state
	case window.KeyA:
		drive.commands[CMD_LEFT] = state
	case window.KeyD:
		drive.commands[CMD_RIGHT] = state
	}
}

type CarModel struct {
	node     *core.Node // node with all car meshes
	meshBase *graphic.Mesh
	caps     []*graphic.Mesh
}

// Builds and returns a new car model with separate meshes
// for the wheels, wheel caps, and car base
func (drive *Drive) newCarModel() *CarModel {
	const EPS = 0.01
	const BASE_WIDTH = 3.0
	const BASE_HEIGHT = 0.6
	const BASE_LENGTH = 4.4
	var BASE_COLOR = math32.NewColorHex(0x10C010)
	const WHEEL_RADIUS = 0.5
	const WHEEL_WIDTH = (BASE_WIDTH / 2) * 0.2
	var WHEEL_COLOR = math32.NewColorHex(0x5706a5)
	const EMBED = 0.5
	const CAP_RADIUS = WHEEL_RADIUS - 0.1
	const CAP_DZ = 0.01
	var CAP_COLOR = math32.NewColorHex(0xA52A2A)

	model := new(CarModel)
	model.node = core.NewNode()

	// texfile := drive.game.DirData() + "/images/wheel.png"
	// tex, err := texture.NewTexture2DFromImage(texfile)
	// if err != nil {
	// 	drive.game.Log().Fatal("Error:%s loading texture:%s", err, texfile)
	// }

	matWheel := material.NewStandard(WHEEL_COLOR)
	matCap := material.NewStandard(CAP_COLOR)
	// matCap.AddTexture(tex)
	var zWheel float32 = BASE_WIDTH/2 - WHEEL_WIDTH/2 - 0.1
	// Create front wheels
	for sideN := 0; sideN < 2; sideN++ {
		geomWheel := geometry.NewCylinder(WHEEL_RADIUS, WHEEL_WIDTH+EPS, 20, 20, true, true)
		meshWheel := graphic.NewMesh(geomWheel, matWheel)
		var zdir float32 = 1.0
		if sideN%2 == 0 {
			zdir = -1.0
		}
		meshWheel.SetPosition(
			BASE_LENGTH/2-WHEEL_RADIUS,
			WHEEL_RADIUS,
			zdir*zWheel,
		)
		meshWheel.SetRotationX(math32.Pi / 2)
		model.node.Add(meshWheel)
		// Add cap
		geomCap := geometry.NewDisk(CAP_RADIUS, 20)
		meshCap := graphic.NewMesh(geomCap, matCap)
		meshCap.SetPositionX(BASE_LENGTH/2 - WHEEL_RADIUS)
		meshCap.SetPositionY(WHEEL_RADIUS)
		zWheelCap := zWheel + WHEEL_WIDTH/2 + CAP_DZ
		if sideN%2 != 0 {
			meshCap.SetPositionZ(zWheelCap)
		} else {
			meshCap.SetPositionZ(-zWheelCap)
		}
		// Rotate the wheel cap circle geometry
		if sideN%2 == 0 {
			geomCap.ApplyMatrix(math32.NewMatrix4().MakeRotationX(-math32.Pi))
		}
		model.caps = append(model.caps, meshCap)
		model.node.Add(meshCap)

	}
	// Create back wheels
	for sideN := 0; sideN < 2; sideN++ {
		geomWheel := geometry.NewCylinder(WHEEL_RADIUS, WHEEL_WIDTH+EPS, 20, 20, true, true)
		meshWheel := graphic.NewMesh(geomWheel, matWheel)
		var zdir float32 = 1.0
		if sideN%2 == 0 {
			zdir = -1.0
		}
		meshWheel.SetPosition(
			-(BASE_LENGTH/2)+WHEEL_RADIUS,
			WHEEL_RADIUS,
			zdir*zWheel,
		)
		meshWheel.SetRotationX(math32.Pi / 2)
		model.node.Add(meshWheel)
		// Add cap
		geomCap := geometry.NewDisk(CAP_RADIUS, 20)
		meshCap := graphic.NewMesh(geomCap, matCap)
		meshCap.SetPositionX(-(BASE_LENGTH / 2) + WHEEL_RADIUS)
		meshCap.SetPositionY(WHEEL_RADIUS)
		zWheelCap := zWheel + WHEEL_WIDTH/2 + CAP_DZ
		if sideN%2 != 0 {
			meshCap.SetPositionZ(zWheelCap)
		} else {
			meshCap.SetPositionZ(-zWheelCap)
		}
		// Rotate the wheel cap circle geometry
		if sideN%2 == 0 {
			geomCap.ApplyMatrix(math32.NewMatrix4().MakeRotationX(-math32.Pi))
		}
		model.caps = append(model.caps, meshCap)
		model.node.Add(meshCap)
	}

	// Create the car chassis and add it to the group
	geomBase := geometry.NewBox(BASE_LENGTH, BASE_WIDTH, BASE_HEIGHT)
	matBase := material.NewStandard(BASE_COLOR)
	model.meshBase = graphic.NewMesh(geomBase, matBase)
	model.meshBase.SetPosition(
		0,
		2*WHEEL_RADIUS+(BASE_HEIGHT/2)-EMBED,
		0,
	)
	model.meshBase.SetRotationX(math32.Pi / 2)
	model.node.Add(model.meshBase)
	return model
}

type Motor interface {
	changeThrottlePosition(float32)
	getOutputTorque() float32
	giveNegativeTorque(float32)
	Update(*game.GameMode)
	getEnginePower() float32
}

type GasEngine struct {
	cylinders        int32
	displacement     float32
	horsepower       int32
	rpm              int32
	throttlePosition float32
	angularMomentum  float64
	peakRpm          float32
	peakPower        float32
}

func (gasEngine *GasEngine) changeThrottlePosition(input float32) {
	gasEngine.throttlePosition = input
}

func (gasEngine *GasEngine) getOutputTorque() float32 {
	return 0.0
}

func (gasEngine *GasEngine) giveNegativeTorque(float32) {

}

func (gasEngine *GasEngine) getEnginePower() float64 {
	throttleModifier := 1. / (1. + math.Exp(float64(8*(gasEngine.throttlePosition-0.5))))
	return 0.05 * float64(gasEngine.rpm) * throttleModifier
}

func (gasEngine *GasEngine) Update() {}

func (drive *Drive) NewGasEngine() *GasEngine {
	engine := new(GasEngine)
	engine.cylinders = 8
	engine.displacement = 4.6
	engine.horsepower = 320
	engine.rpm = 0
	engine.throttlePosition = 0

	crankInertiaCoefficient := 1.1
	flywheelMass := 13.
	flywheelRadius := .144
	clutchMass := 11.
	clutchRadius := .14
	engine.angularMomentum = crankInertiaCoefficient*.5*(flywheelMass*math.Pow(2, flywheelRadius)) + (clutchMass * math.Pow(2, clutchRadius))

	return engine
}

// Process user inputs;
// Update engine throttle position based on throttle input value
// Update brake pressure based on brake input value
// Update steering rack input based on steering input value

// Calculate single step of the physics simulation
// Get current output torque from engine
// Calculate torque through transmission to each wheel
// Calculate whether wheels slip or not
// Send back torque to engine (based on wheel slip)
// Calculate engine RPM change
