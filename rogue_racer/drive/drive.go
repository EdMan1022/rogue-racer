package drive

import (
	"fmt"
	"math"
	"time"

	"github.com/EdMan1022/rogue-racer/rogue_racer/game"
	"github.com/g3n/engine/camera"
	"github.com/g3n/engine/core"
	"github.com/g3n/engine/geometry"
	"github.com/g3n/engine/graphic"
	"github.com/g3n/engine/gui"
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
	game         *game.Game
	model        *CarModel // car model
	engine       Engine
	rpmLabel     *gui.Label
	transmission Transmission
	wheels       map[int]Wheel
	driveLine    DriveLine
	carBody      CarBody
	commands     [CMD_LAST]bool // commands states
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

	drive.engine = drive.NewGasEngine()
	drive.transmission = drive.NewTransmission()
	drive.transmission.shiftToGear(1)
	drive.wheels = make(map[int]Wheel)
	for i := 0; i < 4; i++ {
		drive.wheels[i] = drive.NewWheel()
	}
	drive.driveLine = drive.NewOpenDiff()
	drive.carBody = drive.NewCarBody()

	// Creates car model
	drive.model = drive.newCarModel()
	game.Scene().Add(drive.model.node)

	// Creates label for the RPM
	drive.rpmLabel = gui.NewLabel(" ")
	drive.rpmLabel.SetFontSize(20)
	drive.rpmLabel.SetPosition(10, 30)
	drive.rpmLabel.SetColor4(&math32.Color4{0.8, 0.8, 0.8, 1})
	drive.rpmLabel.SetText(fmt.Sprintf("%3.1f", drive.engine.getRPM()))
	// game.GameModePanel().Add(drive.rpmLabel)
	game.MainPanel().Add(drive.rpmLabel)

	// Subscribes game to keyboard events
	game.Subscribe(window.OnKeyDown, drive.onKey)
	game.Subscribe(window.OnKeyUp, drive.onKey)
}

// Update is called to update the physics every frame
func (drive *Drive) Update(game *game.Game, deltaTime time.Duration) {

	// Handle user inputs
	// Need to have analog throttle, just update throttle position for now
	if drive.commands[CMD_FORWARD] {
		drive.engine.updateThrottlePosition(1.)
	}
	if drive.commands[CMD_BACKWARD] {
		drive.engine.updateThrottlePosition(0.)
	}
	// TODO add steering, brakes, clutch, handbrake, other car controls

	// Calculate drag force
	dragForce := drive.carBody.dragForce()
	// Get normal force at tires
	normalForce := drive.carBody.normalForce()
	// Get the max amount of net torque wheels can handle before slipping
	maxWheelTorques := make(map[int]float64)
	for i, force := range normalForce {
		drive.wheels[i].updateNormalForce(force)
		maxWheelTorques[i] = drive.wheels[i].getMaxTorque()
	}

	// Get net torque through wheels
	// Calculate engine torque at wheels
	//TODO Need to refactor all this, in neutral the engine should still see non-zero torque back in as input torque
	engineTorqueToWheels := drive.driveLine.torqueToWheels(drive.engine.getOutputTorque() * drive.transmission.currentRatio())
	// Calculate drag torque at wheels
	dragTorqueToWheels := make(map[int]float64)
	dragTorqueToWheels[0] = 0.
	dragTorqueToWheels[1] = 0.
	dragTorqueToWheels[2] = .5 * dragForce * drive.wheels[2].getWheelRadius()
	dragTorqueToWheels[3] = .5 * dragForce * drive.wheels[3].getWheelRadius()

	for i, forceFraction := range drive.carBody.weightDist() {
		dragTorqueToWheels[i] = forceFraction * dragForce * drive.wheels[i].getWheelRadius()
	}

	// Need separate torque vectors for engine and car side of wheel
	// If the wheels are slipping, the drag torque isn't acting on slowing the engine down
	// If the tires aren't slipping, then the retarding force at the wheels is the drag
	engineNetWheelTorque := make(map[int]float64)
	carNetWheelTorque := make(map[int]float64)
	transBackMultiply := 0.
	if drive.transmission.currentRatio() > .005 {
		transBackMultiply = 1. / drive.transmission.currentRatio()
	}

	for i, iDragTorque := range dragTorqueToWheels {

		if engineTorqueToWheels[i] > drive.wheels[i].getMaxTorque() {
			engineNetWheelTorque[i] = engineTorqueToWheels[i] - drive.wheels[i].getSlidingTorque()*transBackMultiply*1./drive.driveLine.getReduction()
			carNetWheelTorque[i] = drive.wheels[i].getSlidingTorque() - iDragTorque
		} else {
			engineNetWheelTorque[i] = engineTorqueToWheels[i] - iDragTorque*transBackMultiply*1./drive.driveLine.getReduction()
			carNetWheelTorque[i] = engineTorqueToWheels[i] - iDragTorque
		}

	}

	// Update each wheel's angular accel
	for i := 0; i < 4; i++ {
		drive.wheels[i].inputTorque(engineTorqueToWheels[i])
	}

	// Update engine angular accel
	netTorque := 0.
	for i := 0; i < 4; i++ {
		netTorque += engineNetWheelTorque[i]
	}

	drive.engine.inputNetTorque(netTorque)
	// Update car's linear accel
	netForce := 0.

	for i := 0; i < 4; i++ {
		netForce += carNetWheelTorque[i] * drive.wheels[i].getWheelRadius()
	}
	drive.carBody.inputForce(netForce)

	// Solve linear and angular differential equation
	for i := 0; i < 4; i++ {
		drive.wheels[i].solveODE(float64(deltaTime.Seconds()))
	}
	drive.engine.solveODE(deltaTime.Seconds())
	drive.carBody.solveODE(deltaTime.Seconds())

	// Calculates distance to move
	x_dist := drive.carBody.getXVelocity() * deltaTime.Seconds()
	// Calculates wheel rotation
	var wheelRotation = -x_dist / 0.5

	// Get car world direction
	var quat math32.Quaternion
	drive.model.node.WorldQuaternion(&quat)
	direction := math32.Vector3{1, 0, 0}
	direction.ApplyQuaternion(&quat)
	direction.Normalize()
	direction.MultiplyScalar(float32(x_dist))
	// Get car world position
	var position math32.Vector3
	drive.model.node.WorldPosition(&position)
	position.Add(&direction)
	drive.model.node.SetPositionVec(&position)

	// Update camera position using the car's directional difference
	var camPosition math32.Vector3
	game.Camera().Node.WorldPosition(&camPosition)
	camPosition.Add(&direction)
	game.Camera().Node.SetPositionVec(&camPosition)

	// Rotate the wheel caps
	for _, wcap := range drive.model.caps {
		wcap.RotateZ(float32(wheelRotation))
	}

	drive.rpmLabel.SetText(fmt.Sprintf("%3.1f", drive.engine.getRPM()))

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

type Engine interface {
	updateThrottlePosition(float64)
	getThrottlePosition() float64
	getOutputTorque() float64
	inputNetTorque(float64)
	solveODE(float64)
	getRPM() float64
}

type GasEngine struct {
	torqueCurve     map[int]float64
	momentOfInertia float64

	throttlePosition    float64
	throttlePercent     float64 // Actual throttle is different from the user input based on a throttle map
	angularPosition     float64 // Degrees
	angularVelocity     float64 // Degrees per second
	angularAcceleration float64 // Degrees per second squared
	redLine             float64
}

func (gasEngine *GasEngine) updateThrottlePosition(input float64) {
	gasEngine.throttlePosition = input
	if input == 0. {
		gasEngine.throttlePercent = .05
	} else {
		gasEngine.throttlePercent = input
	}
}

func (gasEngine *GasEngine) getThrottlePosition() float64 {
	return gasEngine.throttlePosition
}

func (gasEngine *GasEngine) getOutputTorque() float64 {
	rpm := gasEngine.angularVelocity / 6 // RPM is just angular velocity (in degrees per second) / 6
	pumpingLosses := 100000 * (.0046) / (2 * 3.1415 * 4) * (1. - gasEngine.throttlePercent)
	pumpingLosses *= (gasEngine.angularVelocity / gasEngine.redLine) // Pumping losses get larger the faster the engine is spinning
	pumpingLosses *= 5.                                              //Arbitrary scaling factor to make car feel correct
	engineOutput := gasEngine.torqueCurve[int(rpm)] * gasEngine.throttlePercent
	return engineOutput - pumpingLosses
}

func (gasEngine *GasEngine) inputNetTorque(netTorque float64) {
	gasEngine.angularAcceleration = netTorque / gasEngine.momentOfInertia
}

func (gasEngine *GasEngine) solveODE(time float64) {
	gasEngine.angularVelocity += gasEngine.angularAcceleration * time
	if gasEngine.angularVelocity >= gasEngine.redLine {
		gasEngine.angularVelocity = gasEngine.redLine
	}
	gasEngine.angularPosition += gasEngine.angularVelocity * time
}

func (gasEngine *GasEngine) getRPM() float64 {
	return gasEngine.angularVelocity / 6
}

func (drive *Drive) NewGasEngine() *GasEngine {
	engine := new(GasEngine)
	engine.throttlePosition = 0
	engine.throttlePercent = 0
	engine.angularAcceleration = 0
	engine.angularVelocity = 0
	engine.angularPosition = 0
	engine.redLine = 48000.

	crankInertiaCoefficient := 1.1
	flywheelMass := 13.
	flywheelRadius := .144
	clutchMass := 11.
	clutchRadius := .14
	engine.momentOfInertia = crankInertiaCoefficient*.5*(flywheelMass*math.Pow(flywheelRadius, 2)) + (clutchMass * math.Pow(clutchRadius, 2))

	engine.torqueCurve = make(map[int]float64)

	//TODO get torque curve from a file here or something, this is super hard coded
	peakTorque := 400.
	for i := 1; i <= 2000; i++ {
		engine.torqueCurve[i] = float64(i) / 2000 * peakTorque
	}
	for i := 2001; i <= 5000; i++ {
		engine.torqueCurve[i] = peakTorque
	}
	for i := 5001; i <= 8000; i++ {
		engine.torqueCurve[i] = -.066*float64(i) + 733.33
	}
	engine.angularVelocity = 6000.

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

type Transmission interface {
	shiftUp()
	shiftDown()
	shiftToGear(int)
	currentRatio() float64
}

type ManualTransmission struct {
	gearRatios  map[int]float64
	currentGear int
}

func (drive *Drive) NewTransmission() *ManualTransmission {
	manualTransmission := new(ManualTransmission)
	manualTransmission.currentGear = 1
	manualTransmission.gearRatios = make(map[int]float64)
	manualTransmission.gearRatios[0] = 0.
	manualTransmission.gearRatios[1] = 3.38
	manualTransmission.gearRatios[2] = 1.99
	manualTransmission.gearRatios[3] = 1.32
	manualTransmission.gearRatios[4] = 1.0
	manualTransmission.gearRatios[5] = .67
	manualTransmission.gearRatios[-1] = 3.38

	return manualTransmission
}

func (transmission *ManualTransmission) shiftUp() {
	transmission.currentGear += 1
}

func (transmission *ManualTransmission) shiftDown() {
	transmission.currentGear -= 1
}

func (transmission *ManualTransmission) shiftToGear(inputGear int) {
	transmission.currentGear = inputGear
}

func (transmission *ManualTransmission) currentRatio() float64 {
	return transmission.gearRatios[transmission.currentGear]
}

type DriveLine interface {
	torqueToWheels(float64) map[int]float64
	getReduction() float64
}

type OpenDiff struct {
	defaultTorqueSplit map[int]float64
	finalDriveRatio    float64
}

func (openDiff *OpenDiff) torqueToWheels(inputTorque float64) map[int]float64 {
	outputTorqueSplit := make(map[int]float64)
	outputTorqueSplit[0] = inputTorque * openDiff.defaultTorqueSplit[0] * openDiff.finalDriveRatio
	outputTorqueSplit[1] = inputTorque * openDiff.defaultTorqueSplit[1] * openDiff.finalDriveRatio
	outputTorqueSplit[2] = inputTorque * openDiff.defaultTorqueSplit[2] * openDiff.finalDriveRatio
	outputTorqueSplit[3] = inputTorque * openDiff.defaultTorqueSplit[3] * openDiff.finalDriveRatio

	return outputTorqueSplit
}
func (openDiff *OpenDiff) getReduction() float64 {
	return openDiff.finalDriveRatio
}

func (drive *Drive) NewOpenDiff() *OpenDiff {
	openDiff := new(OpenDiff)
	openDiff.finalDriveRatio = 3.27
	openDiff.defaultTorqueSplit = make(map[int]float64)
	openDiff.defaultTorqueSplit[0] = 0.
	openDiff.defaultTorqueSplit[1] = 0.
	openDiff.defaultTorqueSplit[2] = 0.5
	openDiff.defaultTorqueSplit[3] = 0.5
	return openDiff
}

type Wheel interface {
	updateNormalForce(float64)
	getMaxTorque() float64
	getWheelRadius() float64
	getSlidingTorque() float64
	inputTorque(float64)
	solveODE(float64)
}

type RubberWheel struct {
	radius float64
	width  float64
	// Variables
	normalForce                  float64 // Newtons
	frictionCoefficient          float64
	slidingFrictionCoefficient   float64
	rollingResistanceCoefficient float64
	pressure                     float64 //Pascals
	temperature                  float64 //Kelvin
	momentOfInertia              float64
	angularPosition              float64
	angularVelocity              float64
	angularAcceleration          float64
	contactPatch                 float64
}

func (drive *Drive) NewWheel() *RubberWheel {
	wheel := new(RubberWheel)
	wheel.radius = .326
	wheel.width = .245
	wheel.normalForce = 0.
	wheel.frictionCoefficient = 1.
	wheel.slidingFrictionCoefficient = .6
	wheel.rollingResistanceCoefficient = .014
	wheel.pressure = 220632.
	wheel.temperature = 300.
	wheel.momentOfInertia = 1.
	wheel.angularPosition = 0.
	wheel.angularVelocity = 0.
	wheel.angularAcceleration = 0.
	wheel.contactPatch = 0.

	return wheel
}

func (wheel *RubberWheel) getMaxTorque() float64 {
	return (wheel.frictionCoefficient * wheel.normalForce) * wheel.radius
}

func (wheel *RubberWheel) updateNormalForce(inputForce float64) {
	wheel.normalForce = inputForce
	wheel.contactPatch = wheel.width * .001
}

func (wheel *RubberWheel) getWheelRadius() float64 {
	return wheel.radius
}

func (wheel *RubberWheel) getSlidingTorque() float64 {
	return wheel.slidingFrictionCoefficient * wheel.normalForce * wheel.radius
}

func (wheel *RubberWheel) inputTorque(torque float64) {
	wheel.angularAcceleration = torque / wheel.momentOfInertia
}

func (wheel *RubberWheel) solveODE(time float64) {
	wheel.angularVelocity += wheel.angularAcceleration * time
	wheel.angularPosition += wheel.angularVelocity * time
}

type CarBody interface {
	dragForce() float64
	normalForce() map[int]float64
	weightDist() map[int]float64
	inputForce(float64)
	getXVelocity() float64
	solveODE(float64)
}

type SedanBody struct {
	frontalArea        float64
	dragCoefficient    float64
	fluidDensity       float64
	mass               float64
	weightDistribution map[int]float64

	xPosition           float64
	xVelocity           float64
	xAcceleration       float64
	yPosition           float64
	yVelocity           float64
	yAcceleration       float64
	netVelocity         float64
	angularPosition     float64
	angularVelocity     float64
	angularAcceleration float64
}

func (drive *Drive) NewCarBody() *SedanBody {
	carBody := new(SedanBody)
	carBody.frontalArea = 1.
	carBody.dragCoefficient = .28
	carBody.mass = 1715.
	carBody.fluidDensity = 1.225
	carBody.weightDistribution = make(map[int]float64)
	carBody.weightDistribution[0] = .25
	carBody.weightDistribution[1] = .25
	carBody.weightDistribution[2] = .25
	carBody.weightDistribution[3] = .25

	carBody.xPosition = 0.
	carBody.xVelocity = 0.
	carBody.xAcceleration = 0.
	carBody.yPosition = 0.
	carBody.yVelocity = 0.
	carBody.yAcceleration = 0.
	carBody.netVelocity = 0.

	carBody.angularPosition = 0.
	carBody.angularVelocity = 0.
	carBody.angularAcceleration = 0.

	return carBody

}

func (carBody *SedanBody) dragForce() float64 {
	return .5 * carBody.fluidDensity * math.Pow(carBody.netVelocity, 2) * carBody.dragCoefficient * carBody.frontalArea
}

func (carBody *SedanBody) normalForce() map[int]float64 {
	normalForceAtWheels := make(map[int]float64)
	normalForceAtWheels[0] = carBody.mass * 9.8 * carBody.weightDistribution[0]
	normalForceAtWheels[1] = carBody.mass * 9.8 * carBody.weightDistribution[1]
	normalForceAtWheels[2] = carBody.mass * 9.8 * carBody.weightDistribution[2]
	normalForceAtWheels[3] = carBody.mass * 9.8 * carBody.weightDistribution[3]
	return normalForceAtWheels
}

func (carBody *SedanBody) weightDist() map[int]float64 {
	return carBody.weightDistribution
}

func (carBody *SedanBody) inputForce(force float64) {
	carBody.xAcceleration = force / carBody.mass
}

func (carBody *SedanBody) getXVelocity() float64 {
	return carBody.xVelocity
}

func (carBody *SedanBody) solveODE(time float64) {
	carBody.xVelocity += carBody.xAcceleration * time
	carBody.xPosition += carBody.xVelocity * time

	carBody.yVelocity += carBody.yAcceleration * time
	carBody.yPosition += carBody.yVelocity * time
}
