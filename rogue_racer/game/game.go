package game

import (
	"fmt"
	"os"
	"path/filepath"
	"strings"
	"time"

	"github.com/g3n/engine/app"
	"github.com/g3n/engine/audio/al"
	"github.com/g3n/engine/camera"
	"github.com/g3n/engine/core"
	"github.com/g3n/engine/gls"
	"github.com/g3n/engine/gui"
	"github.com/g3n/engine/light"
	"github.com/g3n/engine/math32"
	"github.com/g3n/engine/renderer"
	"github.com/g3n/engine/util"
	"github.com/g3n/engine/util/logger"
	"github.com/g3n/engine/window"
	"github.com/kardianos/osext"
)

type Game struct {
	*app.Application
	log         *logger.Logger
	currentMode GameMode // Current game mode
	name        string
	scene       *core.Node     // Scene rendered
	modeScene   *core.Node     // Scene populated by individual game modes
	ambLight    *light.Ambient // Scene ambient light
	frameRater  *util.FrameRater
	dirData     string // Full path of the data directory

	// GUI
	mainPanel     *gui.Panel
	gameModePanel *gui.Panel
	labelFPS      *gui.Label

	camera *camera.Camera
	orbit  *camera.OrbitControl
}

type GameMode interface {
	Start(*Game)
	Update(*Game, time.Duration)
	Cleanup(*Game)
}

func Create(gameMode GameMode) *Game {
	game := new(Game)
	game.Application = app.App()
	game.name = "Rogue Racer"
	game.log = logger.New("G3ND", nil)
	game.log.AddWriter(logger.NewConsole(false))
	game.log.SetFormat(logger.FTIME | logger.FMICROS)
	game.log.SetLevel(logger.DEBUG)

	game.modeScene = core.NewNode()
	game.scene = core.NewNode()
	game.scene.Add(game.modeScene)
	width, height := game.GetSize()
	aspect := float32(width) / float32(height)
	game.camera = camera.New(aspect)
	game.scene.Add(game.camera)
	game.orbit = camera.NewOrbitControl(game.camera)

	game.ambLight = light.NewAmbient(&math32.Color{1.0, 1.0, 1.0}, 0.5)
	game.scene.Add(game.ambLight)

	// Create frame rater
	game.frameRater = util.NewFrameRater(60)
	game.buildGUI()

	// Check for data directory and abort if none found
	game.dirData = game.checkDirData("data")
	game.log.Info("Using data diretory:%s", game.dirData)

	// Sets the default window resize event handler
	game.Subscribe(window.OnWindowSize, func(evname string, ev interface{}) { game.OnWindowResize() })
	game.OnWindowResize()

	// Subscribe to key events
	game.Subscribe(window.OnKeyDown, func(evname string, ev interface{}) {
		kev := ev.(*window.KeyEvent)
		if kev.Key == window.KeyEscape {
			game.Exit()
		}
	})

	// Set up scene
	game.setupScene()
	game.log.Info("Game starting")

	// Start a drive manually
	game.currentMode = gameMode
	game.currentMode.Start(game)
	return game
}

func (game *Game) checkDirData(dirDataName string) string {
	// Check first if data directory is in the current directory
	if _, err := os.Stat(dirDataName); err == nil {
		dirData, err := filepath.Abs(dirDataName)
		if err != nil {
			panic(err)
		}
		return dirData
	}

	// Get the executable path
	execPath, err := osext.Executable()
	if err != nil {
		panic(err)
	}
	// Checks if data directory is in the executable directory
	execDir := filepath.Dir(execPath)
	path := filepath.Join(execDir, dirDataName)
	if _, err := os.Stat(path); err == nil {
		return path
	}
	// Assumes the executable is in $GOPATH/bin
	goPath := filepath.Dir(execDir)
	path = filepath.Join(goPath, "src", "github.com", "g3n", "g3nd", dirDataName)
	// Checks data path
	if _, err := os.Stat(path); err == nil {
		return path
	}

	// If the data directory hasn't been found, manually scan the $GOPATH directories
	rawPaths := os.Getenv("GOPATH")
	paths := strings.Split(rawPaths, ":")
	for _, j := range paths {
		// Checks data path
		path = filepath.Join(j, "src", "github.com", "g3n", "g3nd", dirDataName)
		if _, err := os.Stat(path); err == nil {
			return path
		}
	}

	// Show error message and aborts
	game.log.Fatal("Data directory NOT FOUND")
	return ""
}

func (game *Game) buildGUI() {
	width, height := game.GetSize()
	game.mainPanel = gui.NewPanel(float32(width), float32(height))
	dl := gui.NewDockLayout()
	game.mainPanel.SetRenderable(false)
	game.mainPanel.SetEnabled(false)
	game.mainPanel.SetLayout(dl)
	game.scene.Add(game.mainPanel)
	gui.Manager().Set(game.mainPanel)

	// Create demoPanel to house GUI elements created by the demos
	game.gameModePanel = gui.NewPanel(0, 0)
	game.gameModePanel.SetColor4(&gui.StyleDefault().Scroller.BgColor)
	game.gameModePanel.SetLayoutParams(&gui.DockLayoutParams{Edge: gui.DockCenter})
	// game.mainPanel.Add(game.gameModePanel)

	// Adds header after the gui central panel to ensure that the control folder
	// stays over the gui panel when opened.
	headerColor := math32.Color4{13.0 / 256.0, 41.0 / 256.0, 62.0 / 256.0, 1}
	lightTextColor := math32.Color4{0.8, 0.8, 0.8, 1}
	header := gui.NewPanel(600, 40)
	header.SetBorders(0, 0, 1, 0)
	header.SetPaddings(4, 4, 4, 4)
	header.SetColor4(&headerColor)
	header.SetLayoutParams(&gui.DockLayoutParams{Edge: gui.DockTop})

	// Horizontal box layout for the header
	hbox := gui.NewHBoxLayout()
	header.SetLayout(hbox)
	game.mainPanel.Add(header)
	const fontSize = 20

	// FPS
	if true {
		l1 := gui.NewLabel(" ")
		l1.SetFontSize(fontSize)
		l1.SetLayoutParams(&gui.HBoxLayoutParams{AlignV: gui.AlignCenter})
		l1.SetText("  FPS: ")
		l1.SetColor4(&lightTextColor)
		header.Add(l1)
		// FPS value
		game.labelFPS = gui.NewLabel(" ")
		game.labelFPS.SetFontSize(fontSize)
		game.labelFPS.SetLayoutParams(&gui.HBoxLayoutParams{AlignV: gui.AlignCenter})
		game.labelFPS.SetColor4(&lightTextColor)
		header.Add(game.labelFPS)
	}

}

// setupScene resets the current scene for executing a new mode
func (game *Game) setupScene() {
	if game.currentMode != nil {
		game.currentMode.Cleanup(game)
	}

	// Destroy all objects in mode scene and GUI
	// game.modeScene.DisposeChildren(true)

	// Clear subscriptions
	game.UnsubscribeAllID(game)

	// Clear all custom cursors and reset current cursor
	game.DisposeAllCustomCursors()
	game.SetCursor(window.ArrowCursor)

	// Set a default background color
	game.Gls().ClearColor(0.6, 0.6, 0.6, 1.0)

	// Reset renderer z-sorting flag
	game.Renderer().SetObjectSorting(true)

	// Reset ambient light
	game.ambLight.SetColor(&math32.Color{1.0, 1.0, 1.0})
	game.ambLight.SetIntensity(0.5)

	// Reset camera
	game.camera.SetPosition(0, 0, 5)
	game.camera.UpdateSize(5)
	game.camera.LookAt(&math32.Vector3{0, 0, 0}, &math32.Vector3{0, 1, 0})
	game.camera.SetProjection(camera.Perspective)
	game.orbit.Reset()

	// If audio active, resets global listener parameters
	al.Listener3f(al.Position, 0, 0, 0)
	al.Listener3f(al.Velocity, 0, 0, 0)
	al.Listenerfv(al.Orientation, []float32{0, 0, -1, 0, 1, 0})

	// TODO add menu stuff
}

// Returns the base directory for data
func (game *Game) DirData() string {
	return game.dirData
}

// AmbLight returns the dfault scene ambient light
func (game *Game) AmbLight() *light.Ambient {
	return game.ambLight
}

// Log returns the application logger
func (game *Game) Log() *logger.Logger {
	return game.log
}

// Scene returns the current application 3D scene
func (game *Game) Scene() *core.Node {
	return game.modeScene
}

// Camera returns the current application camera
func (game *Game) Camera() *camera.Camera {
	return game.camera
}

// Orbit retuns the current camera orbit control
func (game *Game) Orbit() *camera.OrbitControl {
	return game.orbit
}

func (game *Game) GameModePanel() *gui.Panel {
	return game.gameModePanel
}
func (game *Game) MainPanel() *gui.Panel {
	return game.mainPanel
}

// OnWindowResize is default handler for window resize events
func (game *Game) OnWindowResize() {

	// Get framebuffer size and set the viewport accordingly
	width, height := game.GetFramebufferSize()
	game.Gls().Viewport(0, 0, int32(width), int32(height))

	// Set the camera aspect ratio
	game.camera.SetAspect(float32(width) / float32(height))

	// TODO gui stuff here
}

// Run runs the app's render loop with the games update function
func (game *Game) Run() {
	game.Application.Run(game.Update)
}

func (game *Game) Update(renderer *renderer.Renderer, deltaTime time.Duration) {

	game.frameRater.Start()

	// Clear the color, depth, and stencil buffers
	game.Gls().Clear(gls.COLOR_BUFFER_BIT | gls.DEPTH_BUFFER_BIT | gls.STENCIL_BUFFER_BIT)

	// Update the current game mode if any
	if game.currentMode != nil {
		game.currentMode.Update(game, deltaTime)
	}

	// Render scene
	err := renderer.Render(game.scene, game.camera)
	if err != nil {
		panic(err)
	}
	game.frameRater.Wait()
	game.updateFPS()
}

func (game *Game) updateFPS() {

	// Get the FPS and potential FPS from the frameRater
	fps, pfps, ok := game.frameRater.FPS(time.Duration(1000) * time.Millisecond)
	if !ok {
		return
	}

	// Show the FPS in the header label
	game.labelFPS.SetText(fmt.Sprintf("%3.1f / %3.1f", fps, pfps))
}
