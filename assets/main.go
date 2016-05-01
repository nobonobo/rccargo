package main

import (
	"fmt"
	"math"
	"net/rpc"
	"net/rpc/jsonrpc"

	"github.com/gopherjs/gopherjs/js"
	"github.com/gopherjs/websocket"
	"github.com/nobonobo/rccargo/protocol"
)

var (
	document  = js.Global.Get("document")
	navigator = js.Global.Get("navigator")
	window    = js.Global.Get("window")
	THREE     = js.Global.Get("THREE")
)

// Start ...
func Start(c *rpc.Client) {
	axes := []func() float64{
		func() float64 { return 0.0 },
		func() float64 { return 0.0 },
		func() float64 { return 0.0 },
	}
	if f := navigator.Get("getGamepads"); f != js.Undefined {
		joystick := navigator.Call("getGamepads").Index(0)
		if joystick != js.Undefined {
			get := func() *js.Object {
				return navigator.Call("getGamepads").Index(0)
			}
			axes[0] = func() float64 {
				return get().Get("axes").Index(0).Float()
			}
			axes[1] = func() float64 {
				return get().Get("axes").Index(3).Float() * -1
			}
			axes[2] = func() float64 {
				return get().Get("buttons").Index(0).Get("value").Float()
			}
		}
	}
	w, h := js.Global.Get("innerWidth").Float(), js.Global.Get("innerHeight").Float()
	//w, h := 640.0, 480.0
	renderer := THREE.Get("WebGLRenderer").New(map[string]interface{}{})
	renderer.Call("setSize", w, h)
	renderer.Call("setClearColor", 0x000000, 1)
	scene := THREE.Get("Scene").New()

	//cameraPan := 0.0
	//cameraTilt := 0.0
	cameraZoom := 20.0
	camera := THREE.Get("PerspectiveCamera").New(cameraZoom, w/h, 0.1, 100)
	camera.Get("position").Call("set", 0.1, -0.4, -1.0)
	scene.Call("add", camera)
	scene.Call("add", THREE.Get("AmbientLight").New(0xffffff))

	window.Call("addEventListener", "resize", func() {
		w, h := js.Global.Get("innerWidth").Float(), js.Global.Get("innerHeight").Float()
		renderer.Call("setSize", w, h)
		camera.Set("aspect", w/h)
		camera.Call("updateProjectionMatrix")
	}, false)

	geometry := THREE.Get("PlaneGeometry").New(20, 20, 32, 32)
	geometry.Call("rotateX", -math.Pi/2)
	material := THREE.Get("MeshBasicMaterial").New(
		map[string]interface{}{"color": 0x404040, "wireframe": true},
	)
	plane := THREE.Get("Mesh").New(geometry, material)
	plane.Get("position").Set("y", -0.5)
	scene.Call("add", plane)

	element := renderer.Get("domElement")
	document.Get("body").Call("appendChild", element)
	stats := js.Global.Get("Stats").New()
	stats.Call("showPanel", 0) // 0: fps, 1: ms, 2: mb, 3+: custom
	document.Get("body").Call("appendChild", stats.Get("dom"))

	profile := protocol.VehicleProfile{}
	name := ""
	for i := 1; i <= 20; i++ {
		name = fmt.Sprintf("player%d", i)
		if err := c.Call("World.Join", name, &profile); err != nil {
			fmt.Println("rpc failed:", err)
			continue
		} else {
			break
		}
	}
	if name == "" {
		fmt.Println("join failed")
		return
	}
	window.Call("addEventListener", "unload", func() {
		res := ""
		c.Go("World.Bye", name, &res, nil)
	}, false)
	fmt.Println("profile:", profile)
	build := func(name string) {
		geometry := THREE.Get("BoxGeometry").New(
			profile.BodyBox[0],
			profile.BodyBox[1],
			profile.BodyBox[2],
		)
		material := THREE.Get("MeshBasicMaterial").New(
			map[string]interface{}{"color": 0xffffff},
		)
		body := THREE.Get("Mesh").New(geometry, material)
		body.Set("name", name)
		scene.Call("add", body)
		for i := 0; i < 4; i++ {
			geometry := THREE.Get("CylinderGeometry").New(
				profile.TireDiameter/2, profile.TireDiameter/2,
				profile.TireWidth, 32,
			)
			geometry.Call("rotateX", math.Pi/2)
			if i%2 == 0 {
				geometry.Call("rotateZ", math.Pi/2)
			} else {
				geometry.Call("rotateZ", -math.Pi/2)
			}
			material := THREE.Get("MeshBasicMaterial").New(
				map[string]interface{}{"color": 0x8080ff, "wireframe": true},
			)
			tire := THREE.Get("Mesh").New(geometry, material)
			tire.Set("name", fmt.Sprintf("%s-tire%d", name, i))
			scene.Call("add", tire)
		}
	}
	//build(name)
	steering := axes[0]()
	accel, brake := axes[1](), axes[2]()
	sx, sy := 0.0, 0.0
	mouse := false
	element.Call("addEventListener", "mousedown", func(ev *js.Object) {
		sx = ev.Get("clientX").Float()
		sy = ev.Get("clientY").Float()
	}, false)
	element.Call("addEventListener", "mousemove", func(ev *js.Object) {
		which := ev.Get("which").Int()
		if which == 1 {
			mouse = true
			dx := (ev.Get("clientX").Float() - sx) / 200.0
			dy := (ev.Get("clientY").Float() - sy) / 200.0
			if dx < -1 {
				dx = -1
			}
			if dx > 1 {
				dx = 1
			}
			if dy < -1 {
				dy = -1
			}
			if dy > 1 {
				dy = 1
			}
			steering = dx
			if dy < 0.0 {
				accel = -dy
				brake = 0.0
			} else {
				accel = 0.0
				brake = dy
			}
		} else {
			mouse = false
			steering = axes[0]()
			accel, brake = axes[1](), axes[2]()
		}
	}, false)
	element.Call("addEventListener", "mouseup", func(ev *js.Object) {
		mouse = false
		steering = axes[0]()
		accel, brake = axes[1](), axes[2]()
	}, false)
	element.Call("addEventListener", "mouseout", func(ev *js.Object) {
		mouse = false
		steering = axes[0]()
		accel, brake = axes[1](), axes[2]()
	}, false)

	update := func() error {
		if !mouse {
			steering = axes[0]()
			accel, brake = axes[1](), axes[2]()
		}
		in := protocol.Input{
			Name:     name,
			Steering: steering,
			Accel:    accel,
			Brake:    brake,
		}
		res := &protocol.Output{}
		if err := c.Call("World.Update", in, &res); err != nil {
			return err
		}
		move := func(v *protocol.Vehicle) {
			for i, w := range v.Tires {
				tire := scene.Call("getObjectByName", fmt.Sprintf("%s-tire%d", v.Name, i))
				tire.Get("position").Call("set",
					w.Position[0],
					w.Position[1],
					w.Position[2],
				)
				tire.Get("quaternion").Call("set",
					w.Quaternion[1],
					w.Quaternion[2],
					w.Quaternion[3],
					w.Quaternion[0],
				)
			}
			body := scene.Call("getObjectByName", v.Name)
			body.Get("position").Call("set",
				v.Body.Position[0],
				v.Body.Position[1],
				v.Body.Position[2],
			)
			body.Get("quaternion").Call("set",
				v.Body.Quaternion[1],
				v.Body.Quaternion[2],
				v.Body.Quaternion[3],
				v.Body.Quaternion[0],
			)
		}
		for _, vehicle := range append(res.Others, res.Self) {
			if vehicle == nil {
				continue
			}
			body := scene.Call("getObjectByName", vehicle.Name)
			if body == js.Undefined {
				build(vehicle.Name)
			}
			move(vehicle)
		}
		if res.Self != nil {
			pos := res.Self.Body.Position
			camera.Call("lookAt", THREE.Get("Vector3").New(pos[0], pos[1], pos[2]))
		}
		return nil
	}

	ch := make(chan struct{})
	go func() {
		defer fmt.Println("update goroutine exit")
		errorCnt := 0
		for {
			if _, ok := <-ch; !ok {
				return
			}
			if err := update(); err != nil {
				errorCnt++
				if errorCnt >= 10 {
					return
				}
			} else {
				errorCnt = 0
			}
		}
	}()

	var render func()
	render = func() {
		stats.Call("begin")
		select {
		case ch <- struct{}{}:
		default:
		}
		renderer.Call("render", scene, camera)
		stats.Call("end")
		js.Global.Call("requestAnimationFrame", render)
	}
	render()
}

func main() {
	c, err := websocket.Dial("ws://localhost:8080/ws")
	if err != nil {
		fmt.Println(err)
	}
	Start(jsonrpc.NewClient(c))
}
