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
	document = js.Global.Get("document")
	window   = js.Global.Get("window")
	THREE    = js.Global.Get("THREE")
)

func start(c *rpc.Client) {
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
	camera.Get("position").Call("set", 0.0, -0.4, -1.0)
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

	profile := protocol.Profile{}
	name := "player"
	if err := c.Call("World.Join", name, &profile); err != nil {
		fmt.Println("rpc failed:", err)
		return
	}
	window.Call("addEventListener", "unload", func() {
		res := ""
		if err := c.Call("World.Bye", name, &res); err != nil {
			fmt.Println("rpc failed:", err)
		}
	}, false)
	fmt.Println("profile:", profile)
	func() {
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
	}()

	var render func()
	render = func() {
		stats.Call("begin")
		go func() {
			res := &protocol.Output{}
			if err := c.Call("World.Update", protocol.Input{Name: "player"}, &res); err != nil {
				fmt.Println("rpc failed:", err)
				return
			}
			for i, w := range res.Self.Tires {
				tire := scene.Call("getObjectByName", fmt.Sprintf("%s-tire%d", res.Self.Name, i))
				tire.Get("position").Call("set", w.Position[0], w.Position[1], w.Position[2])
				tire.Get("quaternion").Call("set", w.Quaternion[0], w.Quaternion[1], w.Quaternion[2], w.Quaternion[3])
			}
			body := scene.Call("getObjectByName", res.Self.Name)
			body.Get("position").Call("set",
				res.Self.Body.Position[0],
				res.Self.Body.Position[1],
				res.Self.Body.Position[2],
			)
			body.Get("quaternion").Call("set",
				res.Self.Body.Quaternion[0],
				res.Self.Body.Quaternion[1],
				res.Self.Body.Quaternion[2],
				res.Self.Body.Quaternion[3],
			)
			pos := res.Self.Body.Position
			camera.Call("lookAt", THREE.Get("Vector3").New(pos[0], pos[1], pos[2]))
		}()
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
	start(jsonrpc.NewClient(c))
}
