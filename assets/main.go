package main

import (
	"fmt"

	"github.com/gopherjs/gopherjs/js"
	"github.com/gopherjs/websocket"
)

var (
	document = js.Global.Get("document")
	THREE    = js.Global.Get("THREE")
)

func setup() {
	w, h := js.Global.Get("innerWidth").Float(), js.Global.Get("innerHeight").Float()
	//w, h := 640.0, 480.0
	renderer := THREE.Get("WebGLRenderer").New(map[string]interface{}{})
	renderer.Call("setSize", w, h)
	renderer.Call("setClearColor", 0x000000, 1)
	scene := THREE.Get("Scene").New()
	//cameraPan := 0.0
	//cameraTilt := 0.0
	cameraZoom := 50.0
	camera := THREE.Get("PerspectiveCamera").New(cameraZoom, w/h)
	camera.Get("position").Call("set", 0.0, 0.0, 0.0)
	scene.Call("add", camera)
	scene.Call("add", THREE.Get("AmbientLight").New(0xffffff))
}

func main() {
	setup()
	c, err := websocket.Dial("ws://localhost:8080/ws")
	if err != nil {
		fmt.Println(err)
		return
	}
	fmt.Println(c)
}
