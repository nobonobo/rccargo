package main

import (
	"fmt"
	"log"
	"net"
	"net/http"
	"net/rpc"
	"net/rpc/jsonrpc"
	"os"

	"golang.org/x/net/websocket"

	"github.com/ianremmler/ode"
	"github.com/nobonobo/rccargo/models"
)

// World ...
type World struct {
	ctx      *models.Context
	vehicles map[string]*models.Vehicle
}

// Input ...
type Input struct {
	Steering float64 `json:"steering"`
	Accel    float64 `json:"accel"`
	Brake    float64 `json:"brake"`
}

// Attitude ...
type Attitude struct {
	Position   ode.Vector3    `json:"position"`
	Quaternion ode.Quaternion `json:"quaternion"`
}

// Vehicle ...
type Vehicle struct {
	Name  string
	Body  Attitude   `json:"body"`
	Tires []Attitude `json:"tires"`
}

// Output ...
type Output struct {
	Vehicles map[string]Vehicle
}

// Update ...
func (w *World) Update(req *Input, rep *Output) error {
	vehicles := map[string]Vehicle{}
	for name, v := range w.vehicles {
		ws := make([]Attitude, 4)
		for i := 0; i < 4; i++ {
			wheel := v.Wheel(i)
			ws[i].Position = wheel.Position()
			ws[i].Quaternion = wheel.Quaternion()
		}
		vehicles[name] = Vehicle{
			Body:  Attitude{Position: v.Position(), Quaternion: v.Quaternion()},
			Tires: ws,
		}
	}
	(*rep).Vehicles = vehicles
	return nil
}

// RPCServer ...
func RPCServer(ws *websocket.Conn) {
	fmt.Println("connect:", ws.RemoteAddr)
	defer fmt.Println("disconnect:", ws.RemoteAddr)
	jsonrpc.ServeConn(ws)
}

var profile = models.Profile{
	BodyDensity:  0.2,
	BodyBox:      ode.V3(0.200, 0.100, 0.350),
	BodyZOffset:  0.0,
	Wheelbase:    0.267,
	Tread:        0.160,
	TireDensity:  0.1,
	TireDiameter: 0.088,
	TireWidth:    0.033,
}

func main() {
	addr := "0.0.0.0:8080"
	l, err := net.Listen("tcp", addr)
	if err != nil {
		fmt.Println(err)
		os.Exit(1)
	}

	ctx := models.NewContext()
	ctx.World.SetGravity(ode.V3(0, 0, -0.5))

	world := &World{ctx: ctx, vehicles: map[string]*models.Vehicle{}}
	world.vehicles["player"] = models.NewVehicle(ctx, profile)

	rpc.Register(world)
	http.Handle("/ws", websocket.Handler(RPCServer))
	http.Handle("/", http.FileServer(http.Dir("assets")))
	log.Println("listen:", addr)
	if err := http.Serve(l, nil); err != nil {
		fmt.Println(err)
		os.Exit(1)
	}
}
