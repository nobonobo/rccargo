package main

import (
	"fmt"
	"log"
	"net"
	"net/http"
	"net/rpc"
	"net/rpc/jsonrpc"
	"os"
	"time"

	"golang.org/x/net/websocket"

	"github.com/ianremmler/ode"
	"github.com/nobonobo/rccargo/models"
	"github.com/nobonobo/rccargo/protocol"
)

var profile = protocol.Profile{
	BodyDensity:  0.05,
	BodyBox:      ode.V3(0.200, 0.050, 0.380),
	BodyZOffset:  0.0,
	Wheelbase:    0.267,
	Tread:        0.160 + 0.1,
	TireDensity:  0.03,
	TireDiameter: 0.088,
	TireWidth:    0.033,
}

// World ...
type World struct {
	ctx      *models.Context
	vehicles map[string]*models.Vehicle
	timers   map[string]*time.Timer
}

// Join ...
func (w *World) Join(name string, rep *protocol.Profile) error {
	if w.vehicles[name] != nil {
		return fmt.Errorf("duplicated name: %s", name)
	}
	w.ctx.Lock()
	w.vehicles[name] = models.NewVehicle(w.ctx, profile)
	w.ctx.Unlock()
	w.timers[name] = time.AfterFunc(5*time.Second, func() {
		w.gc(name)
	})
	*rep = profile
	return nil
}

func (w *World) gc(name string) {
	if v := w.vehicles[name]; v != nil {
		w.ctx.Lock()
		v.Destroy()
		w.ctx.Unlock()
	}
	if tm := w.timers[name]; tm != nil {
		tm.Stop()
	}
	delete(w.vehicles, name)
	delete(w.timers, name)
}

// Bye ...
func (w *World) Bye(name string, rep *string) error {
	w.gc(name)
	return nil
}

// Update ...
func (w *World) Update(req *protocol.Input, rep *protocol.Output) error {
	for name, v := range w.vehicles {
		ws := make([]protocol.Attitude, 4)
		for i := 0; i < 4; i++ {
			wheel := v.Wheel(i)
			ws[i].Position = wheel.Position()
			ws[i].Quaternion = wheel.Quaternion()
		}
		pv := protocol.Vehicle{
			Name:  name,
			Body:  protocol.Attitude{Position: v.Position(), Quaternion: v.Quaternion()},
			Tires: ws,
		}
		if req.Name == name {
			(*rep).Self = pv
			for i := 0; i < 4; i++ {
				wheel := v.Wheel(i)
				d := req.Steering - wheel.Joint.Angle1()
				if d > 0.1 {
					d = 0.1
				}
				if d < -0.1 {
					d = -0.1
				}
				wheel.Joint.SetParam(ode.VelJtParam, d*10.0)
				wheel.Joint.SetParam(ode.VelJtParam2, -req.Accel)
			}
			continue
		}
		(*rep).Others = append((*rep).Others, pv)
	}
	if t := w.timers[req.Name]; t != nil {
		t.Reset(5 * time.Second)
	}
	return nil
}

func (w *World) handle(ws *websocket.Conn) {
	fmt.Println("connect:", ws.Request().RemoteAddr)
	defer fmt.Println("disconnect:", ws.Request().RemoteAddr)
	jsonrpc.ServeConn(ws)
}

func callback(data interface{}, obj1, obj2 ode.Geom) {
	ctx := data.(*models.Context)
	body1, body2 := obj1.Body(), obj2.Body()
	if body1 != 0 && body2 != 0 && body1.Connected(body2) {
		return
	}
	cts := obj1.Collide(obj2, 1, 0)
	if len(cts) > 0 {
		if obj1.IsSpace() || obj2.IsSpace() {
			log.Fatalln("space!")
		}
		contact := ode.NewContact()
		contact.Surface.Mode = 0
		contact.Surface.Mu = 0.1
		contact.Surface.Mu2 = 0
		contact.Geom = cts[0]
		ct := ctx.World.NewContactJoint(ctx.JointGroup, contact)
		ct.Attach(body1, body2)
	}
}

func main() {
	addr := "0.0.0.0:8080"
	l, err := net.Listen("tcp", addr)
	if err != nil {
		fmt.Println(err)
		os.Exit(1)
	}

	ctx := models.NewContext()
	ctx.World.SetGravity(ode.V3(0, -9.80665, 0))
	//ctx.World.SetCFM(10e-5)
	//ctx.World.SetERP(0.9)
	ctx.World.SetAutoDisable(false)

	world := &World{
		ctx:      ctx,
		vehicles: map[string]*models.Vehicle{},
		timers:   map[string]*time.Timer{},
	}

	world.ctx.NewPlane(ode.V4(0, 1, 0, -0.5))

	go func() {
		for {
			time.Sleep(time.Millisecond)
			ctx.Iter(time.Millisecond, callback)
		}
	}()

	rpc.Register(world)
	http.Handle("/ws", websocket.Handler(world.handle))
	http.Handle("/", http.FileServer(http.Dir("assets")))
	log.Println("listen:", addr)
	if err := http.Serve(l, nil); err != nil {
		fmt.Println(err)
		os.Exit(1)
	}
}
