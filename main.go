package main

import (
	"encoding/json"
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
	World: protocol.WorldProfile{
		Gravity:                []float64{0, -9.80665, 0},
		CFM:                    10e-5,
		ERP:                    0.8,
		QuickStepW:             1e-3,
		QuickStepNumIterations: 10,
		Mu:      1e-6,
		SoftCfm: 1e-6,
		SoftErp: 0.3,
	},
	Vehicle: protocol.VehicleProfile{
		BodyDensity:        0.05,
		BodyBox:            []float64{0.200, 0.050, 0.380},
		BodyZOffset:        0.0,
		Wheelbase:          0.267,
		Tread:              0.160,
		TireDensity:        0.03,
		TireDiameter:       0.088,
		TireWidth:          0.033,
		FudgeFactorJtParam: 0.01,
		SuspensionStep:     1.0,
		SuspensionSpring:   100,
		SuspensionDamping:  0.1,
	},
}

// World ...
type World struct {
	ctx      *models.Context
	vehicles map[string]*models.Vehicle
	timers   map[string]*time.Timer
}

// Join ...
func (w *World) Join(name string, rep *protocol.VehicleProfile) error {
	if w.vehicles[name] != nil {
		return fmt.Errorf("duplicated name: %s", name)
	}
	w.vehicles[name] = w.ctx.AddVehicle()
	w.timers[name] = time.AfterFunc(5*time.Second, func() {
		w.gc(name)
	})
	*rep = profile.Vehicle
	log.Println("join:", name)
	return nil
}

func (w *World) gc(name string) {
	if v := w.vehicles[name]; v != nil {
		w.ctx.RmVehicle(v)
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
	log.Println("bye:", name)
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
		pv := &protocol.Vehicle{
			Name:  name,
			Body:  protocol.Attitude{Position: v.Position(), Quaternion: v.Quaternion()},
			Tires: ws,
		}
		if req.Name == name {
			(*rep).Self = pv
			w.ctx.Lock()
			v.Update(req)
			w.ctx.Unlock()
		} else {
			(*rep).Others = append((*rep).Others, pv)
		}
	}
	if t := w.timers[req.Name]; t != nil {
		t.Reset(5 * time.Second)
	}
	return nil
}

func (w *World) handle(ws *websocket.Conn) {
	log.Println("connect:", ws.Request().RemoteAddr)
	defer log.Println("disconnect:", ws.Request().RemoteAddr)
	jsonrpc.ServeConn(ws)
}

func callback(data interface{}, obj1, obj2 ode.Geom) {
	ctx := data.(*models.Context)
	body1, body2 := obj1.Body(), obj2.Body()
	if body1 != 0 && body2 != 0 && body1.Connected(body2) {
		return
	}
	cts := obj1.Collide(obj2, 4, 0)
	for _, c := range cts {
		contact := ode.NewContact()
		contact.Surface.Mode = ode.SoftERPCtParam | ode.SoftCFMCtParam
		contact.Surface.Mu = profile.World.Mu
		contact.Surface.SoftCfm = profile.World.SoftCfm
		contact.Surface.SoftErp = profile.World.SoftErp
		contact.Geom = c
		ct := ctx.World.NewContactJoint(ctx.JointGroup, contact)
		ct.Attach(body1, body2)
	}
}

func main() {
	fp, err := os.Open("./profile.json")
	if err != nil {
		log.Fatalln(err)
	}
	if err := json.NewDecoder(fp).Decode(&profile); err != nil {
		log.Fatalln(err)
	}
	b, err := json.MarshalIndent(profile, "", "  ")
	if err != nil {
		log.Fatalln(err)
	}
	os.Stderr.Write(b)
	addr := "0.0.0.0:8080"
	l, err := net.Listen("tcp", addr)
	if err != nil {
		fmt.Println(err)
		os.Exit(1)
	}

	ctx := models.NewContext(profile)
	ctx.World.SetGravity(ode.V3(profile.World.Gravity...))
	ctx.World.SetCFM(profile.World.CFM)
	ctx.World.SetERP(profile.World.ERP)
	ctx.World.SetQuickStepW(profile.World.QuickStepW)
	ctx.World.SetQuickStepNumIterations(profile.World.QuickStepNumIterations)
	//ctx.World.SetAutoDisable(false)

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
