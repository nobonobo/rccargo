package models

/*
#include <stdint.h>
*/
import "C"
import (
	"fmt"
	"log"
	"sync"
	"time"

	"github.com/ianremmler/ode"
	"github.com/nobonobo/rccargo/protocol"
)

func init() {
	ode.Init(0, ode.AllAFlag)
}

// Context ...
type Context struct {
	sync.Mutex
	ode.World
	ode.Space
	JointGroup ode.JointGroup
}

// NewContext ...
func NewContext() *Context {
	return &Context{
		World:      ode.NewWorld(),
		Space:      ode.NilSpace().NewHashSpace(),
		JointGroup: ode.NewJointGroup(10000),
	}
}

// Iter ...
func (ctx *Context) Iter(step time.Duration, callback ode.NearCallback) {
	ctx.Lock()
	defer ctx.Unlock()
	ctx.Space.Collide(ctx, callback)
	ctx.World.Step(float64(step) / float64(time.Second))
	ctx.JointGroup.Empty()
}

// Wheel ...
type Wheel struct {
	Joint ode.Hinge2Joint
	body  ode.Body
	geom  ode.Geom
}

// NewWheel ...
func NewWheel(ctx *Context, density, diameter, width float64) *Wheel {
	body := ctx.World.NewBody()
	//geom := ctx.Space.NewCylinder(diameter/2, width)
	geom := ctx.Space.NewSphere(diameter / 2)
	geom.SetBody(body)
	mass := ode.NewMass()
	//mass.SetCylinder(density, 3, diameter/2, width) // 3: z-axis length = width
	mass.SetSphere(density, diameter/2)
	body.SetMass(mass)
	joint := ctx.World.NewHinge2Joint(ode.JointGroup(0))
	return &Wheel{Joint: joint, body: body, geom: geom}
}

func (w *Wheel) Destroy() {
	w.Joint.Destroy()
	w.body.Destroy()
	w.geom.Destroy()
}

func (w *Wheel) Position() ode.Vector3 {
	return w.body.Position()
}

func (w *Wheel) Quaternion() ode.Quaternion {
	return w.body.Quaternion()
}

func (w *Wheel) Rotation() ode.Matrix3 {
	return w.body.Rotation()
}

// Vehicle ...
type Vehicle struct {
	body   ode.Body
	geom   ode.Geom
	wheels []*Wheel
}

// NewVehicle ...
func NewVehicle(ctx *Context, profile protocol.Profile) *Vehicle {
	body := ctx.World.NewBody()
	geom := ctx.Space.NewBox(profile.BodyBox)
	geom.SetBody(body)
	mass := ode.NewMass()
	mass.SetBox(profile.BodyDensity, profile.BodyBox)
	body.SetMass(mass)
	v := &Vehicle{body: body, geom: geom, wheels: []*Wheel{}}
	for i := 0; i < 4; i++ {
		w := NewWheel(ctx,
			profile.TireDensity,
			profile.TireDiameter,
			profile.TireWidth,
		)
		v.wheels = append(v.wheels, w)
	}
	fmt.Println("body:", v.Quaternion())
	for i, w := range v.wheels {
		lr := -1.0
		if i%2 == 0 {
			lr = 1.0
		}
		fr := -1.0
		if i/2 == 0 {
			fr = 1.0
		}
		x := lr * profile.Tread / 2
		y := -0.05
		z := fr * profile.Wheelbase / 2
		log.Printf("wheel(%d): lr=%4.1f, fr=%4.1f, %s", i, lr, fr, fmt.Sprint(w.Quaternion()))
		w.Joint.Attach(v.body, w.body)
		w.Joint.SetAxis1(ode.V3(0, 1, 0))
		w.Joint.SetAxis2(ode.V3(0, 0, fr))
		w.Joint.SetParam(ode.FudgeFactorJtParam, 0.1)
		w.Joint.SetParam(ode.FMaxJtParam, 0.2)  // 操舵トルク最大値Nm
		w.Joint.SetParam(ode.FMaxJtParam2, 0.1) // 動輪トルク最大値Nm
		/*
			step, spring, damping := 0.05, 9.0, 0.85
			w.Joint.SetParam(ode.SuspensionCFMJtParam, step*spring/(step*spring+damping))
			w.Joint.SetParam(ode.SuspensionERPJtParam, 1.0/(step*spring+damping))
		*/
		w.Joint.SetParam(ode.SuspensionCFMJtParam, 0.8)
		w.Joint.SetParam(ode.SuspensionERPJtParam, 0.4)
		w.body.SetPosition(ode.V3(x, y, z))
		w.Joint.SetAnchor(w.Position())
	}
	return v
}

func (v *Vehicle) Destroy() {
	for _, w := range v.wheels {
		w.Destroy()
	}
	v.body.Destroy()
	v.geom.Destroy()
}

func (v *Vehicle) Position() ode.Vector3 {
	return v.body.Position()
}

func (v *Vehicle) Quaternion() ode.Quaternion {
	return v.body.Quaternion()
}

func (v *Vehicle) Rotation() ode.Matrix3 {
	return v.body.Rotation()
}

func (v *Vehicle) Wheel(index int) *Wheel {
	return v.wheels[index]
}
