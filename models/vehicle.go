package models

/*
#include <stdint.h>
*/
import "C"
import "github.com/ianremmler/ode"

func init() {
	ode.Init(0, ode.AllAFlag)
}

// Context ...
type Context struct {
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
func (ctx *Context) Iter(callback ode.NearCallback) {
	ctx.Space.Collide(ctx, callback)
	ctx.World.Step(0.05)
	ctx.JointGroup.Empty()
}

// Profile ...
type Profile struct {
	BodyDensity  float64     // default 0.2
	BodyBox      ode.Vector3 // { width, height, length }
	BodyZOffset  float64     // Offset Adjust from center of wheels
	Wheelbase    float64     // default 0.267m
	Tread        float64     // default 0.160m
	TireDensity  float64     // default 0.1
	TireDiameter float64     // default 0.088m
	TireWidth    float64     // default 0.033m
}

// Wheel ...
type Wheel struct {
	ode.Hinge2Joint
	body ode.Body
	geom ode.Geom
}

// NewWheel ...
func NewWheel(ctx *Context, vehicle *Vehicle, density, diameter, width float64) *Wheel {
	mass := ode.NewMass()
	mass.SetCylinder(density, 1, diameter/2, width) // 1: x-axis length = width
	body := ctx.NewBody()
	body.SetMass(mass)
	geom := ctx.NewCylinder(diameter/2, width)
	suspension := ctx.World.NewHinge2Joint(ode.JointGroup(0))
	suspension.Attach(vehicle.body, body)
	return &Wheel{Hinge2Joint: suspension, body: body, geom: geom}
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
func NewVehicle(ctx *Context, profile Profile) *Vehicle {
	mass := ode.NewMass()
	mass.SetBox(profile.BodyDensity, profile.BodyBox)
	body := ctx.NewBody()
	body.SetMass(mass)
	geom := ctx.NewBox(profile.BodyBox)
	v := &Vehicle{body: body, geom: geom, wheels: []*Wheel{}}
	for i := 0; i < 4; i++ {
		w := NewWheel(ctx, v,
			profile.TireDensity,
			profile.TireDiameter,
			profile.TireWidth,
		)
		v.wheels = append(v.wheels, w)
	}
	v.wheels[0].SetAnchor(ode.V3(profile.Tread/2, 0, profile.Wheelbase/2))
	v.wheels[1].SetAnchor(ode.V3(-profile.Tread/2, 0, profile.Wheelbase/2))
	v.wheels[2].SetAnchor(ode.V3(profile.Tread/2, 0, -profile.Wheelbase/2))
	v.wheels[3].SetAnchor(ode.V3(-profile.Tread/2, 0, -profile.Wheelbase/2))
	return v
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
