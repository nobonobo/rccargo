package models

import "github.com/ianremmler/ode"

func init() {
	ode.Init(0, ode.AllAFlag)
}

// Context ...
type Context struct {
	ode.World
	ode.Space
	JointGroup ode.JointGroup
	callback   ode.NearCallback
}

// NewContext ...
func NewContext(callback ode.NearCallback) *Context {
	return &Context{
		World:      ode.NewWorld(),
		Space:      ode.NilSpace().NewHashSpace(),
		JointGroup: ode.NewJointGroup(10000),
		callback:   callback,
	}
}

func (ctx *Context) Iter() {
	ctx.Space.Collide(nil, ctx.callback)
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
	body ode.Body
	geom ode.Geom
}

// Vehicle ...
type Vehicle struct {
	body   ode.Body
	geom   ode.Geom
	wheels []*Wheel
}

func NewWheel(ctx *Context, density, diameter, width float64) *Wheel {
	mass := ode.NewMass()
	mass.SetCylinder(density, 1, diameter/2, width) // 1: x-axis length = width
	body := ctx.NewBody()
	body.SetMass(mass)
	geom := ctx.NewCylinder(diameter/2, width)
	return &Wheel{body: body, geom: geom}
}

func NewVehicle(ctx *Context, profile Profile) *Vehicle {
	wheels := []*Wheel{}
	for i := 0; i < 4; i++ {
		wheels = append(wheels, NewWheel(ctx,
			profile.TireDensity,
			profile.TireDiameter,
			profile.TireWidth,
		))
	}
	mass := ode.NewMass()
	mass.SetBox(profile.BodyDensity, profile.BodyBox)
	body := ctx.NewBody()
	body.SetMass(mass)
	geom := ctx.NewBox(profile.BodyBox)
	return &Vehicle{body: body, geom: geom, wheels: wheels}
}
