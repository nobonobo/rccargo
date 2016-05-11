package models

import (
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
	sync.RWMutex
	ode.World
	Space      ode.HashSpace
	JointGroup ode.JointGroup
	Profile    protocol.Profile
	vehicles   map[string]*Vehicle
}

// NewContext ...
func NewContext(profile protocol.Profile) *Context {
	return &Context{
		World:      ode.NewWorld(),
		Space:      ode.NilSpace().NewHashSpace(),
		JointGroup: ode.NewJointGroup(10000),
		Profile:    profile,
		vehicles:   map[string]*Vehicle{},
	}
}

// Iter ...
func (ctx *Context) Iter(step time.Duration, callback ode.NearCallback) {
	ctx.Lock()
	defer ctx.Unlock()
	for _, v := range ctx.vehicles {
		v.Update()
	}
	ctx.Space.Collide(ctx, callback)
	ctx.World.QuickStep(float64(step) / float64(time.Second))
	//ctx.World.Step(float64(step) / float64(time.Second))
	ctx.JointGroup.Empty()
}

// AddVehicle ...
func (ctx *Context) AddVehicle(name string, pos []float64) *Vehicle {
	ctx.Lock()
	defer ctx.Unlock()
	if v := ctx.vehicles[name]; v != nil {
		v.Destroy()
	}
	v := NewVehicle(ctx, ctx.Profile.Vehicle)
	v.SetPosition(pos)
	ctx.vehicles[name] = v
	return v
}

// GetVehicle ...
func (ctx *Context) GetVehicle(name string) *Vehicle {
	ctx.RLock()
	defer ctx.RUnlock()
	return ctx.vehicles[name]
}

// RmVehicle ...
func (ctx *Context) RmVehicle(name string) {
	ctx.Lock()
	defer ctx.Unlock()
	if v := ctx.vehicles[name]; v != nil {
		v.Destroy()
	}
	delete(ctx.vehicles, name)
}

// IterVehicles ...
func (ctx *Context) IterVehicles(f func(string, *Vehicle)) {
	ctx.Lock()
	defer ctx.Unlock()
	for name, v := range ctx.vehicles {
		f(name, v)
	}
}
