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
	ctx.World.QuickStep(float64(step) / float64(time.Second))
	ctx.JointGroup.Empty()
}

// AddVehicle ...
func (ctx *Context) AddVehicle(profile protocol.Profile) *Vehicle {
	ctx.Lock()
	defer ctx.Unlock()
	return NewVehicle(ctx, profile)
}

// RmVehicle ...
func (ctx *Context) RmVehicle(v *Vehicle) {
	ctx.Lock()
	defer ctx.Unlock()
	v.Destroy()
}
