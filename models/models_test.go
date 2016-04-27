package models

import (
	"testing"

	"github.com/ianremmler/ode"
)

func callback(data interface{}, obj1, obj2 ode.Geom) {
	ctx := data.(*Context)
	contact := ode.NewContact()
	body1, body2 := obj1.Body(), obj2.Body()
	if body1 != 0 && body2 != 0 && body1.Connected(body2) {
		return
	}
	contact.Surface.Mode = 0
	contact.Surface.Mu = 0.1
	contact.Surface.Mu2 = 0
	cts := obj1.Collide(obj2, 1, 0)
	if len(cts) > 0 {
		contact.Geom = cts[0]
		ct := ctx.World.NewContactJoint(ctx.JointGroup, contact)
		ct.Attach(body1, body2)
	}
}

func TestVehicle(t *testing.T) {
	ctx := NewContext()
	ctx.World.SetGravity(ode.V3(0, 0, -0.5))

	profile := Profile{
		BodyDensity:  0.2,
		BodyBox:      ode.V3(0.200, 0.100, 0.350),
		BodyZOffset:  0.0,
		Wheelbase:    0.267,
		Tread:        0.160,
		TireDensity:  0.1,
		TireDiameter: 0.088,
		TireWidth:    0.033,
	}
	v := NewVehicle(ctx, profile)
	t.Log(v)
	for i := 0; i < 1000; i++ {
		ctx.Iter(callback)
	}
}
