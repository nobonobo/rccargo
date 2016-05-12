package models

/*
#include <stdint.h>
*/
import "C"
import (
	"math"

	mgl "github.com/go-gl/mathgl/mgl64"
	"github.com/ianremmler/ode"

	"github.com/nobonobo/rccargo/protocol"
)

// Wheel ...
type Wheel struct {
	Joint ode.Hinge2Joint
	body  ode.Body
	geom  ode.Geom
}

// NewWheel ...
func NewWheel(ctx *Context, density, diameter, width float64) *Wheel {
	body := ctx.World.NewBody()
	body.SetRotation(ode.NewMatrix3(
		0.0, 0.0, 1.0,
		-1.0, 0.0, 0.0,
		0.0, -1.0, 0.0,
	))
	mass := ode.NewMass()
	mass.SetCylinder(density, 1, diameter/2, width) // 1: x-axis length = width
	body.SetMass(mass)
	geom := ctx.Space.NewCylinder(diameter/2, width)
	geom.SetBody(body)
	joint := ctx.World.NewHinge2Joint(ode.JointGroup(0))
	return &Wheel{Joint: joint, body: body, geom: geom}
}

func (w *Wheel) Destroy() {
	w.Joint.Destroy()
	w.geom.Destroy()
	w.body.Destroy()
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
	body      ode.Body
	geom      ode.Geom
	wheels    []*Wheel
	tread     float64
	wheelbase float64
	accel     float64
	brake     float64
	steering  float64
}

// NewVehicle ...
func NewVehicle(ctx *Context, profile protocol.VehicleProfile) *Vehicle {
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
	v.tread = profile.Tread
	v.wheelbase = profile.Wheelbase
	camber := 15.0 // deg
	for i, w := range v.wheels {
		w.Joint.Attach(v.body, w.body)
		ax1 := mgl.HomogRotate3DX(mgl.DegToRad(camber)).Mul4x1(mgl.Vec4{0, 0, -1})
		w.Joint.SetAxis1(ode.V3(ax1[0], ax1[1], ax1[2]))
		w.Joint.SetAxis2(ode.V3(1, 0, 0))
		w.Joint.SetParam(ode.FudgeFactorJtParam, profile.FudgeFactorJtParam)
		w.Joint.SetParam(ode.FMaxJtParam, 1.0) // 操舵トルク最大値Nm
		if i/2 == 0 {
			w.Joint.SetParam(ode.LoStopJtParam, -1.0) // 操舵最小角
			w.Joint.SetParam(ode.HiStopJtParam, 1.0)  // 操舵最大角
		} else {
			w.Joint.SetParam(ode.LoStopJtParam, 0.0) // 操舵最小角
			w.Joint.SetParam(ode.HiStopJtParam, 0.0) // 操舵最大角
		}

		k := profile.SuspensionStep * profile.SuspensionSpring
		base := k + profile.SuspensionDamping
		w.Joint.SetParam(ode.SuspensionCFMJtParam, k/base)
		w.Joint.SetParam(ode.SuspensionERPJtParam, 1.0/base)
		lr := 1.0
		if i%2 == 0 {
			lr = -1.0
		}
		fr := -1.0
		if i/2 == 0 {
			fr = 1.0
		}
		x := lr * v.tread / 2
		y := fr * v.wheelbase / 2
		z := -0.025
		w.body.SetPosition(ode.V3(x, y, z))
		w.Joint.SetAnchor(w.Position())
	}
	return v
}

func (v *Vehicle) Destroy() {
	for _, w := range v.wheels {
		w.Destroy()
	}
	v.geom.Destroy()
	v.body.Destroy()
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

func (v *Vehicle) Update(dt float64) {
	for _, wheel := range v.wheels[:2] {
		d := (v.steering / 3.0) - wheel.Joint.Angle1()
		if d > 2*math.Pi {
			d = 2 * math.Pi
		}
		if d < -2*math.Pi {
			d = -2 * math.Pi
		}
		wheel.Joint.SetParam(ode.VelJtParam, d*8*dt*1e3)
	}
}

func (v *Vehicle) Set(in *protocol.Input) {
	v.steering = -in.Steering
	for i, wheel := range v.wheels {
		if in.Brake > 0.5 {
			brake := (in.Brake-0.5)*2 - in.Accel
			// 動輪目標速度rad/s
			wheel.Joint.SetParam(ode.VelJtParam2, 0.0)
			// 動輪トルク最大値Nm
			wheel.Joint.SetParam(ode.FMaxJtParam2, brake*1e-3)
		} else {
			factor := 0.55
			if i < 2 {
				factor = 0.45
			}
			// 動輪目標速度rad/s
			wheel.Joint.SetParam(ode.VelJtParam2, 160.0)
			// 動輪トルク最大値Nm
			wheel.Joint.SetParam(ode.FMaxJtParam2, factor*in.Accel*1e-3)
		}
	}
}

func (v *Vehicle) SetPosition(pos ode.Vector3) {
	for i, w := range v.wheels {
		lr := 1.0
		if i%2 == 0 {
			lr = -1.0
		}
		fr := -1.0
		if i/2 == 0 {
			fr = 1.0
		}
		x := pos[0] + lr*v.tread/2
		y := pos[1] + fr*v.wheelbase/2
		z := pos[2] - 0.025
		w.body.SetPosition(ode.V3(x, y, z))
	}
	v.body.SetPosition(pos)
}
