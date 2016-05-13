# rccargo
RC-Car Simulator

- ODE physics
- 1/10 scale RC-Car
- Multiplayer support(now:websocket -> future:WebRTC-base)
- Gamepad or Mouse or Touch Input

DEMO: https://rccargo.arukascloud.io

# Build and Run

## OS-X

```sh
brew install ode --with-shared --with-double-precision
go get -u github.com/ianremmler/ode
go get -u github.com/nobonobo/rccargo
cd $GOPATH/src/github.com/nobonobo/rccargo; rcccargo
```

## Linux

- ode install (./configure --with-shared --with-double-precision --with-libccd)

```sh
go get -u github.com/ianremmler/ode
go get -u github.com/nobonobo/rccargo
cd $GOPATH/src/github.com/nobonobo/rccargo; rcccargo
```

# Open Browser

```sh
open http://localhost:8080/
```

# Frontend

- gopherjs base
- three.js

# Backend

- go websocket and static-files server
- world simulation by ode

# Open Dynamics Engine

Site:
    http://www.ode.org

go-wrapper:
    https://github.com/ianremmler/ode

document:
    https://godoc.org/github.com/ianremmler/ode

# Concept

- scale 1/10 only.
- one type chassis 4wd+RS540 only.
- no gimmick.
- no customize.
- player camera human-eye only.
- reply camera few pos variety.
- free-run style.
- ghost of fastest-lap.
- physics-sim-server
- webgl-renderer+player-input

# coordinate system

GL: right-hand

- X: left(-)-right(+)
- Y: front(+)-back(-)
- Z: up(+)-down(-)

# world params

world parameters:
    - ERP = 0.8
    - CFM = 1.0e-5
    - gravity = -9.8m/s2(Z)

# ode compositions

- World(motion-space)
    - Body
        - Mass
        - Data

- Space(collision space)
    - Geom
        - Data

# join and update sequence

1. open brouwser assets/index.html
2. ws connect to host/ws
3. new vehicle add to world
4. jsonrpc call "World.Update"
5. World render by WebGL
6. repeat to 4.
