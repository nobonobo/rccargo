# rccargo
RC-Car Simulator

- ODE physics
- 1/10 scale RC-Car
- Multiplayer support(WebRTC-base)

# Build

## OS-X

```sh
brew install ode --with-shared --with-double-precision
go get -u github.com/ianremmler/ode
go get -u github.com/nobonobo/rccargo
```

## Linux

- ode install (./configure --with-shared --with-double-precision)

```sh
go get -u github.com/ianremmler/ode
go get -u github.com/nobonobo/rccargo
```

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
