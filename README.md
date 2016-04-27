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

- ode install (./configure --with-shared --with-double-precision --with-libccd)

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

# coordinate system

GL: right-hand

X: left(+)-right(-)
Y: up(+)-down(-)
Z: front(+)-back(-)

direction: (1=x, 2=y, 3=z)

# world params

world parameters:
    ERP = 0.8
    CFM = 1.0e-5
    step size = 1/100
    gravity = Y(-) 9.8m/s2

Course Setup


# ode compositions

- World 運動空間
    - Body 質点
        - Mass 質量（モーメント形状付き）
        - Data 任意のデータ

- Space 衝突空間
    - Geom 形状
        - Data 任意のデータ

# join sequence

1. open brouwser assets/index.html
2. ws connect to host/ws
3. new vehicle add to world
4. jsonrpc call "World.Update"
5. World render by WebGL
6. repeat to 4.
