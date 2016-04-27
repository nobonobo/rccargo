package main

import (
	"fmt"

	"github.com/gopherjs/websocket"
)

func main() {
	c, err := websocket.Dial("ws://localhost:8080/ws")
	if err != nil {
		fmt.Println(err)
		return
	}
	fmt.Println(c)
}
