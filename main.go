package main

import (
	"log"
	"os"

	"github.com/gaoliveira21/nes/core"
)

func main() {
	bus := new(core.Bus)
	bus.Init()

	bin, err := os.ReadFile("./bin/6502_functional_test.bin")

	if err != nil {
		log.Fatal(err)
	}

	for i := 0x0000; i <= 0xFFFF; i++ {
		bus.Ram[i] = bin[i]
	}

	bus.Cpu.Pc = 0x0400

	for {
		bus.Cpu.Exec()
	}
}
