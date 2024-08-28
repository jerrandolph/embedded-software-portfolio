package main

import (
	"fmt"
	"log"
	"math"
	"os"
	"time"

	influxdb2 "github.com/influxdata/influxdb-client-go"
	"github.com/influxdata/influxdb-client-go/api/write"
	"go.bug.st/serial"
)

const (
	uartPort     = "/dev/ttyACM1"
	uartBaudRate = 115200
	uartDataBits = 8
)

// packet contains a single instance of accelerometer data and the amount of power the controller wants to send to the
// DC motor. Note, while powerVal can be larger than +1 or smaller tha -1, in the MSP-430 the PWM signal is capped to +1
// or -1. +1 corresponds to 100% duty cycle with a +/- poliarity, and -1 corresponds to 100% duty cycle with a -/+ polarity.
type packet struct {
	accelerometerVal int16
	powerVal         float32
}

// streamUARTData creates a channel that will collect accelerometer and DC motor data from a serial channel.
// streamUARTData continually pulls UART serial data from "/
func streamUARTData() <-chan packet {
	ch := make(chan packet, 128)

	go func() {
		defer func() {
			close(ch)
		}()
		port, err := serial.Open(uartPort, &serial.Mode{
			BaudRate: uartBaudRate,
			DataBits: uartDataBits,
		})
		if err != nil {
			panic(err)
		}
		port.SetReadTimeout(-1)
		buf := make([]byte, 512)

		// Each packet consts of the hex value 0xBEEF, then a 16 bit integer, then a 32 bit
		// floating point number. It's possible to end up with an incomplete packet at the
		// end of the buffer. In that case, we prepend the leftovers to the start of the buffer.
		// "offset" is used to keep track of how many bytes in buf are actually from the previous
		// call to port.Read.
		offset := 0

		for {
			// Unprocessed bytes have (potentially) been prepended to buf.
			n, err := port.Read(buf[offset:])
			if err != nil {
				fmt.Printf("error: %v\n", err)
				return
			}
			i := 0
			for ; i < n-8; i++ {
				// Look for 0xBEEF, 6 bytes of data, and then another 0xBEEF
				if buf[i] == 0xef && buf[i+1] == 0xbe && buf[i+8] == 0xef && buf[i+9] == 0xbe {
					break
				}
			}
			if i == n {
				fmt.Println("Couldn't find 0xBEEF sentinel")
				return
			}
			for ; i < n-7; i += 8 {
				// Igore i and i+1 (that's the 0xBEEF sentinel)

				// The MSP430 is sending the LSB first.
				accVal := int16(buf[i+2]) + int16(buf[i+3])<<8
				powerVal := math.Float32frombits(uint32(buf[i+4]) +
					uint32(buf[i+5])<<8 +
					uint32(buf[i+6])<<16 +
					uint32(buf[i+7])<<24)

				ch <- packet{
					accelerometerVal: accVal,
					powerVal:         powerVal,
				}
			}
			// Make sure we process the unprocessed bytes in the next pass
			if i != n {
				copy(buf, buf[i:n])
				offset = n - i
			}
		}
	}()
	return ch
}

func main() {
	token := os.Getenv("INFLUXDB_TOKEN")
	url := "http://localhost:8086"

	client := influxdb2.NewClient(url, token)
	writeAPI := client.WriteAPI("Self", "MSP430")

	ch := streamUARTData()

	// Re-use these maps so we're not constantly creating them in the for loop below.
	accelerometerField := map[string]any{}
	powerField := map[string]any{}
	actualPowerField := map[string]any{}

	go func() {
		err := <-writeAPI.Errors()
		log.Fatalln("writeAPI failure: ", err)
	}()
	for packet := range ch {
		// Use the same timestamp for all these data points
		now := time.Now()

		accelerometerField["value"] = packet.accelerometerVal
		accPoint := write.NewPoint("accelerometer", nil, accelerometerField, now)
		writeAPI.WritePoint(accPoint)

		powerField["value"] = packet.powerVal
		powerPoint := write.NewPoint("dc_motor_power", nil, powerField, now)
		writeAPI.WritePoint(powerPoint)

		actualPowerField["value"] = max(min(packet.powerVal, 1), -1)
		actualPowerPoint := write.NewPoint("real_dc_motor_power", nil, actualPowerField, now)
		writeAPI.WritePoint(actualPowerPoint)
	}
	fmt.Println("done")
}
